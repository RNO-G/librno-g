#include "flower.h"
#include <linux/spi/spidev.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <endian.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <string.h>

#define NUM_SCALER_REGS 2*(3+(RNO_G_NUM_LT_CHANNELS+1)*3+(RNO_G_NUM_LT_BEAMS+1)*3)

typedef enum
{
  FLWR_REG_FW_VER = 0x01,
  FLWR_REG_FW_DATE = 0x02,
  FLWR_REG_SCAL_RD = 0x03,
  FLWR_REG_META_EVENT_COUNT =0x0a,
  FLWR_REG_META_TRIG_COUNT =0x0b,
  FLWR_REG_META_PPS_COUNT =0x0c,
  FLWR_REG_META_TRIG_TIME_LOW =0x0d,
  FLWR_REG_META_TRIG_TIME_HIGH =0x0e,
  FLWR_REG_META_TRIG_INFO =0x0f,
  FLWR_REG_DATA_STATUS = 0x07,
  FLWR_REG_DATA_CHUNK0 = 0x23,
  FLWR_REG_DATA_CHUNK1 = 0x24,
  FLWR_REG_SCAL_UPD = 0x28,
  FLWR_REG_SCAL_SEL = 0x29,
  FLWR_REG_CALPULSE=0x2a,
  FLWR_REG_SCAL_TIME_LOW = 0x2c,
  FLWR_REG_SCAL_TIME_HIGH = 0x2d,
  FLWR_REG_DATVALID=0x3a,
  FLWR_REG_PD_REG=0x3a,
  FLWR_REG_CFG_REG0 = 0x3b,
  FLWR_REG_CFG_REG1 = 0x3c,
  FLWR_REG_TRIG_ENABLES = 0x3d,
  FLWR_REG_FORCE_TRIG = 0x40,
  FLWR_REG_CHANNEL = 0x41,
  FLWR_REG_MODE = 0x42,
  FLWR_REG_RAM_ADDR = 0x45,
  FLWR_REG_READ = 0x47,
  FLWR_REG_CLR_READOUT = 0x48,
  FLWR_REG_PRETRIG= 0x4C,
  FLWR_REG_BUF_CLEAR= 0x4D,
  FLWR_REG_PHASED_MASK = 0x50,
  FLWR_REG_PHASED_THRESHOLD_OFFSET = 0x51,
  FLWR_REG_TRIGOUT_SYSOUT = 0x5c,
  FLWR_REG_TRIGOUT_AUXOUT = 0x5d,
  FLWR_REG_TRIG_CH0_THR = 0x57,
  FLWR_REG_TRIG_CH1_THR = 0x58,
  FLWR_REG_TRIG_CH2_THR = 0x59,
  FLWR_REG_TRIG_CH3_THR = 0x5a,
  FLWR_REG_TRIG_PARAM  = 0x5b,
  FLWR_REG_PPS_DELAY = 0x5e, 
  FLWR_REG_TRIG_COINC_MASK = 0x5f,
  FLWR_REG_SET_READ_REG = 0x6d,
  FLWR_REG_PHASED_THRESHOLDS = 0x80,
  FLWR_REG_MAX=0xff
} e_flower_reg; 

typedef enum
{
  HMCAD_ADR_DUAL_CGAIN = 0x2B,
  HMCAD_ADR_CGAIN_CFG = 0x33
} e_hmcad_reg;

struct flower_dev
{
  int spi_fd;
  union
  {
    struct
    {
      uint8_t addr;
      uint8_t station;
      uint8_t major;
      uint8_t minor;
    } ver;
    flower_word_t word;
  } fwver;
  int fwver_int;

  union
  {
    struct
    {
      uint16_t year;
      uint8_t day;
      uint8_t month;
    } date;
    flower_word_t word;
  } fwdate;

  rno_g_lt_simple_trigger_config_t coinc_trig_cfg; 
  rno_g_lt_phased_trigger_config_t phased_trig_cfg; 
  uint8_t coinc_trig_thresh[4]; 
  uint8_t coinc_servo_thresh[4]; 

  uint16_t phased_trig_thresh[RNO_G_NUM_LT_BEAMS]; 
  uint16_t phased_servo_thresh[RNO_G_NUM_LT_BEAMS]; 

  int must_clear; 
  int force_trigger_preclear;

};

static int export_gpio_if_not_exported(int gpionum)
{
  char buf[128];
  sprintf(buf, "/sys/class/gpio/gpio%d", gpionum);
  if (access(buf, F_OK))
  {
    //export the GPIO, lazy programming
    char cmdbuf[128];
    sprintf(cmdbuf,"echo %d > /sys/class/gpio/export", gpionum);
    system(cmdbuf);
    usleep(100000); //wait to make sure it come up
    return access(buf, F_OK);
  }
      return 0;
}


flower_dev_t * flower_open(const char * spi_device, int spi_en_gpio)
{
  flower_dev_t * dev = 0;
  int locked_spi, spi_fd;

  spi_fd = open(spi_device, O_RDWR);
  if (spi_fd < 0)
  {
    fprintf(stderr,"flower_open: Could not open %s\n", spi_device);
    return 0;
  }

  locked_spi = flock(spi_fd, LOCK_EX | LOCK_NB);
  if (locked_spi < 0)
  {
    fprintf(stderr,"flower_open: could not get exclusive access to %s\n", spi_device);
    close(spi_fd);
    return 0;
  }

  dev = calloc(1,sizeof(*dev));
  if (!dev)
  {
    fprintf(stderr,"Could not allocate memory for flower\n");
    return 0;
  }
  dev->spi_fd = spi_fd;

  int spi_clock = 16000000;
  uint8_t mode = 0;
  uint8_t bits_per_word = 8;
  ioctl(spi_fd, SPI_IOC_WR_MODE,&mode);
  ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ,&spi_clock);
  ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD,&bits_per_word);

  if (spi_en_gpio)
  {
    char buf[512];
    int gpionum = abs(spi_en_gpio);
    export_gpio_if_not_exported(gpionum);

    //make sure it's NOT active low
    sprintf(buf,"echo 0 > /sys/class/gpio/gpio%d/active_low", gpionum);
    system(buf);

    //make sure it's an output
    sprintf(buf,"echo out > /sys/class/gpio/gpio%d/direction", gpionum);
    system(buf);

    //make sure it has the right value
    sprintf(buf,"echo %d > /sys/class/gpio/gpio%d/value", spi_en_gpio < 0 ? 0 : 1, gpionum);
    system(buf);

  }

  flower_read_register(dev, FLWR_REG_FW_VER, &dev->fwver.word);
  flower_read_register(dev, FLWR_REG_FW_DATE, &dev->fwdate.word);
  dev->fwver_int = 1000 * dev->fwver.ver.major + dev->fwver.ver.minor;
  //have to finagle the date
  flower_word_t word = dev->fwdate.word;
  dev->fwdate.date.day = word.bytes[3];
  dev->fwdate.date.month = word.bytes[2] & 0xf;
  dev->fwdate.date.year = (((uint32_t)word.bytes[1]) << 4) | (word.bytes[2] >> 4);

  //read in the current thresholds
  flower_word_t thresh_word;
  for (int i = 0; i < 4; i++)
  {
    flower_read_register(dev, FLWR_REG_TRIG_CH0_THR+i, &thresh_word); 
    dev->coinc_trig_thresh[i] = thresh_word.bytes[3]; 
    dev->coinc_servo_thresh[i] = thresh_word.bytes[2]; 
  }

  
  //read in the trigger configuration 
  flower_word_t cfg_word; 
  flower_read_register(dev,FLWR_REG_TRIG_PARAM, &cfg_word); 
  dev->coinc_trig_cfg.vpp_mode = cfg_word.bytes[1]; 
  dev->coinc_trig_cfg.window = cfg_word.bytes[2];  
  dev->coinc_trig_cfg.num_coinc = cfg_word.bytes[3]; 

  if (dev->fwver_int>=9)
  {  
    //these will need to be moved into a new fw check when implemented
    //flower_read_register(dev,FLWR_REG_TRIG_COINC_MASK, &cfg_word); 
    //dev->coinc_trig_cfg.channel_mask=cfg_word.bytes[3]; //trig config would need updated to include thi
    dev->coinc_trig_cfg.channel_mask=0xf;

    for (int i = 0; i < RNO_G_NUM_LT_BEAMS; i++) 
    {
      flower_read_register(dev, FLWR_REG_PHASED_THRESHOLDS+i, &thresh_word);  
      dev->phased_trig_thresh[i] = ((thresh_word.bytes[2]&0xf)<<8)+thresh_word.bytes[3]; 
      dev->phased_servo_thresh[i] = (thresh_word.bytes[1]<<4)+((thresh_word.bytes[2]&0xf0)>>4); 
    }
    flower_read_register(dev,FLWR_REG_PHASED_MASK, &cfg_word); 
    dev->phased_trig_cfg.beam_mask = cfg_word.bytes[3]+(cfg_word.bytes[2]<<8);
    flower_read_register(dev,FLWR_REG_PHASED_MASK+1, &cfg_word); 
    dev->phased_trig_cfg.phased_threshold_offset = cfg_word.bytes[3]+(cfg_word.bytes[2]<<8);
  }

  return dev; 
}

static int write_words(flower_dev_t *dev, int N,  const flower_word_t * words)
{
  if (!dev) return -1;
  return ((int) (N*sizeof(*words))) != write(dev->spi_fd, words, N*sizeof(*words));
}
static int write_word(flower_dev_t *dev, const flower_word_t * word)
{

  if (!dev) return -1;
  return ((int)sizeof(*word)) != write(dev->spi_fd, word, sizeof(*word));
}

/*
static int read_word (flower_dev_t * dev, flower_word_t *word)
{
  return ((int)sizeof(*word)) != read(dev->spi_fd, word, sizeof(*word));
}
*/

int flower_read_register(flower_dev_t*dev, uint8_t addr, flower_word_t * result)
{
  if (addr <1 || !dev) return -1;
  struct spi_ioc_transfer xfer[2] = {0};
  flower_word_t word = {0} ;
  word.bytes[0] = FLWR_REG_SET_READ_REG ;
  word.bytes[3] = addr;
  xfer[0].tx_buf = (uintptr_t)  word.bytes;
  xfer[0].len = 4;
  xfer[0].rx_buf = 0;
  xfer[1].tx_buf = 0;
  xfer[1].len = 4;
  xfer[1].rx_buf =  (uintptr_t) result->bytes;

  return 2*sizeof(word) != ioctl(dev->spi_fd, SPI_IOC_MESSAGE(2), xfer);
}

int flower_set_phased_thresholds(flower_dev_t *dev, const uint16_t * phased_trigger_thresholds, const uint16_t * phased_servo_thresholds, uint16_t mask) 
{
  if (!dev || dev->fwver_int<9) return -1; 

  flower_word_t words[RNO_G_NUM_LT_BEAMS] = {0}; 

  int ii = 0; 

  for (int i = 0; i < RNO_G_NUM_LT_BEAMS; i++) 
  {
    if (mask & (1 << i)) 
    {
      uint16_t phased_servo = phased_servo_thresholds[i]; 
      uint16_t phased_trig = phased_trigger_thresholds[i]; 
      if (phased_servo > 4095) phased_servo = 4095; 
      if (phased_trig > 4095) phased_trig = 4095; 
      words[ii].bytes[0] = FLWR_REG_PHASED_THRESHOLDS+i; 
      words[ii].bytes[1] = (phased_servo&0xff0)>>4; 

      words[ii].bytes[2] = ((phased_trig&0xf00)>>8)+((phased_servo&0xf)<<4); 
      words[ii].bytes[3] = phased_trig&0xff; 
      dev->phased_trig_thresh[i] = phased_trig;
      dev->phased_servo_thresh[i] = phased_servo;
      ii++; 
    }

  }
  return write_words(dev, ii, words); 
}

int flower_set_coinc_thresholds(flower_dev_t *dev, const uint8_t * trigger_thresholds, const uint8_t * servo_thresholds, uint8_t mask) 
{
  if (!dev) return -1;

  flower_word_t words[4] = {0};

  int ii = 0;
  for (int i = 0; i < 4; i++)
  {
    if (mask & (1 << i))
    {
      uint8_t servo = servo_thresholds[i]; 
      uint8_t trig = trigger_thresholds[i]; 
      if (servo > 127) servo = 127; 
      if (trig > 127) trig = 127; 
      words[ii].bytes[0] = FLWR_REG_TRIG_CH0_THR+i; 
      words[ii].bytes[2] = servo; 
      words[ii].bytes[3] = trig; 
      dev->coinc_trig_thresh[i] = trig;
      dev->coinc_servo_thresh[i] = servo;
      ii++; 
    }
  }
  return write_words(dev, ii, words);
}

int flower_configure_trigger(flower_dev_t * dev, rno_g_lt_simple_trigger_config_t  cfg, rno_g_lt_phased_trigger_config_t phased_cfg) 
{
  if (!dev) return -1;
  int ret = 0;
  flower_word_t word = {0};
  word.bytes[0] = FLWR_REG_TRIG_PARAM;
  word.bytes[1] = cfg.vpp_mode; 
  word.bytes[2] = cfg.window; 
  word.bytes[3] = cfg.num_coinc; 
  ret = write_word(dev,&word); 
  if (!ret) dev->coinc_trig_cfg = cfg; 

  /* not implemented yet but leaving software so commenting write/read
  if(dev->fwver_int>=10)
  {  
    word.bytes[0] = FLWR_REG_TRIG_COINC_MASK;
    word.bytes[1] = 0; 
    word.bytes[2] = 0; 
    word.bytes[3] = cfg.channel_mask; 
    ret = write_word(dev,&word); 
    if (!ret) dev->coinc_trig_cfg = cfg; 
  }
  */

  if(dev->fwver_int>=9)
  {
    word.bytes[0] = FLWR_REG_PHASED_MASK;
    word.bytes[1] = 0; 
    word.bytes[2] = (phased_cfg.beam_mask&0xff00)>>8;
    word.bytes[3] = phased_cfg.beam_mask&0xff; 
    ret += write_word(dev,&word); 
    if (!ret) dev->phased_trig_cfg = phased_cfg; 

    word.bytes[0] = FLWR_REG_PHASED_THRESHOLD_OFFSET+1;
    word.bytes[1] = 0; 
    word.bytes[2] = (phased_cfg.phased_threshold_offset&0xff00)>>8;
    word.bytes[3] = phased_cfg.phased_threshold_offset&0xff; 
    ret += write_word(dev,&word); 
    if (!ret) dev->phased_trig_cfg = phased_cfg; 
  }


  return ret; 
}

int flower_fill_header(flower_dev_t * dev, rno_g_header_t * hd)
{
  if (!dev) return -1; 
  hd->lt_simple_trigger_cfg = dev->coinc_trig_cfg; 
  hd->lt_phased_trigger_cfg = dev->phased_trig_cfg;
  return 0; 
}

int flower_close(flower_dev_t * dev)
{
  if (!dev) return -1;
  flock(dev->spi_fd, LOCK_UN);
  close(dev->spi_fd);
  free(dev);
  return 0;
}

static flower_word_t scal_sel_regs[NUM_SCALER_REGS]; 
__attribute__((constructor)) 
static void fill_scal_sel_regs() 
{
  for (int i = 0; i < NUM_SCALER_REGS; i++) 
  {
    scal_sel_regs[i].bytes[0] = FLWR_REG_SCAL_SEL;
    scal_sel_regs[i].bytes[3] = i;
  }
}

int flower_fill_daqstatus(flower_dev_t *dev, rno_g_daqstatus_t *ds)
{

  if (!dev) return -1;

  #define MAX_DSNMSG (3*(NUM_SCALER_REGS)+5)

  struct spi_ioc_transfer xfer[MAX_DSNMSG] = {0};

  static flower_word_t update_word = {.bytes = {FLWR_REG_SCAL_UPD,0,0,1}};
  static flower_word_t selectread_word = {.bytes = {FLWR_REG_SET_READ_REG,0,0, FLWR_REG_SCAL_RD}};
  static flower_word_t update_tlow = {.bytes = {FLWR_REG_SET_READ_REG,0,0,FLWR_REG_SCAL_TIME_LOW}}; 
  static flower_word_t update_thigh = {.bytes = {FLWR_REG_SET_READ_REG,0,0,FLWR_REG_SCAL_TIME_HIGH}}; 
  flower_word_t dest_scaler[NUM_SCALER_REGS] = {0}; 
  uint16_t raw_scalers[2*NUM_SCALER_REGS]; 
  flower_word_t dest_time[2] = {0}; 

  struct timespec start;
  struct timespec end;


  for (int i = 0; i < 4; i++)
  {
    ds->lt_coinc_trigger_thresholds[i] = dev->coinc_trig_thresh[i];
    ds->lt_coinc_servo_thresholds[i] = dev->coinc_servo_thresh[i];
  }

  //these don't need version checking since it doesn't do the read (should be 0 in dev)
  for (int i = 0; i < RNO_G_NUM_LT_BEAMS; i++) 
  {
    ds->lt_phased_trigger_thresholds[i] = dev->phased_trig_thresh[i];
    ds->lt_phased_servo_thresholds[i] = dev->phased_servo_thresh[i];
  }
  ds->lt_phased_threshold_offset=dev->phased_trig_cfg.phased_threshold_offset;

  //TODO can speed this up by interlacing reads and writes and caching the first part

  xfer[0].tx_buf = (uintptr_t) update_word.bytes;
  xfer[0].len = sizeof(flower_word_t);

  xfer[1].tx_buf  = (uintptr_t) update_tlow.bytes;
  xfer[1].len = sizeof(flower_word_t);
  xfer[2].rx_buf  = (uintptr_t) dest_time[0].bytes;
  xfer[2].len = sizeof(flower_word_t);

  xfer[3].tx_buf  =  (uintptr_t)update_thigh.bytes;
  xfer[3].len = sizeof(flower_word_t);
  xfer[4].rx_buf  = (uintptr_t) dest_time[1].bytes;
  xfer[4].len = sizeof(flower_word_t);

  int ixfer = 0; 
  int max_reg;
  if(dev->fwver_int<8) max_reg=32;
  else if(dev->fwver_int==8) max_reg=34;
  else max_reg=NUM_SCALER_REGS; //dev->fwver_int==9

  for (int ireg = 0; ireg <max_reg; ireg++) 
  {
    //if (ireg == 18) ireg=31; //scalers 36-61 are empty
    xfer[3*ixfer+5].tx_buf = (uintptr_t) scal_sel_regs[ireg].bytes; 
    xfer[3*ixfer+5].len = sizeof(flower_word_t);
    xfer[3*ixfer+5].rx_buf = 0;
    xfer[3*ixfer+6].tx_buf =  (uintptr_t)selectread_word.bytes;
    xfer[3*ixfer+6].rx_buf = 0;
    //xfer[3*ixfer+6].cs_change = 1;
    xfer[3*ixfer+6].len = sizeof(flower_word_t);
    xfer[3*ixfer+7].rx_buf =  (uintptr_t )dest_scaler[ireg].bytes; // will have to finagle these after
    xfer[3*ixfer+7].len = sizeof(flower_word_t);
    xfer[3*ixfer+7].tx_buf =  0;
    ixfer++;
  }

  int nxfer;// =  (dev->fwver_int < 8) ? 3*19+5 : 3*21+5; 
  if(dev->fwver_int <8) nxfer=3*19+5;
  else if (dev->fwver_int == 8) nxfer=3*21+5;
  else nxfer=3*NUM_SCALER_REGS+5; //dev->fwver_int==9

  clock_gettime(CLOCK_REALTIME,&start);
  int ret = ioctl(dev->spi_fd, SPI_IOC_MESSAGE(nxfer), xfer);
  clock_gettime(CLOCK_REALTIME,&end);
  //  printf("status ioctl: %d\n", ret);

  ds->when_lt = (start.tv_sec*0.5 + end.tv_sec*0.5) + 1e-9*(start.tv_nsec*0.5 + end.tv_nsec*0.5);

  if (ret > 0)
  {
    for (int i = 0; i < NUM_SCALER_REGS; i++) 
    {
      uint16_t low =  dest_scaler[i].bytes[3] | ((dest_scaler[i].bytes[2] & 0x0f ) << 8) ;
      uint16_t high = (dest_scaler[i].bytes[1] << 4) | ((dest_scaler[i].bytes[2] & 0xf0)>>4); 
      raw_scalers[2*i] = low;
      raw_scalers[2*i+1] = high;
    }
    
    ds->lt_scalers.cycle_counter = 0;
    
    //these need different version
    if(dev->fwver_int>=9)
    {
      ds->lt_scalers.s_1Hz.trig_coinc = raw_scalers[6];
      for (int i = 0; i < 4; i++) ds->lt_scalers.s_1Hz.trig_per_chan[i] = raw_scalers[7+i]; 
      ds->lt_scalers.s_1Hz.servo_coinc = raw_scalers[11];
      for (int i = 0; i < 4; i++) ds->lt_scalers.s_1Hz.servo_per_chan[i] = raw_scalers[12+i]; 
      ds->lt_scalers.s_1Hz_gated.trig_coinc = raw_scalers[16];
      for (int i = 0; i < 4; i++) ds->lt_scalers.s_1Hz_gated.trig_per_chan[i] = raw_scalers[17+i]; 
      ds->lt_scalers.s_1Hz_gated.servo_coinc = raw_scalers[21];
      for (int i = 0; i < 4; i++) ds->lt_scalers.s_1Hz_gated.servo_per_chan[i] = raw_scalers[22+i]; 
      ds->lt_scalers.s_100Hz.trig_coinc = raw_scalers[26];
      for (int i = 0; i < 4; i++) ds->lt_scalers.s_100Hz.trig_per_chan[i] = raw_scalers[27+i]; 
      ds->lt_scalers.s_100Hz.servo_coinc = raw_scalers[31];
      for (int i = 0; i < 4; i++) ds->lt_scalers.s_100Hz.servo_per_chan[i] = raw_scalers[32+i]; 

      #define PHASED_SCALER_BASE 36
      ds->lt_scalers.s_1Hz.trig_phased = raw_scalers[PHASED_SCALER_BASE];
      for (int i = 0; i < RNO_G_NUM_LT_BEAMS; i++) ds->lt_scalers.s_1Hz.trig_per_beam[i] = raw_scalers[PHASED_SCALER_BASE+1+i]; 
      ds->lt_scalers.s_1Hz.servo_phased = raw_scalers[PHASED_SCALER_BASE+1+RNO_G_NUM_LT_BEAMS];
      for (int i = 0; i < RNO_G_NUM_LT_BEAMS; i++) ds->lt_scalers.s_1Hz.servo_per_beam[i] = raw_scalers[PHASED_SCALER_BASE+2+RNO_G_NUM_LT_BEAMS+i]; 
      ds->lt_scalers.s_1Hz_gated.trig_phased = raw_scalers[PHASED_SCALER_BASE+2+2*RNO_G_NUM_LT_BEAMS];
      for (int i = 0; i < RNO_G_NUM_LT_BEAMS; i++) ds->lt_scalers.s_1Hz_gated.trig_per_beam[i] = raw_scalers[PHASED_SCALER_BASE+3+2*RNO_G_NUM_LT_BEAMS+i]; 
      ds->lt_scalers.s_1Hz_gated.servo_phased = raw_scalers[PHASED_SCALER_BASE+3+3*RNO_G_NUM_LT_BEAMS];
      for (int i = 0; i < RNO_G_NUM_LT_BEAMS; i++) ds->lt_scalers.s_1Hz_gated.servo_per_beam[i] = raw_scalers[PHASED_SCALER_BASE+4+3*RNO_G_NUM_LT_BEAMS+i]; 
      ds->lt_scalers.s_100Hz.trig_phased = raw_scalers[PHASED_SCALER_BASE+4+4*RNO_G_NUM_LT_BEAMS];
      for (int i = 0; i < RNO_G_NUM_LT_BEAMS; i++) ds->lt_scalers.s_100Hz.trig_per_beam[i] = raw_scalers[PHASED_SCALER_BASE+5+4*RNO_G_NUM_LT_BEAMS+i]; 
      ds->lt_scalers.s_100Hz.servo_phased = raw_scalers[PHASED_SCALER_BASE+5+5*RNO_G_NUM_LT_BEAMS];
      for (int i = 0; i < RNO_G_NUM_LT_BEAMS; i++) ds->lt_scalers.s_100Hz.servo_per_beam[i] = raw_scalers[PHASED_SCALER_BASE+6+5*RNO_G_NUM_LT_BEAMS+i]; 
      
      ds->lt_scalers.scaler_counter_1Hz = raw_scalers[0]; 

      uint64_t cyc_low =( be32toh(dest_scaler[1].word) & 0xffffff);  
      uint64_t cyc_high =( be32toh(dest_scaler[2].word) & 0xffffff);  
      ds->lt_scalers.cycle_counter = cyc_low  | (cyc_high << 24); 

    }
    else //dev->fwver_int<9
    {
      ds->lt_scalers.s_1Hz.trig_coinc = raw_scalers[0];
      for (int i = 0; i < 4; i++) ds->lt_scalers.s_1Hz.trig_per_chan[i] = raw_scalers[1+i];
      ds->lt_scalers.s_1Hz.servo_coinc = raw_scalers[5];
      for (int i = 0; i < 4; i++) ds->lt_scalers.s_1Hz.servo_per_chan[i] = raw_scalers[6+i];
      ds->lt_scalers.s_1Hz_gated.trig_coinc = raw_scalers[12];
      for (int i = 0; i < 4; i++) ds->lt_scalers.s_1Hz_gated.trig_per_chan[i] = raw_scalers[13+i];
      ds->lt_scalers.s_1Hz_gated.servo_coinc = raw_scalers[12+5];
      for (int i = 0; i < 4; i++) ds->lt_scalers.s_1Hz_gated.servo_per_chan[i] = raw_scalers[18+i];
      ds->lt_scalers.s_100Hz.trig_coinc = raw_scalers[24];
      for (int i = 0; i < 4; i++) ds->lt_scalers.s_100Hz.trig_per_chan[i] = raw_scalers[25+i];
      ds->lt_scalers.s_100Hz.servo_coinc = raw_scalers[24+5];
      for (int i = 0; i < 4; i++) ds->lt_scalers.s_100Hz.servo_per_chan[i] = raw_scalers[30+i];

      ds->lt_scalers.scaler_counter_1Hz = raw_scalers[63]; 
        
      uint64_t cyc_low =( be32toh(dest_scaler[32].word) & 0xffffff);  
      uint64_t cyc_high =( be32toh(dest_scaler[33].word) & 0xffffff);  
      ds->lt_scalers.cycle_counter = cyc_low  | (cyc_high << 24); 
    }
  
    uint64_t t_low = ( be32toh(dest_time[0].word) & 0xffffff ); 
    uint64_t t_high = ( be32toh(dest_time[1].word) & 0xffffff ); 
    ds->lt_scalers.ncycles =  t_low | t_high << 24; 
    
    //printf("scaler 0x20: %x %x %x %x\n", dest_scaler[32].bytes[0], dest_scaler[32].bytes[1], dest_scaler[32].bytes[2], dest_scaler[32].bytes[3]); 
    //printf("scaler 0x21: %x %x %x %x\n", dest_scaler[33].bytes[0], dest_scaler[33].bytes[1], dest_scaler[33].bytes[2], dest_scaler[33].bytes[3]); 

    return 0;
  }
  return ret ?: -1;
}

int flower_dump(FILE * f, flower_dev_t *dev)
{
  int ret = 0;
  ret+= fprintf(f,"FLOWER HANDLE at 0x%p\n", dev);
  if (!dev)
  {
    fprintf(f,"  NULL HANDLE!!!\n");
    return -1;
  }
  ret+= fprintf(f,"  FWVER:  %02d.%02d.%02d (0x%x, [0x%x,0x%x,0x%x,0x%x])\n",
                dev->fwver.ver.station, dev->fwver.ver.major, dev->fwver.ver.minor,
                dev->fwver.word.word,
                dev->fwver.word.bytes[0], dev->fwver.word.bytes[1],
                dev->fwver.word.bytes[2], dev->fwver.word.bytes[3]);
  ret+= fprintf(f,"  FWDATE:  %d-%02d-%02d (0x%x, [0x%x,0x%x,0x%x,0x%x])\n",
                dev->fwdate.date.year, dev->fwdate.date.month, dev->fwdate.date.day,
                dev->fwdate.word.word,
                dev->fwdate.word.bytes[0], dev->fwdate.word.bytes[1], 
                dev->fwdate.word.bytes[2], dev->fwdate.word.bytes[3]); 
  ret+= fprintf(f,"  TRIGCONFIG:  window: %d, num_coinc: %d, vpp_mode: %d, channel_mask: %d\n", 
                dev->coinc_trig_cfg.window, dev->coinc_trig_cfg.num_coinc, dev->coinc_trig_cfg.vpp_mode, dev->coinc_trig_cfg.channel_mask); 

  ret+= fprintf(f,"  PHASEDTRIGCONFIG:  mask: %d, threshold_offset %d \n", 
                dev->phased_trig_cfg.beam_mask,dev->phased_trig_cfg.phased_threshold_offset); 

  for (int i = 0; i < 4; i++) 
  {
     ret+= fprintf(f,"  THRESH_CH%d:  servo:  %d, trig: %d\n", i, dev->coinc_servo_thresh[i], dev->coinc_trig_thresh[i]);
  }

    for (int i = 0; i < RNO_G_NUM_LT_BEAMS; i++) 
  {
     ret+= fprintf(f,"  THRESH_BEAM%d:  servo:  %d, trig: %d\n", i, dev->phased_servo_thresh[i], dev->phased_trig_thresh[i]);
  }
 

 
  return ret; 
}

static const flower_word_t buffer_clear = {.bytes={FLWR_REG_BUF_CLEAR,0,0,1}};
int flower_buffer_clear(flower_dev_t * dev)
{
  return write_word(dev, &buffer_clear);
}


static const flower_word_t sw_trig_low = {.bytes={FLWR_REG_FORCE_TRIG,0,0,0}};
static const flower_word_t sw_trig_high = {.bytes={FLWR_REG_FORCE_TRIG,0,0,1}};
static flower_word_t sw_trig_high_with_preclear[2]  = { buffer_clear, sw_trig_high };

int flower_force_trigger(flower_dev_t * dev)
{
  if (!dev) return -1;
  int ret = 0;

  if (dev->fwver_int < 6)
  {
    ret += write_word(dev, &sw_trig_low);
  }

  if (dev->force_trigger_preclear)
  {
    ret += write_words(dev, 2, sw_trig_high_with_preclear);
  }
  else
  {
    ret += write_word(dev, &sw_trig_high);
  }

  if (dev->fwver_int < 6)
  {
    dev->must_clear = 1;
  }

  return ret;
}

int flower_buffer_check(flower_dev_t * dev, int * avail)
{
  flower_word_t check;
  int ret = flower_read_register(dev, FLWR_REG_DATA_STATUS, &check);
  if (!ret && avail)  *avail = check.bytes[3] & 0x1;
  return ret;
}

int flower_read_waveforms(flower_dev_t *dev, int len, uint8_t ** dest)
{

  if (!dev) return -1;

  static flower_word_t select_chip[2]  =
  { {.bytes={FLWR_REG_CHANNEL, 0,0,1}}
  , {.bytes={FLWR_REG_CHANNEL, 0,0,2}} };

  static flower_word_t select_data[2]  =
  { {.bytes={FLWR_REG_DATA_CHUNK0, 0,0,0}}
  , {.bytes={FLWR_REG_DATA_CHUNK1, 0,0,0}} };


  static flower_word_t select_addr[256] = {0};

  if (!select_addr[0].bytes[0])
  {
      for (int i = 0; i < 256; i++)
      {
        select_addr[i].bytes[0] =FLWR_REG_RAM_ADDR;
        select_addr[i].bytes[3] =i;
      }
  }

  if (len > 1024) len = 1024;

  int ret = 0;
  for (int ichip = 0; ichip < 2; ichip++)
  {
    ret += write_word(dev, select_chip+ichip);
    //let's do 256 samples at a time to avoid the spi ioctl limit
    //we need 5 commands per address (set address, 2xread first, 2xread second, to avoid duplexing)
    struct spi_ioc_transfer xfer[320] = {0};
    int isamp = 0;
    while (isamp < len)
    {
      int howmany = isamp + 256 > len ? len -isamp: 256;
      int nxfers = ceil(howmany/4);
      for (int ixfer = 0; ixfer < nxfers; ixfer++)
      {
        xfer[ixfer*5].tx_buf = (uintptr_t) select_addr[isamp>>2].bytes;
        xfer[ixfer*5].rx_buf = 0;
        xfer[ixfer*5].len = 4;
        //xfer[ixfer*5].delay_usecs = 100;
        xfer[ixfer*5].cs_change = 1;
        xfer[ixfer*5+1].tx_buf = (uintptr_t) select_data[0].bytes;
        xfer[ixfer*5+1].rx_buf = 0;
        xfer[ixfer*5+1].len = 4;
        xfer[ixfer*5+1].cs_change = 1;
        xfer[ixfer*5+2].rx_buf = (uintptr_t) (&dest[2*ichip][isamp]);
        xfer[ixfer*5+2].tx_buf =0;
        xfer[ixfer*5+2].len = 4;
        xfer[ixfer*5+2].cs_change = 1;
        xfer[ixfer*5+3].tx_buf = (uintptr_t) select_data[1].bytes;
        xfer[ixfer*5+3].rx_buf = 0;
        xfer[ixfer*5+3].len = 4;
        xfer[ixfer*5+3].cs_change = 1;
        xfer[ixfer*5+4].rx_buf = (uintptr_t) (&dest[2*ichip+1][isamp]);
        xfer[ixfer*5+4].tx_buf =0;
        xfer[ixfer*5+4].len = 4;
        xfer[ixfer*5+4].cs_change = 1;
        //xfer[ixfer*5+4].delay_usecs = 100;
        isamp+=4;
      }

      ioctl(dev->spi_fd, SPI_IOC_MESSAGE(nxfers*5), xfer);
    }
  }

  if (dev->must_clear && dev->fwver_int < 6)
  {
    ret+= write_word(dev, &sw_trig_low);
  }
  return ret;
}

int flower_read_waveform_metadata(flower_dev_t * dev, flower_waveform_metadata_t * meta)
{
  if (!meta) return 1;

  flower_word_t ev_counter = {0};
  flower_word_t trig_counter = {0};
  flower_word_t pps_counter = {0};
  flower_word_t trig_type = {0};
  flower_word_t time_low = {0};
  flower_word_t time_high = {0};
  flower_word_t latched_time_low = {0};
  flower_word_t latched_time_high = {0};

  struct spi_ioc_transfer xfer[] =
  {
    { .len = 4, .tx_buf = (uintptr_t) (uint8_t[]){ FLWR_REG_SCAL_UPD, 0,0,1}}, //latch recent PPS (also updates scalers
    { .len = 4, .tx_buf = (uintptr_t) (uint8_t[]){ FLWR_REG_SET_READ_REG, 0,0,FLWR_REG_META_EVENT_COUNT}},
    { .len = 4, .rx_buf = (uintptr_t) ev_counter.bytes},
    { .len = 4, .tx_buf = (uintptr_t) (uint8_t[]){ FLWR_REG_SET_READ_REG, 0,0,FLWR_REG_META_TRIG_COUNT}},
    { .len = 4, .rx_buf = (uintptr_t) trig_counter.bytes},
    { .len = 4, .tx_buf = (uintptr_t) (uint8_t[]){ FLWR_REG_SET_READ_REG, 0,0,FLWR_REG_META_PPS_COUNT}},
    { .len = 4, .rx_buf = (uintptr_t) pps_counter.bytes},
    { .len = 4, .tx_buf = (uintptr_t) (uint8_t[]){ FLWR_REG_SET_READ_REG, 0,0,FLWR_REG_META_TRIG_INFO}},
    { .len = 4, .rx_buf = (uintptr_t) trig_type.bytes},
    { .len = 4, .tx_buf = (uintptr_t) (uint8_t[]){ FLWR_REG_SET_READ_REG, 0,0,FLWR_REG_META_TRIG_TIME_LOW}},
    { .len = 4, .rx_buf = (uintptr_t) time_low.bytes},
    { .len = 4, .tx_buf = (uintptr_t) (uint8_t[]){ FLWR_REG_SET_READ_REG, 0,0,FLWR_REG_META_TRIG_TIME_HIGH}},
    { .len = 4, .rx_buf = (uintptr_t) time_high.bytes},
    { .len = 4, .tx_buf = (uintptr_t) (uint8_t[]){ FLWR_REG_SET_READ_REG, 0,0,FLWR_REG_SCAL_TIME_LOW}},
    { .len = 4, .rx_buf = (uintptr_t) latched_time_low.bytes},
    { .len = 4, .tx_buf = (uintptr_t) (uint8_t[]){ FLWR_REG_SET_READ_REG, 0,0,FLWR_REG_SCAL_TIME_HIGH}},
    { .len = 4, .rx_buf = (uintptr_t) latched_time_high.bytes}
  };

  const int nxfer = sizeof (xfer)/sizeof(*xfer);

  int nioctl = ioctl(dev->spi_fd, SPI_IOC_MESSAGE(nxfer), xfer);
  if (nioctl < 0 || nioctl != sizeof(flower_word_t) * nxfer)
  {
    fprintf(stderr,"in flower_read_waveform_metadata: ioctl returned %d, less than %d expected\n", nioctl, 4 * nxfer);
    return -1;
  }

  if (
      (ev_counter.bytes[0] & 0xf)!= FLWR_REG_META_EVENT_COUNT ||
      trig_counter.bytes[0] != FLWR_REG_META_TRIG_COUNT ||
      trig_type.bytes[0] != FLWR_REG_META_TRIG_INFO ||
      time_low.bytes[0] != FLWR_REG_META_TRIG_TIME_LOW ||
      time_high.bytes[0] != FLWR_REG_META_TRIG_TIME_HIGH ||
      latched_time_low.bytes[0] != FLWR_REG_SCAL_TIME_LOW ||
      latched_time_high.bytes[0] != FLWR_REG_SCAL_TIME_HIGH
      )
  {
    fprintf(stderr,"wrong first byte in a register!!!\n");
    return -1;
  }

  meta->event_counter = be32toh(ev_counter.word) & 0xffffff;
  meta->trigger_counter = be32toh(trig_counter.word) & 0xffffff;
  meta->trigger_type = trig_type.bytes[3];
  meta->pps_counter = be32toh(pps_counter.word) & 0xffffff;
  meta->pps_flag = trig_type.bytes[1];
  meta->timestamp = be32toh(time_high.word) & 0xffffff;
  meta->timestamp <<= 24;
  meta->timestamp |= be32toh(time_low.word) & 0xffffff;
  meta->recent_pps_timestamp = be32toh(latched_time_high.word) & 0xffffff;
  meta->recent_pps_timestamp <<= 24;
  meta->recent_pps_timestamp |= be32toh(latched_time_low.word) & 0xffffff;

  return 0;
};


int flower_set_gains(flower_dev_t *dev, const uint8_t * codes)
{

  if (!dev) return -1;
  for (int ichip = 0; ichip < 2; ichip++)
  {
    //select chip
    flower_word_t word = {.bytes={FLWR_REG_CFG_REG0,0,0,ichip}};
    write_word(dev,&word);

    //make sure in xcfg
    word.bytes[0] = FLWR_REG_CFG_REG1;
    word.bytes[1] = HMCAD_ADR_CGAIN_CFG;
    word.bytes[3] = 1;
    write_word(dev,&word);

    //set gains for 2 channels
    word.bytes[1] = HMCAD_ADR_DUAL_CGAIN;
    word.bytes[3] = (codes[2*ichip] & 0xf)  | ((codes[2*ichip+1] & 0xf) << 4);
    write_word(dev,&word);
  }
  return 0;
}

int flower_set_fine_gains(flower_dev_t *dev, uint8_t * sub_numerators)
{
  flower_word_t word_low_channels = {.bytes = {140, 0, sub_numerators[1] & 0x1f,  sub_numerators[0] & 0x1f}};
  flower_word_t word_high_channels = {.bytes = {141, 0, sub_numerators[3] & 0x1f,  sub_numerators[2] & 0x1f}};

  return write_word(dev,&word_low_channels) + write_word(dev,&word_high_channels);
}

static double getrms(int N, uint8_t* X)
{
  double sum = 0;
  double sum2 = 0;
  for (int i = 0; i < N ; i++)
  {
    sum+=X[i];
    sum2+=X[i]*X[i];
  }

  double mean = sum/N;
  return sqrt(sum2/N - mean*mean);
}


int flower_set_trigger_enables(flower_dev_t *dev, flower_trigger_enables_t enables)
{
  flower_word_t word = {.bytes = {FLWR_REG_TRIG_ENABLES, enables.enable_ext, (enables.enable_phased<<1)+enables.enable_coinc, enables.enable_pps }}; 
  return write_word(dev,&word); 
}


int flower_set_trigout_enables(flower_dev_t * dev, flower_trigout_enables_t enables)
{
  flower_word_t word1 = {.bytes={FLWR_REG_TRIGOUT_SYSOUT,0,enables.enable_pps_sysout,enables.enable_rf_sysout}};
  flower_word_t word2 = {.bytes={FLWR_REG_TRIGOUT_AUXOUT,0,enables.enable_pps_auxout,enables.enable_rf_auxout}};
  return write_word(dev, &word1) + write_word(dev, &word2);
}

int flower_equalize(flower_dev_t * dev, float target_rms, uint8_t * v_gain_codes, int opts, int do_fine_gain_adjust, uint8_t * v_fine_gain_number)
{
  if (!dev) return -1;

  float rms[RNO_G_NUM_LT_CHANNELS] = {0};
  float gain_remainder[RNO_G_NUM_LT_CHANNELS] = {0};
  uint8_t mask = (~(opts & 0xf)) & 0xf;
  int verbose = opts & 0x80000000;
  static uint8_t data[RNO_G_NUM_LT_CHANNELS][1024];
  static uint8_t * data_ptrs[RNO_G_NUM_LT_CHANNELS] = { data[0], data[1], data[2], data[3] };
  uint8_t gain_codes[RNO_G_NUM_LT_CHANNELS] = {0};
  uint8_t done = 0;
  uint8_t num_waveforms = 10;
  uint8_t sub_numerators[RNO_G_NUM_LT_CHANNELS] = {0};

  while (done != mask)
  {

    for (int i = 0; i < RNO_G_NUM_LT_CHANNELS; i++)
    {
      if (done & ( 1 << i) || !(mask & (1 << i))) continue;

      rms[i] = 0;
    }

    flower_set_gains(dev, gain_codes);
    int avail = 0;

    for(int w = 0; w < num_waveforms; w++)
    {
      flower_buffer_clear(dev);
      flower_force_trigger(dev);
      while (!avail) flower_buffer_check(dev,&avail);
      flower_read_waveforms(dev, 1024, data_ptrs);

      for (int i = 0; i < RNO_G_NUM_LT_CHANNELS; i++)
      {
        if (done & ( 1 << i) || !(mask & (1 << i))) continue;

        rms[i] += getrms(1024, data[i])/num_waveforms;
      }

    }

    for (int i = 0; i < RNO_G_NUM_LT_CHANNELS; i++)
    {

      if (done & ( 1 << i) || !(mask & (1 << i))) continue;

      if (verbose) printf("ch: %d, gain_code: %d, rms: %f\n", i, gain_codes[i], rms[i]);

      if (rms[i] < target_rms && gain_codes[i] < FLOWER_GAIN_50X)
      {
        gain_codes[i]++;
      }
      else
      {
        done |= (1 << i);
        if (v_gain_codes) v_gain_codes[i] = gain_codes[i];
        if (verbose) printf("  ch %d done!\n", i);
      }
    }
  }

  if (dev->fwver_int>16)
  {
    if (do_fine_gain_adjust)
    {
      for(int ch = 0; ch<RNO_G_NUM_LT_CHANNELS; ch++)
      {
        gain_remainder[ch] = target_rms/rms[ch]*64;

        if (gain_remainder[ch] > 64) gain_remainder[ch] = 64;
        if (gain_remainder[ch] < 33) gain_remainder[ch] = 33;

        sub_numerators[ch] = (int)(64-gain_remainder[ch]);
        rms[ch] = rms[ch]*(64-sub_numerators[ch])/64;
        if (v_fine_gain_number) v_fine_gain_number[ch] = sub_numerators[ch];
      }
    }

    flower_set_fine_gains(dev, sub_numerators);
  }

  if (verbose) 
  {
    printf("Ending RMS values: cho %f, ch1 %f, ch2 %f, ch3 %f\n",rms[0],rms[1],rms[2],rms[3]);
    printf("Set gain codes: ch0 %i, ch1 %i, ch2 %i, ch3 %i\n",gain_codes[0],gain_codes[1],gain_codes[2],gain_codes[3]);
    printf("Set fine gain codes: ch0 %i, ch1 %i, ch2 %i, ch3 %i\n",sub_numerators[0],sub_numerators[1],sub_numerators[2],sub_numerators[3]);
  }

  return 0;
}


int flower_get_fwversion(flower_dev_t *dev, uint8_t *station, uint8_t *major, uint8_t *minor,
                         uint16_t *year, uint8_t *month, uint8_t *day)
{

  if (!dev) return -1;
  if (station) *station = dev->fwver.ver.station;
  if (major) *major = dev->fwver.ver.major;
  if (minor) *minor = dev->fwver.ver.minor;
  if (year) *year = dev->fwdate.date.year;
  if (month) *month = dev->fwdate.date.month;
  if (day) *day = dev->fwdate.date.day;
  return 0;

}

int flower_get_fwversion_int(flower_dev_t *dev, int *version_int)
{
  if (!dev) return -1;
  if (version_int) *version_int = dev->fwver_int;
  return 0;
}

int flower_set_delayed_pps_delay(flower_dev_t * dev, uint32_t delay)
{
  if (!dev || (dev->fwver_int < 8)) return -1;

  flower_word_t word = {.bytes = {FLWR_REG_PPS_DELAY, (delay >> 16) & 0xff,(delay >> 8) & 0xff,  delay & 0xff, }};
  return write_word(dev,&word);
}

int flower_get_delayed_pps_delay(flower_dev_t * dev, uint32_t *delay)
{
  if (!dev || (dev->fwver_int < 8)) return -1;
  flower_word_t word;
  int ret = flower_read_register(dev, FLWR_REG_PPS_DELAY, &word);
  if (!ret)  *delay = word.bytes[3] | (word.bytes[2] <<8) | (word.bytes[1] << 16);
  return ret;

}

const char * flower_trigger_type_as_string(uint8_t type)
{
  if (type >= FLOWER_TRIG_INVALID) type = FLOWER_TRIG_INVALID;
  const char * types[] = {"NONE","SOFT","EXT","COINC","PHASED","PPS","INVALID"};
  return types[type];
}

void flower_enable_force_trigger_preclear(flower_dev_t * dev, int preclear)
{
  if (dev) dev->force_trigger_preclear = preclear;
}

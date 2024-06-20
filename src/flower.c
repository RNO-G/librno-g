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


typedef enum
{
  FLWR_REG_FW_VER = 0x01,
  FLWR_REG_FW_DATE = 0x02,
  FLWR_REG_SCAL_RD = 0x03,
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
  FLWR_REG_TRIGOUT_SYSOUT = 0x5c,
  FLWR_REG_TRIGOUT_AUXOUT = 0x5d,
  FLWR_REG_TRIG_CH0_THR = 0x57,
  FLWR_REG_TRIG_CH1_THR = 0x58,
  FLWR_REG_TRIG_CH2_THR = 0x59,
  FLWR_REG_TRIG_CH3_THR = 0x5a,
  FLWR_REG_TRIG_PARAM  = 0x5b,
  FLWR_REG_PPS_DELAY = 0x5e,
  FLWR_REG_SET_READ_REG = 0x6d,
  FLWR_REG_MAX=0x7f
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
      uint8_t major;
      uint8_t reserved;
      uint8_t rev : 4;
      uint8_t minor : 4;
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

  rno_g_lt_simple_trigger_config_t trig_cfg;
  uint8_t trig_thresh[4];
  uint8_t servo_thresh[4];
  int must_clear;

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

  dev = calloc(sizeof(*dev),1);
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
  dev->fwver_int = 10000 * dev->fwver.ver.major + 100 * dev->fwver.ver.minor + dev->fwver.ver.rev;
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
    dev->trig_thresh[i] = thresh_word.bytes[3];
    dev->servo_thresh[i] = thresh_word.bytes[2];
  }

  //read in the trigger configuration
  flower_word_t cfg_word;
  flower_read_register(dev,FLWR_REG_TRIG_PARAM, &cfg_word);
  dev->trig_cfg.vpp_mode = cfg_word.bytes[1];
  dev->trig_cfg.window = cfg_word.bytes[2];
  dev->trig_cfg.num_coinc = cfg_word.bytes[3];
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
  if (addr <1 || addr > FLWR_REG_MAX || !dev) return -1;
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

int flower_set_thresholds(flower_dev_t *dev, const uint8_t * trigger_thresholds, const uint8_t * servo_thresholds, uint8_t mask)
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
      dev->trig_thresh[i] = trig;
      dev->servo_thresh[i] = servo;
      ii++;
    }
  }
  return write_words(dev, ii, words);
}

int flower_configure_trigger(flower_dev_t * dev, rno_g_lt_simple_trigger_config_t  cfg)
{
  if (!dev) return -1;
  int ret = 0;
  flower_word_t word = {0};
  word.bytes[0] = FLWR_REG_TRIG_PARAM;
  word.bytes[1] = cfg.vpp_mode;
  word.bytes[2] = cfg.window;
  word.bytes[3] = cfg.num_coinc;
  ret = write_word(dev,&word);
  if (!ret) dev->trig_cfg = cfg;
  return ret;
}

int flower_fill_header(flower_dev_t * dev, rno_g_header_t * hd)
{
  if (!dev) return -1;
  hd->lt_simple_trigger_cfg = dev->trig_cfg;
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


static flower_word_t scal_sel_regs[34];
__attribute__((constructor))
static void fill_scal_sel_regs()
{
  for (int i = 0; i < 34; i++)
  {
    scal_sel_regs[i].bytes[0] = FLWR_REG_SCAL_SEL;
    scal_sel_regs[i].bytes[3] = i;
  }
}

int flower_fill_daqstatus(flower_dev_t *dev, rno_g_daqstatus_t *ds)
{

  if (!dev) return -1;

  #define MAX_DSNMSG (3*(21)+5)

  struct spi_ioc_transfer xfer[MAX_DSNMSG] = {0};

  static flower_word_t update_word = {.bytes = {FLWR_REG_SCAL_UPD,0,0,1}};
  static flower_word_t selectread_word = {.bytes = {FLWR_REG_SET_READ_REG,0,0, FLWR_REG_SCAL_RD}};
  static flower_word_t update_tlow = {.bytes = {FLWR_REG_SET_READ_REG,0,0,FLWR_REG_SCAL_TIME_LOW}};
  static flower_word_t update_thigh = {.bytes = {FLWR_REG_SET_READ_REG,0,0,FLWR_REG_SCAL_TIME_HIGH}};
  flower_word_t dest_scaler[34] = {0};
  uint16_t raw_scalers[64];
  flower_word_t dest_time[2] = {0};

  struct timespec start;
  struct timespec end;


  for (int i = 0; i < 4; i++)
  {
    ds->lt_trigger_thresholds[i] = dev->trig_thresh[i];
    ds->lt_servo_thresholds[i] = dev->servo_thresh[i];
  }

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
  int max_reg = dev->fwver_int < 8 ? 32 : 34;
  for (int ireg = 0; ireg <max_reg; ireg++)
  {
    if (ireg == 18) ireg=31; //scalers 36-61 are empty
    xfer[3*ixfer+5].tx_buf = (uintptr_t) scal_sel_regs[ireg].bytes;
    xfer[3*ixfer+5].len = sizeof(flower_word_t);
    xfer[3*ixfer+5].rx_buf = 0;
    xfer[3*ixfer+6].tx_buf =  (uintptr_t)selectread_word.bytes;
    xfer[3*ixfer+6].rx_buf = 0;
//    xfer[3*ixfer+6].cs_change = 1;
    xfer[3*ixfer+6].len = sizeof(flower_word_t);
    xfer[3*ixfer+7].rx_buf =  (uintptr_t )dest_scaler[ireg].bytes; // will have to finagle these after
    xfer[3*ixfer+7].len = sizeof(flower_word_t);
    xfer[3*ixfer+7].tx_buf =  0;
    ixfer++;
  }

  int nxfer =  (dev->fwver_int < 8) ? 3*19+5 : 3*21+5;

  clock_gettime(CLOCK_REALTIME,&start);
  int ret = ioctl(dev->spi_fd, SPI_IOC_MESSAGE(nxfer), xfer);
  clock_gettime(CLOCK_REALTIME,&end);
//  printf("status ioctl: %d\n", ret);

  ds->when_lt = (start.tv_sec*0.5 + end.tv_sec*0.5) + 1e-9*(start.tv_nsec*0.5 + end.tv_nsec*0.5);

  if (ret > 0)
  {
    for (int i = 0; i < 32; i++)
    {
      uint16_t low =  dest_scaler[i].bytes[3] | ((dest_scaler[i].bytes[2] & 0x0f ) << 8) ;
      uint16_t high = (dest_scaler[i].bytes[1] << 4)  | ((dest_scaler[i].bytes[2] & 0xf0)>>4);
      raw_scalers[2*i] = low;
      raw_scalers[2*i+1] = high;
    }

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


    uint64_t t_low = ( be32toh(dest_time[0].word) & 0xffffff );
    uint64_t t_high = ( be32toh(dest_time[1].word) & 0xffffff );
    ds->lt_scalers.ncycles =  t_low | t_high << 24;
    ds->lt_scalers.scaler_counter_1Hz = raw_scalers[63];

    //printf("scaler 0x20: %x %x %x %x\n", dest_scaler[32].bytes[0], dest_scaler[32].bytes[1], dest_scaler[32].bytes[2], dest_scaler[32].bytes[3]);
    //printf("scaler 0x21: %x %x %x %x\n", dest_scaler[33].bytes[0], dest_scaler[33].bytes[1], dest_scaler[33].bytes[2], dest_scaler[33].bytes[3]);
    if (max_reg > 32)
    {
      uint64_t cyc_low =( be32toh(dest_scaler[32].word) & 0xffffff);
      uint64_t cyc_high =( be32toh(dest_scaler[33].word) & 0xffffff);
      ds->lt_scalers.cycle_counter = cyc_low  | (cyc_high << 24);
    }
    else
    {
      ds->lt_scalers.cycle_counter = 0;

    }



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
                dev->fwver.ver.major, dev->fwver.ver.minor, dev->fwver.ver.rev,
                dev->fwver.word.word,
                dev->fwver.word.bytes[0], dev->fwver.word.bytes[1],
                dev->fwver.word.bytes[2], dev->fwver.word.bytes[3]);
  ret+= fprintf(f,"  FWDATE:  %d-%02d-%02d (0x%x, [0x%x,0x%x,0x%x,0x%x])\n",
                dev->fwdate.date.year, dev->fwdate.date.month, dev->fwdate.date.day,
                dev->fwdate.word.word,
                dev->fwdate.word.bytes[0], dev->fwdate.word.bytes[1],
                dev->fwdate.word.bytes[2], dev->fwdate.word.bytes[3]);
  ret+= fprintf(f,"  TRIGCONFIG:  window: %d, num_coinc: %d, vpp_mode: %d\n",
                dev->trig_cfg.window, dev->trig_cfg.num_coinc, dev->trig_cfg.vpp_mode);

  for (int i = 0; i < 4; i++)
  {
    ret+= fprintf(f,"  THRESH_CH%d:  servo:  %d, trig: %d\n", i, dev->servo_thresh[i], dev->trig_thresh[i]);
  }

  return ret;
}

static flower_word_t sw_trig_low = {.bytes={FLWR_REG_FORCE_TRIG,0,0,0}};
static flower_word_t sw_trig_high = {.bytes={FLWR_REG_FORCE_TRIG,0,0,1}};

int flower_force_trigger(flower_dev_t * dev)
{
  if (!dev) return -1;
  int ret = 0;

  if (dev->fwver_int < 6)
  {
    ret += write_word(dev, &sw_trig_low);
  }

  ret += write_word(dev, &sw_trig_high);

  if (dev->fwver_int < 6)
  {
    dev->must_clear = 1;
  }

  return ret;
}

static flower_word_t buffer_clear = {.bytes={FLWR_REG_BUF_CLEAR,0,0,1}};
int flower_buffer_clear(flower_dev_t * dev)
{
  return write_word(dev, &buffer_clear);
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
 //       xfer[ixfer*5].delay_usecs = 100;
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
//        xfer[ixfer*5+4].delay_usecs = 100;
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
  flower_word_t word = {.bytes = {FLWR_REG_TRIG_ENABLES, enables.enable_ext, enables.enable_coinc, enables.enable_pps }};
  return write_word(dev,&word);
}


int flower_set_trigout_enables(flower_dev_t * dev, flower_trigout_enables_t enables)
{
  flower_word_t word1 = {.bytes={FLWR_REG_TRIGOUT_SYSOUT,0,enables.enable_pps_sysout,enables.enable_rf_sysout}};
  flower_word_t word2 = {.bytes={FLWR_REG_TRIGOUT_AUXOUT,0,enables.enable_pps_auxout,enables.enable_rf_auxout}};
  return write_word(dev, &word1) + write_word(dev, &word2);
}

int flower_equalize(flower_dev_t * dev, float target_rms, uint8_t * v_gain_codes, int opts)
{
  if (!dev) return -1;

  float rms[RNO_G_NUM_LT_CHANNELS] = {0};

  uint8_t mask = (~(opts & 0xf)) & 0xf;
  int verbose = opts & 0x80000000;
  static uint8_t data[RNO_G_NUM_LT_CHANNELS][1024];
  static uint8_t * data_ptrs[RNO_G_NUM_LT_CHANNELS] = { data[0], data[1], data[2], data[3] };
  uint8_t gain_codes[RNO_G_NUM_LT_CHANNELS] = {0};
  uint8_t done = 0;

  while (done != mask)
  {
    flower_set_gains(dev, gain_codes);
    flower_buffer_clear(dev);
    flower_force_trigger(dev);
    int avail = 0;
    while (!avail) flower_buffer_check(dev,&avail);

    flower_read_waveforms(dev, 1024, data_ptrs);
    for (int i = 0; i < RNO_G_NUM_LT_CHANNELS; i++)
    {
      if (done & ( 1 << i) || !(mask & (1 << i))) continue;

      rms[i] = getrms(256, data[i]);

      if (verbose) printf("ch: %d, gain_code: %d, rms: %f\n", i, gain_codes[i], rms[i]);

      if (rms[i] < target_rms && gain_codes[i] < FLOWER_GAIN_TOO_HIGH)
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

  return 0;
}


int flower_get_fwversion(flower_dev_t *dev, uint8_t *major, uint8_t *minor,
                         uint8_t *rev, uint16_t *year, uint8_t *month, uint8_t *day)
{

  if (!dev) return -1;

  if (major) *major = dev->fwver.ver.major;
  if (minor) *minor = dev->fwver.ver.minor;
  if (rev) *rev = dev->fwver.ver.rev;
  if (year) *year = dev->fwdate.date.year;
  if (month) *month = dev->fwdate.date.month;
  if (day) *day = dev->fwdate.date.day;
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

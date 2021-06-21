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


typedef enum
{
  FLWR_REG_FW_VER = 0x01,
  FLWR_REG_FW_DATE = 0x02,
  FLWR_REG_SCAL_RD = 0x03, 
  FLWR_REG_SCAL_UPD = 0x27,
  FLWR_REG_SCAL_SEL = 0x29, 
  FLWR_REG_CALPULSE=0x2a, 
  FLWR_REG_SCAL_TIME_LOW = 0x2c, 
  FLWR_REG_SCAL_TIME_HIGH = 0x2d, 
  FLWR_REG_DATVALID=0x3a,
  FLWR_REG_FORCE_TRIG = 0x40,
  FLWR_REG_CHANNEL = 0x41,
  FLWR_REG_MODE = 0x42,
  FLWR_REG_RAM_ADDR = 0x45,
  FLWR_REG_READ = 0x47,
  FLWR_REG_CHUNK = 0x49,
  FLWR_REG_TRIG_CH0_THR = 0x57,
  FLWR_REG_TRIG_CH1_THR = 0x58,
  FLWR_REG_TRIG_CH2_THR = 0x59,
  FLWR_REG_TRIG_CH3_THR = 0x5a,
  FLWR_REG_TRIG_PARAM  = 0x5b,
  FLWR_REG_SET_READ_REG = 0x6d,
  FLWR_REG_MAX=0x7f
} e_flower_reg; 

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
  return ((int) (N*sizeof(*words))) != write(dev->spi_fd, words, N*sizeof(*words)); 
}
static int write_word(flower_dev_t *dev, const flower_word_t * word) 
{

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
  if (addr <1 || addr > FLWR_REG_MAX) return -1; 
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


int flower_soft_trigger(flower_dev_t * dev, int trig) 
{
  flower_word_t word = {0}; 
  word.bytes[0] = FLWR_REG_FORCE_TRIG; 
  word.bytes[3] = trig & 0xff; 
  return write_word(dev,&word); 
}

int flower_set_thresholds(flower_dev_t *dev, const uint8_t * trigger_thresholds, const uint8_t * servo_thresholds, uint8_t mask) 
{

  flower_word_t words[4] = {0}; 

  int ii = 0; 
  for (int i = 0; i < 4; i++) 
  {
    if (mask & (1 << i)) 
    {
      words[ii].bytes[0] = FLWR_REG_TRIG_CH0_THR+i; 
      words[ii].bytes[2] = servo_thresholds[i]; 
      words[ii].bytes[3] = trigger_thresholds[i]; 
      dev->trig_thresh[i] = trigger_thresholds[i];
      dev->servo_thresh[i] = servo_thresholds[i];
      ii++; 
    }
  }
  return write_words(dev, ii, words); 
}

int flower_configure_trigger(flower_dev_t * dev, rno_g_lt_simple_trigger_config_t  cfg) 
{
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
  hd->lt_simple_trigger_cfg = dev->trig_cfg; 
  return 0; 
}

int flower_close(flower_dev_t * dev)
{
  flock(dev->spi_fd, LOCK_UN); 
  close(dev->spi_fd); 
  free(dev); 
  return 0; 
}



static flower_word_t scal_sel_regs[RNO_G_NUM_LT_SCALERS]; 
__attribute__((constructor)) 
static void fill_scal_sel_regs() 
{
  for (int i = 0; i < RNO_G_NUM_LT_SCALERS; i++) 
  {
    scal_sel_regs[i].bytes[0] = FLWR_REG_SCAL_SEL;
    scal_sel_regs[i].bytes[3] = i; 
  }
}

int flower_fill_daqstatus(flower_dev_t *dev, rno_g_daqstatus_t *ds)
{
  #define DSNMSG (3*(RNO_G_NUM_LT_SCALERS/2)+5)
  struct spi_ioc_transfer xfer[DSNMSG] = {0}; 

  static flower_word_t update_word = {.bytes = {FLWR_REG_SCAL_UPD,0,0,1}}; 
  static flower_word_t selectread_word = {.bytes = {FLWR_REG_SET_READ_REG,0,0, FLWR_REG_READ}};
  static flower_word_t update_tlow = {.bytes = {FLWR_REG_SET_READ_REG,0,0,FLWR_REG_SCAL_TIME_LOW}}; 
  static flower_word_t update_thigh = {.bytes = {FLWR_REG_SET_READ_REG,0,0,FLWR_REG_SCAL_TIME_HIGH}}; 
  flower_word_t dest[RNO_G_NUM_LT_SCALERS/2] = {0}; 
  flower_word_t dest_time[2] = {0}; 

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

  for (int i = 0; i < RNO_G_NUM_LT_SCALERS/2; i++) 
  {
    xfer[3*i+5].tx_buf = (uintptr_t) scal_sel_regs[i].bytes; 
    xfer[3*i+5].len = sizeof(flower_word_t);
    xfer[3*i+6].tx_buf =  (uintptr_t)selectread_word.bytes; 
    xfer[3*i+6].len = sizeof(flower_word_t);
    xfer[3*i+7].rx_buf =  (uintptr_t )dest[i].bytes; // will have to finagle these after
    xfer[3*i+7].len = sizeof(flower_word_t);
  }

  int ret = ioctl(dev->spi_fd, SPI_IOC_MESSAGE(DSNMSG), xfer); 

 //TODO: figure out exact amount
  if (ret > 0) 
  {
    for (int i = 0; i < RNO_G_NUM_LT_SCALERS/2; i++) 
    {
      uint16_t low =  dest[i].bytes[3] | ((dest[i].bytes[2] & 0x0f ) << 8) ;
      uint16_t high = (dest[i].bytes[1] << 4)  | ((dest[i].bytes[2] & 0xf0)>>4); 
      ds->lt_scalers[2*i] = low;
      ds->lt_scalers[2*i+1] = high;
    }
    return 0; 
  }
  return ret ?: -1; 
}

int flower_dump(FILE * f, flower_dev_t *dev) 
{
  int ret = 0; 
  ret+= fprintf(f,"FLOWER HANDLE at 0x%p\n", dev); 
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
  return ret; 
}



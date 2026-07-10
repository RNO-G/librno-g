
#define _GNU_SOURCE
#include "rno-g-cal.h"

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#ifdef USE_LIBGPIOS
#include "libgpios.h"
#include <stdbool.h>
#endif


struct rno_g_cal_dev
{
  int fd;
  int bus;
  char rev;
  int debug;
  uint8_t addr;
  rno_g_calpulser_out_t ch;
  rno_g_calpulser_mode_t mode;
  uint8_t atten;
  int enabled;
  int setup;
#ifdef USE_LIBGPIOS
  gpios_line_t gpio;
#else
  FILE * fgpiodir;
  FILE * fgpioval;
#endif
};

const char valid_revs[] = "DEFN";

const uint8_t addr0 = 0x38;
const uint8_t addr1 = 0x3f;
#ifdef USE_LIBGPIOS
const uint8_t addr2 = 0x19; //temp sensor
#else
const uint8_t addr2 = 0x18; //temp sensor
#endif
const uint8_t addr_hum = 0x40; //humidity sensor for simplicity always defined, only used for revN == USE_LIBGPIOS
const uint8_t TRIGGER_HUMIDITY_MEASURE_NO_HOLD = 0xF5;

const uint8_t output_reg = 0x01;
const uint8_t config_reg = 0x03;
const uint8_t tmp_reg = 0x05;

#ifndef USE_LIBGPIOS
const char gpio_dir_format[] = "/sys/class/gpio/gpio%hu/direction";
const char gpio_val_format[] = "/sys/class/gpio/gpio%hu/value";
#endif


rno_g_cal_dev_t * rno_g_cal_open(uint8_t bus, uint16_t gpio, char rev)
{
  char fname[20];

  if (!rev) //assume revE?
  {
    rev = 'E';
  }
  else
  {
    rev = toupper(rev);
  }

#ifdef USE_LIBGPIOS
  rev = 'N'; // overwrite
#endif

  if (!strchr(valid_revs, rev))
  {
    fprintf(stderr, "rev%c is not valid!\n", rev);
    return NULL;
  }

#ifndef USE_LIBGPIOS
  // make sure we have the gpio exported
  char * gpio_dir = 0;
  asprintf(&gpio_dir, gpio_dir_format, gpio);

// lazy code
  if (access(gpio_dir, W_OK))
  {
    FILE * f = fopen("/sys/class/gpio/export","w");
    if (!f)
    {
      fprintf(stderr,"Do we even have GPIOs? Bailing\n");
      return 0;
    }

    fprintf(f,"%d\n", gpio);
    fclose(f);
    usleep(150000); // long enough?
  }

#else
  (void) gpio;
  gpios_line_t line;
  int ok = gpios_get_line_by_label("CAL_EN", &line, GPIOS_OUTPUT);
  if (ok)
  {
    fprintf(stderr, "Could not open GPIO CAL_EN\n");
    return NULL;
  }

#endif
  snprintf(fname, sizeof(fname)-1, "/dev/i2c-%hhu", bus);

  int fd = open(fname, O_RDWR);

  if (fd < 0)
  {
    fprintf(stderr,"Could not open %s.\n", fname);
    perror("  Reason: ");
    return NULL;
  }

  rno_g_cal_dev_t * dev = calloc(1, sizeof(rno_g_cal_dev_t));

  if (!dev)
  {
    fprintf(stderr,"Could not allocate memory for rno_g_cal_dev_t!!!\n");
    close(fd);
    return NULL;
  }

#ifndef USE_LIBGPIOS
  //open the gpio file
  FILE * fgpio = fopen(gpio_dir,"w");
  if (!fgpio)
  {
    fprintf(stderr, "Could not open %s for writing\n", gpio_dir);
    close(fd);
    free(gpio_dir);
    free(dev);
    return NULL;
  }

  free(gpio_dir);

  char * gpio_val = 0;
  asprintf(&gpio_val, gpio_val_format, gpio);
  FILE * fval = fopen(gpio_val,"w");
  if (!fval)
  {
    fprintf(stderr, "Could not open %s for writing\n", gpio_val);
    close(fd);
    free(gpio_val);
    fclose(fgpio);
    free(dev);
    return NULL;
  }

  free(gpio_val);



  setbuf(fgpio,0);
  setbuf(fval,0);
  dev->fgpiodir = fgpio;
  dev->fgpioval = fval;
#else
  memcpy(&dev->gpio, &line, sizeof(line));
#endif

  dev->fd = fd;
  dev->rev = rev;
  dev->bus = bus;

  dev->setup = 0 ;
  dev->enabled = -1;

  return dev;

}

int rno_g_cal_close(rno_g_cal_dev_t * dev)
{
  if (!dev) return -1;
#ifdef USE_LIBGPIOS
  gpios_release(&dev->gpio);
#else
  fclose(dev->fgpiodir);
  fclose(dev->fgpioval);
#endif
  close(dev->fd);
  free(dev);
  return 0;
}

int rno_g_cal_enable(rno_g_cal_dev_t * dev)
{
  if (!dev) return 1;
  dev->enabled = 1;
#ifdef USE_LIBGPIOS
  return gpios_set_value(&dev->gpio,true);
#else
  return (fprintf(dev->fgpiodir,"out\n") != sizeof("out\n")-1) || (fprintf(dev->fgpioval,"1\n") != sizeof("1\n")-1);
#endif
}

int rno_g_cal_disable(rno_g_cal_dev_t * dev)
{
  if (!dev) return 1;
  dev->enabled = 0;

#ifdef USE_LIBGPIOS
  return gpios_set_value(&dev->gpio,false);
#else
  return (fprintf(dev->fgpioval,"0\n") != sizeof("0\n")-1) || (fprintf(dev->fgpiodir,"in\n") != sizeof("in\n")-1);
#endif
}


int rno_g_cal_disable_no_handle(uint16_t gpio)
{
#ifdef USE_LIBGPIOS
  gpios_line_t line;
  //open as an input, that will turn it off :)
  int ok = gpios_get_line_by_label("CAL_EN", &line, 0);
  gpios_release(&line);
  return ok;
#else
  char gpio_dir[128];
  snprintf(gpio_dir,sizeof(gpio_dir), gpio_dir_format, gpio);

  if (access(gpio_dir, W_OK))
  {
    //not exported (at least in a way we canwrite to);
    // so it's probably off
    return 0;

  }

  FILE * f = fopen(gpio_dir,"w");
  if (!f) return 0;

  int ret = fprintf(f,"in\n") != sizeof("in\n"-1);
  fclose(f);
  return ret;
#endif
}


static int do_write(rno_g_cal_dev_t * dev, uint8_t addr, uint8_t reg, uint8_t val)
{

  if (!dev) return 1;
  uint8_t buf[2] = {reg, val};
  struct i2c_msg msg =
  {
    .addr = addr,
    .flags = 0,
    .len = sizeof(buf),
    .buf = buf
  };

  if (dev->debug)
  {
    printf("DBG: do_write(.addr=0x%02x, .flags=0x%02x, .len=%02x, .buf={0x%02x,0x%02x}\n", msg.addr, msg.flags, msg.len, msg.buf[0], msg.buf[1]);
  }

  struct i2c_rdwr_ioctl_data i2c_data = {.msgs = &msg, .nmsgs = 1 };

  if (ioctl(dev->fd, I2C_RDWR, &i2c_data) < 0 )
  {
    return -errno;
  }

  return 0;
}


static int do_readv(rno_g_cal_dev_t * dev, uint8_t addr, uint8_t reg, int N, uint8_t* data)
{

  if (!dev) return 1;
  struct i2c_msg txn[2] = {
    {.addr = addr, .flags = 0, .len = sizeof(reg), .buf = &reg},
    {.addr = addr, .flags = I2C_M_RD, .len = N, .buf = data} };

  if (dev->debug)
  {
    printf("DBG: do_readv(.addr=0x%02x, .reg=0x%02x, .len=%d)\n", addr, reg, N);

  }
  struct i2c_rdwr_ioctl_data i2c_data = {.msgs = txn, .nmsgs = 2 };

  if (ioctl(dev->fd, I2C_RDWR, &i2c_data) < 0 )
  {
    return -errno;
  }

  if (dev->debug)
  {
    printf("  read: ");
    for (int i = 0; i < N; i++)
    {
      printf ("0x%02x ", data[i]);
    }
    printf("\n");
  }

  return 0;
}


static int do_read(rno_g_cal_dev_t *dev, uint8_t addr, uint8_t reg, uint8_t * val)
{
  return do_readv(dev, addr, reg, 1, val);
}

// write a single raw command byte (no separate register byte), e.g. for the humidity sensor
static int do_write_cmd(rno_g_cal_dev_t * dev, uint8_t addr, uint8_t cmd)
{
  if (!dev) return 1;
  struct i2c_msg msg = { .addr = addr, .flags = 0, .len = sizeof(cmd), .buf = &cmd };

  if (dev->debug)
  {
    printf("DBG: do_write_cmd(.addr=0x%02x, .cmd=0x%02x)\n", addr, cmd);
  }

  struct i2c_rdwr_ioctl_data i2c_data = {.msgs = &msg, .nmsgs = 1 };

  if (ioctl(dev->fd, I2C_RDWR, &i2c_data) < 0 )
  {
    return -errno;
  }

  return 0;
}

// read N raw bytes with no register byte sent first (e.g. for the humidity sensor)
static int do_readv_raw(rno_g_cal_dev_t * dev, uint8_t addr, int N, uint8_t * data)
{
  if (!dev) return 1;
  struct i2c_msg msg = { .addr = addr, .flags = I2C_M_RD, .len = N, .buf = data };

  if (dev->debug)
  {
    printf("DBG: do_readv_raw(.addr=0x%02x, .len=%d)\n", addr, N);
  }

  struct i2c_rdwr_ioctl_data i2c_data = {.msgs = &msg, .nmsgs = 1 };

  if (ioctl(dev->fd, I2C_RDWR, &i2c_data) < 0 )
  {
    return -errno;
  }

  if (dev->debug)
  {
    printf("  read: ");
    for (int i = 0; i < N; i++)
    {
      printf ("0x%02x ", data[i]);
    }
    printf("\n");
  }

  return 0;
}


int rno_g_cal_setup(rno_g_cal_dev_t* dev)
{
  if (!dev) return 1;
  if (do_write(dev, addr0, output_reg,0x00)) return -errno;
  if (do_write(dev, addr0, config_reg,0x00)) return -errno;
  if (do_write(dev, addr1, output_reg,0x00)) return -errno;
  if (do_write(dev, addr1, config_reg,0x00)) return -errno;
#ifdef USE_LIBGPIOS
  dev->ch = (rno_g_calpulser_out_t) -1; //nothing selected yet (no RF output maps to the all-zero register state)
#else
  dev->ch = RNO_G_CAL_NO_OUTPUT;
#endif
  dev->mode = RNO_G_CAL_NO_SIGNAL;
  dev->atten = 63; //max, I think
  dev->setup = 1;
  return 0;
}


int rno_g_cal_set_pulse_mode(rno_g_cal_dev_t *dev, rno_g_calpulser_mode_t type)
{
  uint8_t val;
  if (!dev) return 1;

  if (dev->setup && dev->mode == type) return 0;  // Also return if type == RNO_G_CAL_NO_SIGNAL

  if (do_read(dev, addr1, output_reg, &val))
  {
    return -errno;
  }

  if (type == RNO_G_CAL_PULSER)
  {
    val|=0x40; //turn on pulser

    if (dev->rev == 'D')
    {
      val&=(~0x08); //turn off VCO?
    }
    else if (dev->rev=='N') //only one VCO for revN
    {
      val&=(~0x02); //turn off VCO?
    }
    else
    {
      val&=(~0x03); // turn off VCO for revE
    }
  }
  // Has to be either RNO_G_CAL_VCO or RNO_G_CAL_VCO2
  else if (dev->rev=='D') //only one VCO for revD
  {
    val&=(~0x40); //turn off pulser?
    val|=0x08; //turn on VCO?
  }
  else if (dev->rev=='N') //only one VCO for revN
  {
    val&=(~0x40); //turn off pulser?
    val|=0x02; //turn on VCO?
  }
  else
  {
    uint8_t orwith = type == RNO_G_CAL_VCO ? 0x01 : 0x02;
    val&=(~0x40); //turn off pulser?
    val|=orwith; //turn on correct VCO
  }

  if (do_write(dev,addr1,output_reg,val))
  {
      return -errno;
  }
  if (dev->setup) dev->mode = type;
  return 0;
}


int rno_g_cal_select(rno_g_cal_dev_t * dev, rno_g_calpulser_out_t ch)
{
  if (!dev) return 1;
  uint8_t val0;
  uint8_t val1;

  if (dev->setup && dev->ch == ch) return 0;
  if (do_read(dev, addr0, output_reg, &val0))
    return -errno;

  if (do_read(dev, addr1, output_reg, &val1))
    return -errno;

  if (dev->rev == 'E' || dev->rev == 'F')
  { //biases are active low
    val1 |= 0xc;
    if (do_write(dev, addr1, output_reg, val1))
      return -errno;
  }
  else if (dev->rev == 'N') { //biases are active high
    val1 &= (~0x1d);
    if (do_write(dev, addr1, output_reg, val1))
      return -errno;
  }

#ifdef USE_LIBGPIOS
  uint8_t bias_bit;
  switch (ch)
  {
    case RNO_G_CAL_RF0:
      val0 |= 0x02;  //set out switch 0 to 1
      val1 |= 0x20;  //set out switch 1 to 1
      bias_bit = 0x01; //en_bias0
      break;
    case RNO_G_CAL_RF1:
      val0 |= 0x02;  //set out switch 0 to 1
      val1 &= ~0x20; //set out switch 1 to 0
      bias_bit = 0x04; //en_bias1
      break;
    case RNO_G_CAL_RF2:
      val0 &= ~0x02; //set out switch 0 to 0
      val1 |= 0x80;  //set out switch 2 to 1
      bias_bit = 0x08; //en_bias2
      break;
    case RNO_G_CAL_RF3:
      val0 &= ~0x02; //set out switch 0 to 0
      val1 &= ~0x80; //set out switch 2 to 0
      bias_bit = 0x10; //en_bias3
      break;
    default:
      fprintf(stderr, "output %d is not a valid output, please select RF0-RF3\n", (int) ch);
      return 1;
  }
#else
  if (ch == RNO_G_CAL_COAX)
  {
    val0 &= (~0x02);
    val1 &= (~0x80);
  }
  else if ( ch == RNO_G_CAL_FIB0)
  {
    if (dev->rev=='D')
    {
      val0 &= ~0x02;
      val1 |= 0x80;
    }
    else
    {
      val0 |= 0x02;
      val1 &= 0x2f;
      val1 &= ~0x04;
    }
  }
  else if ( ch == RNO_G_CAL_FIB1 )
  {
    if (dev->rev=='D')
    {
      val0 |= 0x02;
      val1 &= 0x20;
    }
    else
    {
      val0 &= ~0x02;
      val1 |= 0x80;
      val1 &= ~0x08;
    }
  }
  else
  {
    //set all switches to 0?
    val0 &= (~0x02);
    val1 &= (~0x80);
    val1 &= (~0x20);
  }
#endif


  if (do_write(dev, addr0, output_reg, val0)) return -errno;
  if (do_write(dev, addr1, output_reg, val1)) return -errno;

#ifdef USE_LIBGPIOS
  //enable the bias for the selected output (matches Python reference: re-read + OR in the bias bit)
  val1 |= bias_bit;
  if (do_write(dev, addr1, output_reg, val1)) return -errno;
#endif

  if (dev->setup) dev->ch = ch;
  return 0;
}

int rno_g_cal_set_atten(rno_g_cal_dev_t *dev, uint8_t atten)
{
  if (!dev) return 1;
  if (dev->setup && dev->atten == atten) return 0;
  //read addr0
  uint8_t val;
  if (do_read(dev, addr0, output_reg, &val))
  {
    return -errno;
  }
  val &=0x02;

  uint8_t set_atten = atten;
  if (atten > 63) atten = 63;
  atten = 63-atten;

  if (dev->debug) printf("atten (inv): 0x%02x\n", atten);

  //bit reverse
  uint8_t reversed = (atten & 1) << 5 ;
  reversed |=  ((atten & 2) >> 1 ) << 4;
  reversed |=  ((atten & 4) >> 2)  << 3;
  reversed |=  ((atten & 8) >> 3)  << 2;
  reversed |=  ((atten & 16) >> 4)  << 1;
  reversed |=  (atten & 32) >> 5;
  if (dev->debug) printf("reversed: 0x%02x\n", reversed);

  val |= (reversed << 2);

  if (do_write(dev, addr0, output_reg, val)) return -errno;
  if (do_write(dev, addr0, output_reg, val | 0x1)) return -errno; //load enable
  if (do_write(dev, addr0, output_reg, val)) return -errno;

  if (dev->setup) dev->atten = set_atten;
  return 0;
}

int rno_g_cal_read_temp(rno_g_cal_dev_t *dev, float *Tout)
{
  if (!dev) return 1;
  if (dev->rev == 'F')
  {
    *Tout = -128;
    return 0;
  }
  uint8_t data[2];

  if (do_readv(dev, addr2, tmp_reg, sizeof(data), data)) return -errno;

  float T = data[1]/16.;
  T+= (data[0] & 0xf) * 16;

  if (data[0] & 0x10)
    T-=256;

  if (Tout)
    *Tout = T;

  return 0;

}

int rno_g_cal_read_humidity(rno_g_cal_dev_t *dev, float *humidity)
{
  if (!dev) return 1;
  if (dev->rev < 'N')
  {
    if (humidity) *humidity = -1;
    return 0;
  }

  // Send humidity measurement command (no-hold master mode)
  if (do_write_cmd(dev, addr_hum, TRIGGER_HUMIDITY_MEASURE_NO_HOLD)) return -errno;

  // Wait for conversion (~20ms typical)
  usleep(50000);

  uint8_t data[3];
  if (do_readv_raw(dev, addr_hum, sizeof(data), data)) return -errno;

  uint16_t raw = (data[0] << 8) | data[1];

  // Clear status bits
  raw &= 0xFFFC;

  // Convert to %RH using datasheet formula
  if (humidity)
    *humidity = -6 + (125.0 * raw / 65536.0);

  return 0;

}


void rno_g_cal_enable_dbg(rno_g_cal_dev_t * dev, int dbg)
{
  if (dev)
    dev->debug = dbg;

}

int rno_g_cal_fill_info(rno_g_cal_dev_t * dev, rno_g_calpulser_info_t *info)
{

  if (!dev) return 1;
  if (dev->enabled!=1 || !dev->setup)
  {
   //just fill with 0's
    memset(info,0,sizeof(*info));
    //except for rev
    info->rev = dev->rev;
    if (!dev->setup || dev->enabled == -1)
    {
      fprintf(stderr,"WARNING, asking for calpulser info but not properly setup yet!\n");
      return 1;
    }
    return 0;
  }

  info->atten_times_2 = dev->atten;
  info->mode = dev->mode;
  info->out = dev->ch;
  info->rev = dev->rev;
  info->enabled = dev->enabled;
  float T = 0;
  rno_g_cal_read_temp(dev,&T);
  info->T_times_16 = T*16;
  return 0;
}

int rno_g_cal_wait_ready(rno_g_cal_dev_t * dev)
{
  if (!dev) return 1;


  int nwait = 0;
  while (do_write(dev, addr0, output_reg,0x00))
  {
    usleep(2500);
    if (nwait++ > 40)
    {  //100 ms
       return -1;
    }
  }

  return 0;
}

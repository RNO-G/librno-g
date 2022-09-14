
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
  FILE * fgpiodir;
  FILE * fgpioval;
}; 

const char valid_revs[] = "DE"; 

const uint8_t addr0 = 0x38; 
const uint8_t addr1 = 0x3f; 
const uint8_t addr2 = 0x18; //temp sensor
const uint8_t output_reg = 0x01; 
const uint8_t config_reg = 0x03; 
const uint8_t tmp_reg = 0x05; 

const char gpio_dir_format[] = "/sys/class/gpio/gpio%hu/direction"; 
const char gpio_val_format[] = "/sys/class/gpio/gpio%hu/value"; 


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

  if (!strchr(valid_revs,rev))
  {
    fprintf(stderr,"rev%c is not valid!\n", rev); 
    return NULL; 
  }

  // make sure we have the gpio exported
  char * gpio_dir = 0; 
  asprintf(&gpio_dir,gpio_dir_format, gpio); 
 
// lazy code 
  if (access(gpio_dir, W_OK))
  {
    FILE * f = fopen("/sys/class/gpio/export","w"); 
    fprintf(f,"%d\n", gpio); 
    fclose(f); 
    usleep(50000); // long enough? 
  }
  snprintf(fname,sizeof(fname)-1, "/dev/i2c-%hhu", bus); 

  int fd = open(fname, O_RDWR); 

  if (fd < 0) 
  {
    fprintf(stderr,"Could not open %s.\n", fname); 
    perror("  Reason: ");
    return NULL; 
  }

  rno_g_cal_dev_t * dev = calloc(sizeof(rno_g_cal_dev_t),1); 



  //open the gpio file
  FILE * fgpio = fopen(gpio_dir,"w"); 
  if (!fgpio) 
  {
    fprintf(stderr, "Could not open %s for writing\n", gpio_dir); 
    close(fd); 
    free(gpio_dir); 
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
    return NULL; 
  }

  free(gpio_val); 

  if (!dev) 
  {
    fprintf(stderr,"Could not allocate memory for rno_g_cal_dev_t!!!\n"); 
    fclose(fgpio); 
    close(fd); 
    return NULL; 
  }



  setbuf(fgpio,0);
  setbuf(fval,0);
  dev->fgpiodir = fgpio; 
  dev->fgpioval = fval; 
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
  fclose(dev->fgpiodir); 
  fclose(dev->fgpioval); 
  close(dev->fd); 
  free(dev); 
  return 0; 
}

int rno_g_cal_enable(rno_g_cal_dev_t * dev) 
{
  dev->enabled = 1; 
  return (fprintf(dev->fgpiodir,"out\n") != sizeof("out\n")-1) || (fprintf(dev->fgpioval,"1\n") != sizeof("1\n")-1);
}

int rno_g_cal_disable(rno_g_cal_dev_t * dev) 
{
  dev->enabled = 0; 
  return (fprintf(dev->fgpioval,"0\n") != sizeof("0\n")-1) || (fprintf(dev->fgpiodir,"in\n") != sizeof("in\n")-1);
}

int rno_g_cal_disable_no_handle(uint16_t gpio) 
{
  char gpio_dir[128]; 
  snprintf(gpio_dir,sizeof(gpio_dir), gpio_dir_format, gpio); 

  if (access(gpio_dir, W_OK))
  {
    //not exported (at least in a way we canwrite to); 
    // so it's probably off
    return 0; 

  }

  FILE * f = fopen(gpio_dir,"w"); 
  int ret = fprintf(f,"in\n") != sizeof("in\n"-1);
  fclose(f); 
  return ret; 
}





static int do_write(rno_g_cal_dev_t * dev, uint8_t addr, uint8_t reg, uint8_t val) 
{

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
  return do_readv(dev,addr,reg,1, val); 
}


int rno_g_cal_setup(rno_g_cal_dev_t* dev)
{
  if (do_write(dev, addr0, output_reg,0x00)) return -errno; 
  if (do_write(dev, addr0, config_reg,0x00)) return -errno; 
  if (do_write(dev, addr1, output_reg,0x00)) return -errno; 
  if (do_write(dev, addr1, config_reg,0x00)) return -errno; 
  dev->ch = RNO_G_CAL_NO_OUTPUT; 
  dev->mode = RNO_G_CAL_NO_SIGNAL; 
  dev->atten = 63; //max, I think
  dev->setup = 1; 
  return 0; 
}


int rno_g_cal_set_pulse_mode(rno_g_cal_dev_t *dev, rno_g_calpulser_mode_t type) 
{
  uint8_t val; 

  if (dev->setup && dev->mode == type) return 0; 

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
    else
    {
      val&=(~0x03); // turn off VCO for revE 
    }
  }
  else if (dev->rev=='D') //only one VCO for revD 
  {
    val&=(~0x40); //turn off pulser?
    val|=0x08; //turn on VCO? 
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
  uint8_t val0; 
  uint8_t val1; 

  if (dev->setup && dev->ch ==  ch) return 0; 
  if (do_read(dev, addr0, output_reg, &val0)) 
    return -errno; 

  if (do_read(dev, addr1, output_reg, &val1)) 
    return -errno; 

  if (dev->rev == 'E') 
  {
    val1 |= 0xc; 
    if (do_write(dev, addr1, output_reg, val1))
      return -errno; 
  }

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
      val1 |= 0x20; 
    }
    else
    {
      val0 |= 0x02; 
      val1 &= 0x2f; 
      val1 &= ~0x04; 
    }
  }
  else
  {
    //set all switches to 0? 
    val0 &= (~0x02); 
    val1 &= (~0x80); 
    val1 &= (~0x20); 
  }


  if (do_write(dev, addr0, output_reg, val0)) return -errno; 
  if (do_write(dev, addr1, output_reg, val1)) return -errno; 
  if (dev->setup) dev->ch = ch; 
  return 0; 
}

int rno_g_cal_set_atten(rno_g_cal_dev_t *dev, uint8_t atten) 
{
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

void rno_g_cal_enable_dbg(rno_g_cal_dev_t * dev, int dbg) 
{
  dev->debug = dbg; 

}

int rno_g_cal_fill_info(rno_g_cal_dev_t * dev, rno_g_calpulser_info_t *info) 
{
  
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

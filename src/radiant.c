#include "radiant.h" 

#include <linux/spi/spidev.h> 
#include <sys/types.h> 
#include <sys/ioctl.h> 
#include <sys/file.h> 
#include <stdlib.h> 
#include <string.h> 


#if defined(__arm__) && !defined(NOVECTORIZE) 
#include <arm_neon.h> 
#endif

//reg map 

// two-byte commands, split into  MSB, LSB so I don't have to think about it as much
static uint8_t SPI_REG_READ[2] = { 0x52,0x44 }; 
static uint8_t SPI_REG_READ_PEEK[2] = { 0x50,0x4b }; 
static uint8_t SPI_REG_REWIND[2] = { 0x53,0x54 }; 
static uint8_t SPI_REG_CLEAR[2] = { 0x43,0x4c }; 
static uint8_t SPI_REG_RESET[2] = { 0x45,0x59 }; 

struct radiant_dev
{
  int spi_fd; 
  int uart_fd; 
  int run; 
  int peek; // use peek form of read
}; 


void radiant_set_run_number(radiant_dev_t * bd, int run) 
{
  bd->run = run; 
}

void radiant_set_read_mode(radiant_dev_t *bd, int peek) 
{
  bd->peek = peek; 
}

int radiant_clear(radiant_dev_t * bd) 
{
  return 2 != write(bd->spi_fd, SPI_REG_CLEAR, 2); 
}

int radiant_reset(radiant_dev_t * bd) 
{
  return 2 != write(bd->spi_fd, SPI_REG_RESET, 2); 
}

int radiant_rewind(radiant_dev_t * bd) 
{
  return 2 != write(bd->spi_fd, SPI_REG_REWIND, 2); 
}

int radiant_read(radiant_dev_t * bd, uint16_t * navail, int nbufs, int * N, uint16_t **bufs)
{
  // to separate MSB and LSB more easily 
  uint8_t avail[2]; 

  uint16_t nxfers = (3+nbufs) & 0x1ff;  // only up to 511 transfers are supported . This also prevents a potential stack overflow.  

  struct spi_ioc_transfer xfers[nxfers] ; 
  memset(xfers,0,sizeof(struct spi_ioc_transfer) * nxfers);

  xfers[0].tx_buf = (uintptr_t)  (bd->peek ? &SPI_REG_READ_PEEK[0] : &SPI_REG_READ[0]);
  xfers[0].rx_buf = 0; 
  xfers[0].len = 1 ;

  xfers[1].tx_buf =  (uintptr_t) (bd->peek ? &SPI_REG_READ_PEEK[1] : &SPI_REG_READ[1]);
  xfers[1].rx_buf =  (uintptr_t) (navail ? &avail[0] : 0) ;
  xfers[1].len = 1 ;

  xfers[2].tx_buf = 0; 
  xfers[2].rx_buf =  (uintptr_t) (navail ? &avail[1] : 0); 
  xfers[2].len = 1; 

  for (int i = 3; i < nxfers; i++)
  {
    xfers[i].tx_buf = 0; 
    xfers[i].rx_buf = (uintptr_t) bufs[i]; 
    xfers[i].len =  2*N[i]; 
  }

  int ret =  ioctl(bd->spi_fd, SPI_IOC_MESSAGE(nxfers), xfers); 

  if (navail) 
  {
    *navail = avail[1]; 
    *navail |= (avail[0] << 8); 
  }
  return ret; 
}



int radiant_check_avail(radiant_dev_t * bd) 
{

  uint16_t navail; 
  int ret = radiant_read(bd,&navail,0,0,0); 
  if (ret<0) return ret; 
  return navail;
}

struct fw_event_header
{
  uint16_t magic; 
  uint16_t programmable; 
  uint16_t counter[2]; 
  uint16_t time[2]; 
  uint16_t tinfo[2]; 
  uint16_t nev[2]; 
  uint16_t nstatus[2]; 
}; 

static int get_gcd(int a, int b) 
{
  if (!b) return a; 
  else return get_gcd(b,a%b); 
}

//TODO figure out how to vectorize this (and maybe also remove high bits while we go?) 
// for now use algorithm from https://www.geeksforgeeks.org/array-rotation/
static void roll16(uint16_t * v, int shift, int N)  //using 
{
  int i,j,k,tmp; 
  int gcd = get_gcd(shift, N); 

  for (i = 0; i < gcd; i++) 
  {
    tmp = v[i]; 
    j = i; 
    while(1) 
    {
      k = j + shift; 
      if (k >= N) k-=N; 
      if (i==k) break;
      v[j]=v[k]; 
      j=k; 
    }
    v[j] = tmp;
  }
}


static void andall16(uint16_t *v, uint16_t andme, int N) 
{

  int i = 0;
#if defined(__arm__) && !defined(NOVECTORIZE) 
//we'll use ARM NEON intrinsics. Since usually this will be a multiple of 32, we'll use vandq_u16 unrolled by 4 

  int Niter = N/32; 

  if (Niter) 
  {
    //probably a smarter way to do this 
    uint16_t andme_v[16] = { andme,andme,andme,andme,andme,andme,andme,andme};
    uint16x8_t vandme = v1d1q_u16 (andme_v); 

    for (i = 0; i < Niter; i++) 
    {
      uint16x8_t v1 = vld1q_u16(v+i*32);
      uint16x8_t v2 = vld1q_u16(v+i*32+8);
      uint16x8_t v3 = vld1q_u16(v+i*32+16);
      uint16x8_t v4 = vld1q_u16(v+i*32+24);
      v1 = vandq_u16(v1,vandme);
      v2 = vandq_u16(v2,vandme);
      v3 = vandq_u16(v3,vandme);
      v4 = vandq_u16(v4,vandme);
      vst1q_u16(v1, v+i*32);
      vst1q_u16(v2,v+i*32+8);
      vst1q_u16(v3,v+i*32+16);
      vst1q_u16(v4,v+i*32+24);
    }

    i*=32; // catch any remaining elements with a normal loop
  }
#endif 


  // finish the last (or do all of them, if not using intrinsics 
  for (; i < N; i++) 
  {
    v[i] &= andme; 
  }
}


int radiant_read_event(radiant_dev_t * bd, rno_g_header_t * hd, rno_g_waveform_t *wf) 
{

  struct fw_event_header fwhd; 
  uint16_t * hd_buf = (uint16_t*) &fwhd; 
  uint16_t navail; 
  int N =sizeof(fwhd)/sizeof(uint16_t);  


  //Read the event metadata. In principle we should know the
  //size of everything ahead of time so we can just read everything
  //in the same syscall, but we can break it up a bit for now I guess. 
  // This also will check if anything is actualy available (slightly less efficiently, but that's ok) 
  int ret = radiant_read(bd, &navail, 1, &N,&hd_buf); 
  
  if (ret) return ret; 
  if (!navail) return 1;  //nothing to read


  //check magic 
  if (HE16(fwhd.magic)!= 0x5244) 
  {
    fprintf(stderr,"Bad magic :%g\n", fwhd.magic);
    return -0x5244; 
  }


  // Start filling in the metadata 
  memset(hd,0,sizeof(*hd)); 
  hd->event_number |= (HE16(fwhd.counter[1]));
  hd->event_number |= (HE16(fwhd.counter[0])) << 16;
  hd->trigger_time |= (HE16(fwhd.time[1]));
  hd->trigger_time |= (HE16(fwhd.time[0])) << 16;
  hd->run_number = bd->run; 

  int num_status = HE16(fwhd.nstatus[1]); 
  num_status |= (HE16(fwhd.nstatus[0])) << 16; 

  int num_ev = HE16(fwhd.nev[1]); 
  num_ev |= (HE16(fwhd.nev[0])) << 16; 

  wf->event_number = hd->event_number; 
  wf->run_number = bd->run; 

  int nsamples = (num_ev-12)/24; 
  wf->nsamples = nsamples; 

  //we will have 25 transactions, the status (which we ignore for now) , and then one for each channel;
  int Ns[1+RNO_G_NUM_RADIANT_CHANNELS]; 
  uint16_t* bufs[1+RNO_G_NUM_RADIANT_CHANNELS]; 

  Ns[0] = num_status; 
  bufs[0] = 0; //ignore status words for now

  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
  {
    Ns[i+1] = nsamples; 
    bufs[i+1] = wf->radiant_waveforms[i]; 
  }

  // the monster read call. We can break this up per channel if it becomes problematic.  
  ret = radiant_read(bd, 0, RNO_G_NUM_RADIANT_CHANNELS+1, Ns,bufs); 
        
  
  //now let's figure out the buffer number, start bytes, rotate, and remove the high bytes. 
  
  //get buffer number (assume little-endian here and assume the same for all channels?) 
  int high_window = (wf->radiant_waveforms[0][0] & 0xc000) >> 14; 

  for (int ichan = 0; ichan < RNO_G_NUM_RADIANT_CHANNELS; ichan++) 
  {


    //now let's find the start window 
    //right now I'm assuming it's the same for each channel
    int nrotate = 0;

    for (int w = 0; w < RNO_G_NUM_WINDOWS; w++) 
    {
      uint16_t val = wf->radiant_waveforms[ichan][RNO_G_WINDOW_SIZE*w];
      if (val & 0x2000)//this is the start window
      {
        hd->radiant_start_window[ichan] = high_window ? w+RNO_G_NUM_WINDOWS: w; 
        nrotate = w * RNO_G_WINDOW_SIZE; 
        break; 
      }
      else if (val & 0x1000) //this is the stop window 
      {
        //figure out the number of windows we must have read... this might just be constant! 
        int num_windows =  nsamples >> 7; 
        int start = (w-num_windows) % RNO_G_NUM_WINDOWS ;
        hd->radiant_start_window[ichan] = start + (high_window ? RNO_G_NUM_WINDOWS : 0);  
        nrotate = start * RNO_G_WINDOW_SIZE; 
        break; 
      }
    }


    //maybe it's possible to rotate and removce high bits at same time? 
    //that sounds a bit more complicated
    if (nrotate) roll16(wf->radiant_waveforms[ichan], nrotate, nsamples); 

    //remove the upper bits
    if (high_window) //have to remove upper bits on all samples
    {
      andall16(wf->radiant_waveforms[ichan], 0x0fff, nsamples); 
    }
    else //only have to remove upper bits on first and last windows, which are now in the right place
    {
      andall16(wf->radiant_waveforms[ichan], 0x0fff, RNO_G_WINDOW_SIZE); 
      andall16(wf->radiant_waveforms[ichan] + (nsamples-RNO_G_WINDOW_SIZE), 0x0fff, RNO_G_WINDOW_SIZE); 
    }
  }

}


radiant_dev_t * radiant_open(const char *spi_device, const char * uart_device) 
{

  radiant_dev_t * dev = 0; 
  int locked, spi_fd = 0, uart_fd =0; 

  //open the device, if we can 
  spi_fd = open(spi_device, O_RDWR); 
  if (spi_fd < 0) 
  {
    fprintf(stderr,"Could not open %s\n", spi_device); 
    return 0; 
  }

  //advisory locks, so we don't accidentally open it twice (how does this work with symlinks?) 
  locked = flock(spi_fd, LOCK_EX | LOCK_NB);
  if (locked < 0) 
  {
    fprintf(stderr,"Could not get exclusive access to %s\n", spi_device); 
    close(spi_fd); 
    return 0; 
  }

  if (uart_device) 
  {
    //TODO

  }


  dev = calloc(sizeof(radiant_dev_t),1); 
  dev->spi_fd = spi_fd; 
  dev->uart_fd = 0; 

  return dev; 
}


void radiant_close(radiant_dev_t * dev) 
{
  if (!dev) return; 

  if (dev->uart_fd) 
  {
    flock(dev->uart_fd, LOCK_UN); 
    close(dev->uart_fd); 
  }
  flock(dev->spi_fd,LOCK_UN); 
  close(dev->spi_fd); 

  free(dev); 
}







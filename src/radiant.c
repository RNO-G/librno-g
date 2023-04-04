#include "radiant.h" 
#include "cobs.h" 
#include <linux/spi/spidev.h> 
#include <sys/types.h> 
#include <stdio.h>
#include <sys/ioctl.h> 
#include <unistd.h> 
#include <sys/file.h> 
#include <stdlib.h> 
#include <string.h> 
#include <fcntl.h> 
#include <termios.h> 
#include <errno.h> 
#include <pthread.h>
#include <poll.h>
#include <signal.h>
#include <assert.h> 
#include <errno.h> 
#include <math.h> 



#if defined(__arm__) && !defined(NOVECTORIZE) 
#include <arm_neon.h> 
#endif

#include "adf4350.h" 

#define EVENT_MAGIC 0x52444530


// BOARD MANAGER REGISTERS
//  maybe move this to API? 
typedef enum
{
  BM_REG_IDENT                      = 0x0,
  BM_REG_DATEVERSION                = 0x4, 
  BM_REG_STATUS                     = 0x8,
  BM_REG_CONTROL                    = 0xc, 
  BM_REG_ANAV10                     = 0x10, 
  BM_REG_ANAV18                     = 0x14, 
  BM_REG_ANAV25                     = 0x18, 
  BM_REG_ANALEFT                    = 0x1c, 
  BM_REG_ANARIGHT                   = 0x20, 
  BM_REG_SPIOUT_LSB                 = 0x24, 
  BM_REG_SPIOUT_MSB                 = 0x28, 
  BM_REG_GPIO_BASE                  = 0x40, 
  BM_REG_GPIO_INCR                  = 0x04,  //NOT AN ADDRESS
  BM_REG_GPIO0                      = 0x40, 
  BM_REG_GPIO1                      = 0x44, 
  BM_REG_GPIO2                      = 0x48, 
  BM_REG_GPIO3                      = 0x4c, 
  BM_REG_GPIO4                      = 0x50, 
  BM_REG_GPIO5                      = 0x54, 
  BM_REG_SIGGPIO                    = 0x58, 
  BM_REG_SIGPIO_IDX                 = 0x06,  //NOT AN ADRRESS
  BM_REG_TDBIAS_BASE                = 0x80, 
  BM_REG_TDBIAS_INCR                = 0x04,  //NOT AN ADDRESS
  BM_REG_TDBIAS0                    = 0x80, 
  BM_REG_TDBIAS1                    = 0x84, 
  BM_REG_TDBIAS2                    = 0x88, 
  BM_REG_TDBIAS3                    = 0x8c, 
  BM_REG_TDBIAS4                    = 0x90, 
  BM_REG_TDBIAS5                    = 0x94, 
  BM_REG_TDBIAS6                    = 0x98, 
  BM_REG_TDBIAS7                    = 0x9c, 
  BM_REG_TDBIAS8                    = 0xa0, 
  BM_REG_TDBIAS9                    = 0xa4, 
  BM_REG_TDBIAS10                   = 0xa8, 
  BM_REG_TDBIAS11                   = 0xac, 
  BM_REG_TDBIAS12                   = 0xb0, 
  BM_REG_TDBIAS13                   = 0xb4, 
  BM_REG_TDBIAS14                   = 0xb8, 
  BM_REG_TDBIAS15                   = 0xbc, 
  BM_REG_TDBIAS16                   = 0xc0, 
  BM_REG_TDBIAS17                   = 0xc4, 
  BM_REG_TDBIAS18                   = 0xc8, 
  BM_REG_TDBIAS19                   = 0xcc, 
  BM_REG_TDBIAS20                   = 0xd0, 
  BM_REG_TDBIAS21                   = 0xd4, 
  BM_REG_TDBIAS22                   = 0xd8, 
  BM_REG_TDBIAS26                   = 0xdc, 
  BM_REG_VPEDLEFT                   = 0xe0, 
  BM_REG_VPEDRIGHT                  = 0xe4 
}e_bm_reg; 


// FPGA REGISTERS
typedef enum
{
  //ID_CTRL space
  RAD_REG_IDENT                       = 0x0, 
  RAD_REG_DATEVERSION                 = 0x4, 
  RAD_REG_CPLD_CTRL                   = 0x8, 
  RAD_REG_CHANNEL_DIS                 = 0xc, 
  RAD_REG_PPSSEL                      = 0x10, 
  RAD_REG_RESET_MODE                  = 0x14, 
  RAD_REG_LED                         = 0x18, 
  RAD_REG_JTAG_LEFT                   = 0x1c, 
  RAD_REG_JTAG_RIGHT                  = 0x20, 
  RAD_REG_SPISS                       = 0x24, 
  RAD_REG_DEVICEDNA                   = 0x2c, 
  RAD_REG_SIMPLE_SPI                  = 0x30, // to 0x3f 


  /// SPIDMA space
  RAD_REG_SPIDMA_CONFIG               = 0x8000, 
  RAD_REG_SPIDMA_CONTROL              = 0x8004, 
  RAD_REG_SPIDMA_CUR_DESCR_NUM        = 0x8008, 
  RAD_REG_SPIDMA_TXN_COUNT            = 0x800c, 
  RAD_REG_SPIDMA_CUR_DESCR_VAL        = 0x8010, 
  RAD_REG_SPIDMA_CUR_DESCR_COUNT      = 0x8014, 
  RAD_REG_SPIDMA_TXFIFOCNT            = 0x8018, 
  RAD_REG_SPIDMA_SPICC_COUNT          = 0x801c, 
  RAD_REG_SPIDMA_DESCR_BASE           = 0x8080, 
  RAD_REG_SPIDMA_DESCR_INCR           = 0x4,  // NOT AN ADDRESS

  //LABController
  RAD_REG_LAB_CTRL_CONTROL            = 0x10000, 
  RAD_REG_LAB_CTRL_TRIGGER            = 0x10054, 
  RAD_REG_LAB_CTRL_READOUT            = 0x10058, 
  RAD_REG_LAB_CTRL_READOUTEMPTY       = 0x10062, 

  //LABRAM space 
  RAD_REG_LAB_RAM_BASE                = 0x20000, 
  RAD_REG_LAB_RAM_INCR                = 0x800,  //2048  ?!??? 


  //TRIG SPACE
  RAD_REG_EV_CTRL                     =  0x30000, 
  RAD_REG_EV_PPS                      =  0x30004, 
  RAD_REG_EV_SYSCLK                   =  0x30008, 
  RAD_REG_EV_LASTSYSCLK               =  0x3000c, 
  RAD_REG_EV_FIFO_BASE                =  0x30100, 
  RAD_REG_TRIG_THRESH_OEB             = 0x30204, 
  RAD_REG_TRIG_THRESH_BASE            = 0x30300, 
  RAD_REG_TRIG_THRESH_INCR            = 0x04,

  RAD_REG_TRIG_OVLDCFG                = 0x30400, 
  RAD_REG_TRIG_OVLDCTRL               = 0x30404, 
  RAD_REG_TRIG_OVLDCHCFG              = 0x30408, 

  RAD_REG_TRIG_MASTEREN               = 0x30600, 
  RAD_REG_TRIG_TRIGINEN               = 0x30604, 
  RAD_REG_TRIG_PULSECTRL              = 0x30608, 

  RAD_REG_TRIG_TRIGEN0                = 0x30700, 
  RAD_REG_TRIG_TRIGMASKB0             = 0x30704, 
  RAD_REG_TRIG_TRIGWINDOW0            = 0x30708, 
  RAD_REG_TRIG_TRIGTHRESH0            = 0x3070c,  //this is number of channels that must pass

  RAD_REG_TRIG_TRIGEN1                = 0x30710, 
  RAD_REG_TRIG_TRIGMASKB1             = 0x30714, 
  RAD_REG_TRIG_TRIGWINDOW1            = 0x30718, 
  RAD_REG_TRIG_TRIGTHRESH1            = 0x3071c,  


  //SCALER space
  RAD_REG_SCALER_PERIOD                 = 0x40000, 
  RAD_REG_SCALER_PRESCALECTL            = 0x40004, 
  RAD_REG_SCALER_SCALMAP_BASE           = 0x40080, 
  RAD_REG_SCALER_SCAL_BASE              = 0x40800, 

  //CALRAM Space ? 
  RAD_REG_CALRAM_BASE               = 0x80000, 
  RAD_REG_CALRAM_GLOBAL_CONTROL     = 0xe0000, 
  RAD_REG_CALRAM_GLOBAL_MODE        = 0xe0004, 
  RAD_REG_CALRAM_GLOBAL_ROLLCOUNT   = 0xe0008 


}e_fpga_reg; 


struct fpga_dma_descr
{
  uint32_t address : 18; 
  uint32_t increment : 1; 
  uint32_t cyclecount : 12; 
  uint32_t last : 1; 
}; 

//Only implementing these two for now :) 
typedef enum 
{
  CALRAM_MODE_NONE, 
  CALRAM_MODE_PED, 
  CALRAM_MODE_ZERO  // not really a mode, jsut used to avoid confusing the radiant 
}calram_mode_t; 



static uint32_t ffffffff = 0xffffffff;
// A lot of things seem to return this is thy're bad
static uint8_t*  BM_BAD_READ = (uint8_t*) &ffffffff; 

enum 
{
  RADIANT_TRIG_ONEBUFMODE = 0x80000000, 
  RADIANT_TRIG_TWOBUFMODE = 0x81000000

} e_trig_settings; 


// this is the correct order, apparently :) 
typedef struct
__attribute__((packed))
date_version
{
    unsigned rev :8  ; 
    unsigned minor : 4; 
    unsigned major : 4; 
    unsigned day: 5;
    unsigned month: 4; 
    unsigned year: 7; 
} date_version_t; 


//TODO decide if these need to be exposed or not
#define RADIANT_NBUF 4
#define RADIANT_WINDOW_SIZE 128 
#define RADIANT_NSAMP_PER_BUF 1024
#define RADIANT_NWIND_PER_BUF 8

/** The radiant device structure. To be optimized for padding */ 
struct radiant_dev
{
  int spi_fd; 
  int uart_fd; 
  int interrupt_fd; 
  int run; 
  int station; 
  int peek; // use peek form of read

  // buffers for cobs writing (one per device) 
  uint8_t reg_buf [258]; 
  uint8_t reg_buf_encoded [260]; 

  //the board manager date veesion
  date_version_t bm_dateversion; 
  date_version_t rad_dateversion; 
  uint32_t rad_dateversion_int; 
  uint32_t cpldctrl; 

  // the gpio statuses. This is read on startup and (hopefully!) kept in sync so we don't have to keep reading it. 
  uint8_t gpio_status[7]; 
  int paranoid_about_gpios; //way to make us always check

  pthread_t interrupt_thread; 
  void * interrupt_aux; 
  void (*interrupt_callback)(radiant_dev_t*, void*); 
  volatile int interrupt_thread_stop; 

  struct pollfd interrupt_fdset; 
  uint16_t readout_nsamp; 
  uint32_t readout_mask; 
  int cleanup_spi_gpio; 
  int nbuffers_per_readout; // 1 for 1024 mode, 2 for 2048 mode
  int calram; //calram mode
  const rno_g_pedestal_t * peds; 
  uint16_t vbias[2]; 
  uint32_t triginen; 
  uint32_t trigAen; 
  uint32_t trigBen; 
  uint8_t prescal[RNO_G_NUM_RADIANT_CHANNELS]; 
  uint32_t thresh[RNO_G_NUM_RADIANT_CHANNELS]; 
  uint32_t pedram[RNO_G_PEDESTAL_NSAMPLES]; 
  double read_timeout; 
  int triggers_per_cycle; 
  double sleep_per_cycle;
  struct timespec sleep_per_cycle_ts; 

}; 


static int do_poll(radiant_dev_t * bd, int timeout) 
{

  if (!bd) return -1; 

  char val; 
  lseek(bd->interrupt_fd,0,SEEK_SET); 
  read(bd->interrupt_fd, &val,1); 
  if (val=='1') return 1; 
  int rc= poll(&bd->interrupt_fdset,1,timeout); 


  if (rc && (bd->interrupt_fdset.revents & POLLPRI))
  {
    lseek(bd->interrupt_fd,0,SEEK_SET); 
    read(bd->interrupt_fd, &val,1); 
    return val=='1'; 
  }
  //note that POLLPRI | POLLERR seems to be the defalut response, 
  //so checking this AFTER pollpri in case there's actually an error? 
  if (bd->interrupt_fdset.revents & POLLERR) 
  {
    return errno; 
  }



  if (rc == 0) return 0; 

  return -1; 

}

static void* radiant_interrupt_thread_func(void * v) 
{
  radiant_dev_t * bd = (radiant_dev_t*) v; 


  while(!bd->interrupt_thread_stop) 
  {
    if (do_poll(bd, -1)) 
    {
      bd->interrupt_callback(bd, bd->interrupt_aux); 
    }
  }
  return 0; 
}

int radiant_poll_trigger_ready(radiant_dev_t *bd, int timeout) 
{
  if (!bd) return -1; 
  if (!bd->interrupt_fd) return EIO; 
  if (bd->interrupt_callback) return EBUSY; 
  return do_poll(bd,timeout); 
}



int radiant_set_trigger_ready_callback(radiant_dev_t * bd, void (*callback)(radiant_dev_t *, void*), void * aux) 
{

  if (!bd) return -1; 

  //No file descriptor to wait on. return -1; 
  if (!bd->interrupt_fd) 
  {
    return EIO; 
  }

  //if we had a previous call back, stop the thread

  if (bd->interrupt_callback) 
  {
    bd->interrupt_thread_stop = 1; 
    pthread_kill(bd->interrupt_thread,SIGUSR1); 
    pthread_join(bd->interrupt_thread,0); 
  }
  
  bd->interrupt_callback = callback; 
  bd->interrupt_aux = aux; 

  //If callback is not null, start a thread
  if (callback) 
  {
    return  pthread_create(&bd->interrupt_thread,0,radiant_interrupt_thread_func, bd); 
  }

  return 0;
}

void radiant_set_run_number(radiant_dev_t * bd, int run) 
{
  if (bd) bd->run = run; 
}
void radiant_set_station_number(radiant_dev_t * bd, int station) 
{
  if (bd) bd->station = station; 
}



void radiant_set_read_mode(radiant_dev_t *bd, int peek) 
{
  if (bd) bd->peek = peek; 
}

int radiant_read(radiant_dev_t * bd, int nbufs, uint16_t * N, uint8_t **bufs)
{
  if (!bd) return -1; 

  uint16_t nxfers = (nbufs) & 0x1ff;  // only up to 511 transfers are supported . This also prevents a potential stack overflow.  
#ifdef RADIANT_SET_DBG
  printf("DBG: radiant_read (nxfers=%d)\n", nxfers); 
#endif
  struct spi_ioc_transfer xfers[nxfers] ; 
  memset(xfers,0,sizeof(struct spi_ioc_transfer) * nxfers);

  const int cs_change = 0; 
  for (int i = 0; i < nxfers; i++)
  {
#ifdef RADIANT_SET_DBG
    printf("DBG:   xfer_%d: len=%d, rx_buf=0x%p, tx_buf=0,cs_change=%d\n", i, N[i], bufs[i], cs_change); 
#endif
    xfers[i].tx_buf = 0; 
    xfers[i].cs_change = cs_change; 
    xfers[i].rx_buf = (uintptr_t) bufs[i]; 
    xfers[i].len =  N[i]; 
  }

#ifdef RADIANT_SET_DBG
  struct timespec start; 
  struct timespec stop; 
  clock_gettime(CLOCK_REALTIME,&start); 
#endif

  int ret =  ioctl(bd->spi_fd, SPI_IOC_MESSAGE(nxfers), xfers); 
#ifdef RADIANT_SET_DBG
  clock_gettime(CLOCK_REALTIME,&stop); 
  double elapsed = stop.tv_sec - start.tv_sec + 1e-9*(stop.tv_nsec - start.tv_nsec); 
  double MBps = ret / elapsed / (1024*1024); 
  printf("DBG: ioctl(%d,%d,0x%p) = %d, %g seconds, %g MByte/s", bd->spi_fd, SPI_IOC_MESSAGE(nxfers), xfers, ret, elapsed, MBps); 

  int itx = 0; 
  uint32_t i_in_tx = 0; 
  for (int i = 0; i < ret; i++) 
  {

    if (i_in_tx >= xfers[itx].len)
    {
      i_in_tx=0; 
      itx++; 
    }

    if (i_in_tx==0) 
    {
      printf("\nDBG: TX%03d:", itx); 
    } 

    if (i_in_tx % 80 == 0) printf("\nDBG:      "); 
    uint8_t* buf = bufs[itx]; 
    printf(" 0x%x", buf[i_in_tx++]); 
  }
  printf("\n"); 
#endif
  if (ret < 0) 
  {
    printf("!! ioctl in radiant_read returned %d, errno: %d: %s\n", ret, errno, strerror(errno)); 
  }

  return ret; 
}


static int get_gcd(int a, int b) 
{
  if (!b) return a; 
  else return get_gcd(b,a%b); 
}

//TODO figure out how to vectorize this (and maybe also remove high bits while we go?) 
// for now use algorithm from https://www.geeksforgeeks.org/array-rotation/ but this can definitely be optimized, I think 
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
      while (k >= N) k-=N; 
      while (k < 0) k+=N; 
      if (i==k) break;
      v[j]=v[k]; 
      j=k; 
    }
    v[j] = tmp;
  }
}

//TODO: if we need to and with other values, can used parameterized macros to fake templates :)
//
static void andall16_0x0fff(uint16_t *v, int N) 
{

  static const uint16_t andme = 0x0fff; 

  int i = 0;
#if defined(__arm__) && !defined(NOVECTORIZE) 
  static uint16x8_t vandme_0x0fff = { 0x0fff, 0x0fff, 0x0fff, 0x0fff,
                                      0x0fff, 0x0fff, 0x0fff, 0x0fff}; 
//we'll use ARM NEON intrinsics. Since usually this will be a multiple of 32, we'll use vandq_u16 unrolled by 4 
  int Niter = N/32; 

  if (Niter) 
  {
    for (i = 0; i < Niter; i++) 
    {
      uint16x8_t v1 = vld1q_u16(v+i*32);
      uint16x8_t v2 = vld1q_u16(v+i*32+8);
      uint16x8_t v3 = vld1q_u16(v+i*32+16);
      uint16x8_t v4 = vld1q_u16(v+i*32+24);
      v1 = vandq_u16(v1,vandme_0x0fff);
      v2 = vandq_u16(v2,vandme_0x0fff);
      v3 = vandq_u16(v3,vandme_0x0fff);
      v4 = vandq_u16(v4,vandme_0x0fff);
      vst1q_u16(v+i*32,v1);
      vst1q_u16(v+i*32+8,v2);
      vst1q_u16(v+i*32+16,v3);
      vst1q_u16(v+i*32+24,v4);
    }

    i*=32; // catch any remaining elements with a normal loop
  }
#endif 


  // finish the last (or do all of them, if not using intrinsics) 
  for (; i < N; i++) 
  {
    v[i] &= andme; 
  }
}

static void sub16(int N, int16_t * A, const int16_t * B) 
{
  int i = 0;
#if defined(__arm__) && !defined(NOVECTORIZE) 
//we'll use ARM NEON intrinsics. Since usually this will be a multiple of 32, we'll use vsubq_s16 unrolled by 4 
  int Niter = N/32; 

  if (Niter) 
  {
    for (i = 0; i < Niter; i++) 
    {
      int16x8_t A1 = vld1q_s16(A+i*32);
      int16x8_t A2 = vld1q_s16(A+i*32+8);
      int16x8_t A3 = vld1q_s16(A+i*32+16);
      int16x8_t A4 = vld1q_s16(A+i*32+24);
      int16x8_t B1 = vld1q_s16(B+i*32);
      int16x8_t B2 = vld1q_s16(B+i*32+8);
      int16x8_t B3 = vld1q_s16(B+i*32+16);
      int16x8_t B4 = vld1q_s16(B+i*32+24);
 
      A1 = vsubq_s16(A1,B1);
      A2 = vsubq_s16(A2,B2);
      A3 = vsubq_s16(A3,B3);
      A4 = vsubq_s16(A4,B4);
      vst1q_s16(A+i*32,A1);
      vst1q_s16(A+i*32+8,A2);
      vst1q_s16(A+i*32+16,A3);
      vst1q_s16(A+i*32+24,A4);
    }

    i*=32; // catch any remaining elements with a normal loop
  }
#endif 
 
  for ( ; i < N; i++) 
  {
    A[i] -= B[i]; 
  }
}




/** simd version of bswap, no longer needed */ 

/*
static void bswap16(uint16_t *vv, int N) 
{
  int i = 0;
  uint8_t*v = (uint8_t*) vv; 

#if defined(__arm__) && !defined(NOVECTORIZE) 
//we'll use ARM NEON intrinsics. Since usually this will be a multiple of 32, we'll use vandq_u16 unrolled by 4 
  int Niter = N/32; 

  if (Niter) 
  {
    for (i = 0; i < Niter; i++) 
    {
      uint8x16_t v1 = vld1q_u8(v+i*64);
      uint8x16_t v2 = vld1q_u8(v+i*64+16);
      uint8x16_t v3 = vld1q_u8(v+i*64+32);
      uint8x16_t v4 = vld1q_u8(v+i*64+48);
      v1 = vrev16q_u8(v1);
      v2 = vrev16q_u8(v2);
      v3 = vrev16q_u8(v3);
      v4 = vrev16q_u8(v4);
      vst1q_u8(v+i*64,v1);
      vst1q_u8(v+i*64+16,v2);
      vst1q_u8(v+i*64+32,v3);
      vst1q_u8(v+i*64+48,v4);
    }

    i*=32; // catch any remaining elements with a normal loop
  }
#endif 


  // finish the last (or do all of them, if not using intrinsics) 
  for (; i < N; i++) 
  {
    vv[i] = __builtin_bswap16(vv[i]); 
  }
}
*/ 




int radiant_read_event(radiant_dev_t * bd, rno_g_header_t * hd, rno_g_waveform_t *wf) 
{

  radiant_raw_event_info_t raw_info; 
  int ret =radiant_read_raw_event(bd, &raw_info, wf); 
  if (ret < 0) return ret; 
  int ret2= radiant_process_raw_event(bd, &raw_info, hd, wf); 
  return ret2 ?: ret; 
}


int radiant_read_raw_event(radiant_dev_t *bd, radiant_raw_event_info_t * raw_info, rno_g_waveform_t *wf)
{
  if (!bd) return -1 ; 

  uint16_t Ns[2+RNO_G_NUM_RADIANT_CHANNELS]; 
  uint8_t* bufs[2+RNO_G_NUM_RADIANT_CHANNELS]; 

  Ns[0] = sizeof(raw_info->fwhd); 
  bufs[0] = (uint8_t*) &(raw_info->fwhd); 

  int nsamples = bd->readout_nsamp; 
  int iactual = 0;
  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
  {
    if (bd->readout_mask & (1 << i))
    {
      Ns[1+iactual] = 2*nsamples; 
      bufs[1+iactual] = (uint8_t*) wf->radiant_waveforms[i]; 
      iactual++; 
    }
  }

  int nactual = iactual; 


  clock_gettime(CLOCK_REALTIME, &raw_info->read_start); 
  int ret = radiant_read(bd, 1+nactual, Ns,bufs); 
  clock_gettime(CLOCK_REALTIME, &raw_info->read_end); 

  //check magic 
  if (raw_info->fwhd.magic!= EVENT_MAGIC )
  {
    fprintf(stderr,"Bad magic :%x\n", raw_info->fwhd.magic);
    return -1; 
  }

  return ret;
}


int radiant_process_raw_event(radiant_dev_t * bd, const radiant_raw_event_info_t *raw_info, rno_g_header_t * hd, rno_g_waveform_t *wf)
{
  int nsamples = raw_info->nsamp; 
  hd->event_number = raw_info->fwhd.count;
  hd->pps_count = raw_info->fwhd.pps; 
  hd->sys_clk = raw_info->fwhd.sysclk; 
  hd->readout_time_secs = raw_info->read_start.tv_sec; 
  hd->readout_time_nsecs = raw_info->read_end.tv_nsec; 
  hd->readout_elapsed_nsecs = (raw_info->read_end.tv_nsec - raw_info->read_start.tv_nsec) + 1000000000 * (raw_info->read_end.tv_sec-raw_info->read_start.tv_sec); 
  hd->run_number = bd->run; 
  hd->station_number = bd->station; 
  hd->sysclk_last_pps = raw_info->fwhd.sysclk_last_pps; 
  hd->sysclk_last_last_pps = raw_info->fwhd.sysclk_last_last_pps; 
  hd->radiant_nsamples = nsamples; 

  hd->raw_evstatus = raw_info->fwhd.status_flags; 
  hd->raw_tinfo = raw_info->fwhd.trig_info; 
  hd->trigger_type = 0 ;

  if (hd->raw_tinfo & RADIANT_TRIGGER_INT0 ) hd->trigger_type |= RNO_G_TRIGGER_RF_RADIANT0;
  if (hd->raw_tinfo & RADIANT_TRIGGER_INT1 ) hd->trigger_type |= RNO_G_TRIGGER_RF_RADIANT1;
  if (hd->raw_tinfo & RADIANT_TRIGGER_EXT ) hd->trigger_type  |= RNO_G_TRIGGER_RF_LT_SIMPLE;
  if (hd->raw_tinfo & RADIANT_TRIGGER_SOFT ) hd->trigger_type |= RNO_G_TRIGGER_SOFT;
  if (hd->raw_tinfo & RADIANT_TRIGGER_PPS ) hd->trigger_type  |= RNO_G_TRIGGER_PPS;
  if (hd->raw_tinfo & RADIANT_TRIGGER_INTERNAL_COPY ) hd->trigger_type  |= RNO_G_TRIGGER_RF_RADIANTX;

  wf->event_number = hd->event_number; 
  wf->run_number = bd->run; 
  wf->station= bd->station; 

  wf->radiant_nsamples = nsamples;

  for (int ichan = 0; ichan < RNO_G_NUM_RADIANT_CHANNELS; ichan++) 
  {

    if (raw_info->readout_mask & (1 << ichan))
    {

      //if we are in 1024-sample mode, ony one start window per event, so set the second to a magic value. 
      if (raw_info->nbuffers_per_readout == 1) 
      {
        hd->radiant_start_windows[ichan][1] = 0xff; 
      }

      //for post v0.3.0
      int nboth_rotate = 0;

      //Loop over the readout buffers 
      //TODO: there may be some efficiency gains in doing the andall and sub16 operations on both buffers at the same time... but probably not huge and the code is simpler this way 
      for (int ibuffer = 0; ibuffer < raw_info->nbuffers_per_readout; ibuffer++) 
      {

        //get buffer number (TODO: demagicify this) 
        int buffer_number = (wf->radiant_waveforms[ichan][ibuffer * RADIANT_NSAMP_PER_BUF] & 0xc000) >> 14; 

        //now let's find the STOP window 
        int nrotate = 0;

        for (int w = 0; w < RADIANT_NWIND_PER_BUF; w++) 
        {
          uint16_t val = wf->radiant_waveforms[ichan][RADIANT_WINDOW_SIZE*w+ibuffer * RADIANT_NSAMP_PER_BUF];

          //TODO: demagicify this 
          if (val & 0x2000)//this is the STOP window. Means the START window is snext
          {
            hd->radiant_start_windows[ichan][ibuffer] =  buffer_number * RADIANT_NWIND_PER_BUF  + ((w+1) % (RADIANT_NWIND_PER_BUF)); 
            nrotate = (w+1) * RADIANT_WINDOW_SIZE; 
            if (bd->rad_dateversion_int >= 300  && ibuffer == 0 && bd->nbuffers_per_readout == 2) nboth_rotate = nrotate; 
            break; 
          }
        }

        //TODO: in some cases (buffer 0, not start or stop window) we can avoid doing this on all samples 
        andall16_0x0fff((uint16_t*)wf->radiant_waveforms[ichan]+ibuffer * RADIANT_NSAMP_PER_BUF,RADIANT_NSAMP_PER_BUF); 

        // Now subtract the pedestals
        if (bd->peds)
        {
          sub16 (RADIANT_NSAMP_PER_BUF,  wf->radiant_waveforms[ichan] + ibuffer * RADIANT_NSAMP_PER_BUF, (int16_t*) bd->peds->pedestals[ichan] + RADIANT_NSAMP_PER_BUF * buffer_number); 
        }

//        unwrap each buffer individually, if we need to 
        if (nrotate &&  ( bd->rad_dateversion_int < 300 || bd->nbuffers_per_readout == 1) ) 
        {
          roll16((uint16_t*)wf->radiant_waveforms[ichan] + ibuffer * RADIANT_NSAMP_PER_BUF, nrotate, RADIANT_NSAMP_PER_BUF); 
        }
      }

      //If the two buffers are out of order, rearrange them
      if (bd->nbuffers_per_readout == 2 && bd->rad_dateversion_int >= 300 && nboth_rotate)
      {
        roll16((uint16_t*)wf->radiant_waveforms[ichan], nboth_rotate, 2*RADIANT_NSAMP_PER_BUF); 
      }

    }
    else
    {
      memset(wf->radiant_waveforms[ichan],0, sizeof(wf->radiant_waveforms[ichan])); 
    }
  }

  return 0; 
}


static int read_bm_gpios(radiant_dev_t * dev) 
{
  // read in the GPIO status 
  for (int i = 0; i < 7; i++) 
  {
    unsigned gpioval; 
    int nb = radiant_get_mem(dev,DEST_MANAGER, BM_REG_GPIO_BASE + BM_REG_GPIO_INCR*i, sizeof(gpioval),  (uint8_t*) &gpioval); 
    if (nb!=4 || gpioval == 0xffffffff) 
    {
      fprintf(stderr, "Could not query Board Manager GPIO%d bits!\n",i); 
      return 1; 
    }
    dev->gpio_status[i] = gpioval &0xff; 
  }
  return 0; 
}
  

static int write_bm_gpio(radiant_dev_t * dev, int which) 
{
  int gpioval = dev->gpio_status[which]; 
  return 4 != radiant_set_mem(dev, DEST_MANAGER, BM_REG_GPIO_BASE+BM_REG_GPIO_INCR* which, sizeof(gpioval), (uint8_t*) &gpioval); 
}
 

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


radiant_dev_t * radiant_open(const char *spi_device, const char * uart_device, int trig_gpio, int spi_en_gpio) 
{

  radiant_dev_t * dev = 0; 
  int locked_spi, locked_uart, spi_fd = 0, uart_fd =0; 

  //open the device, if we can 
  spi_fd = open(spi_device, O_RDWR); 
  if (spi_fd < 0) 
  {
    fprintf(stderr,"Could not open %s\n", spi_device); 
    return 0; 
  }
  uart_fd = open(uart_device, O_RDWR); 

  if (uart_fd < 0) 
  {
    fprintf(stderr,"Could not open %s\n", uart_device); 
    close(spi_fd); 
    return 0; 
  }


  //advisory locks, so we don't accidentally open it twice (how does this work with symlinks?) 
  locked_spi = flock(spi_fd, LOCK_EX | LOCK_NB);
  locked_uart = flock(uart_fd, LOCK_EX | LOCK_NB);
  if (locked_spi < 0 || locked_uart < 0) 
  {
    if (locked_spi < 0) 
      fprintf(stderr,"Could not get exclusive access to %s\n", spi_device); 
    if (locked_uart < 0) 
      fprintf(stderr,"Could not get exclusive access to %s\n", uart_device); 
    close(spi_fd); 
    close(uart_fd); 
    return 0; 
  }


  //now set up the SPI 
  int spi_clock = RADIANT_SPI_SPEED * 1000000; 
  ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ,&spi_clock); 

  uint8_t mode = 0; 
  uint8_t bits_per_word = 32; 
  ioctl(spi_fd, SPI_IOC_WR_MODE,&mode); 
  //set it up in 32 bitsperword mode
  ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD,&bits_per_word); 
  
  //and set up the UART 
  // SEE THIS NICE GUIDE HERE https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp

  struct termios tty; 


  tcgetattr(uart_fd,  &tty); 

  //clear parity bit 
  tty.c_cflag &= ~PARENB; 
  
  //one stop bit
  tty.c_cflag &= ~CSTOPB; 

  //8 bits 
  tty.c_cflag &= ~CSIZE; 
  tty.c_cflag |= CS8; 

  //no hw flow control 
  tty.c_cflag &=~CRTSCTS; 

  //turn on read/disable ctrl lines
  tty.c_cflag |= CREAD | CLOCAL; 

  //turn OFF canoncial mode
  tty.c_lflag &= ~ICANON; 

  //disable echo bits (probably already disabled?) 
  tty.c_lflag &= ~ECHO; 
  tty.c_lflag &= ~ECHOE; 
  tty.c_lflag &= ~ECHONL;

  //disable interpretation of signal chars
  tty.c_lflag &= ~ISIG; 

  //disable software flow control 
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); 

  //disable special handling of input  bytes
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|INLCR|ICRNL); 

  //disable any special output modes 
  tty.c_oflag &= ~OPOST; 
  tty.c_oflag &= ~ONLCR; 


  //make it nonblocking
  tty.c_cc[VTIME]=0; 
  tty.c_cc[VMIN]=0; 

  //baud rate  
  cfsetispeed(&tty, B1000000); //blazing fast! 
  cfsetospeed(&tty, B1000000);

  //set the serial attrs 
  if (0 != tcsetattr(uart_fd, TCSANOW, &tty))
  {
    fprintf(stderr,"Could not configure serial port %s :(. Got error %d: %s\n", uart_device, errno, strerror(errno)); 
    close(uart_fd); 
    close(spi_fd); 
    return 0; 
  }

  //drain the port 
  tcflush(uart_fd, TCIOFLUSH); 

  //Write a few 0s to make sure we're synchronized
  uint16_t zeros[2] = {0}; 
  write(uart_fd,&zeros,sizeof(zeros)); 


  dev = calloc(sizeof(radiant_dev_t),1); 
  dev->spi_fd = spi_fd; 
  dev->uart_fd = uart_fd; 
  dev->read_timeout = 1; 

  // verify that we identify correctly
  char check_bm[4]; 
  int nb = radiant_get_mem(dev, DEST_MANAGER, BM_REG_IDENT, 4, (uint8_t*) check_bm); 

  if (nb != 4 || memcmp(check_bm, "MBDR",4)) //little endian output
  {
    fprintf(stderr, "RADIANT BOARD MANAGER DID NOT IDENTIFY PROPERLY. GOT \"%c%c%c%c\"\n", check_bm[3], check_bm[2], check_bm[1], check_bm[0]); 
    radiant_close(dev); 
    return 0; 
  }
  
  char check_radiant[4]; 
  nb = radiant_get_mem(dev, DEST_FPGA, RAD_REG_IDENT, 4, (uint8_t*) check_radiant); 

  if (nb!=4 || memcmp(check_radiant,"TNDR",4))
  {
    fprintf(stderr, "RADIANT DID NOT IDENTIFY PROPERLY. GOT \"%c%c%c%c\"\n", check_radiant[3], check_radiant[2], check_radiant[1], check_radiant[0]); 
    radiant_close(dev); 
    return 0; 
  }

  // read in the BM version information 
  nb = radiant_get_mem(dev, DEST_MANAGER, BM_REG_DATEVERSION, 4, (uint8_t *) &dev->bm_dateversion);
  if (nb!=4 || !memcmp(BM_BAD_READ, &dev->bm_dateversion,4)) 
  {
    fprintf(stderr, "Could not read version from RADIANT Board Manager\n"); 
    radiant_close(dev); 
    return 0; 
  }

  // read in the radiant version information 
  nb = radiant_get_mem(dev, DEST_FPGA, RAD_REG_DATEVERSION, 4, (uint8_t *) &dev->rad_dateversion);
  if (nb!=4 || !memcmp(BM_BAD_READ, &dev->rad_dateversion,4)) 
  {
    fprintf(stderr, "Could not read version from RADIANT\n"); 
    radiant_close(dev); 
    return 0; 
  }

  dev->rad_dateversion_int=10000*dev->rad_dateversion.major + 100 * dev->rad_dateversion.minor + dev->rad_dateversion.rev; 

  //read in the radiant CPLD_CTRL, check the programmed bits 
  nb = radiant_get_mem(dev, DEST_FPGA, RAD_REG_CPLD_CTRL, 4, (uint8_t*) &dev->cpldctrl); 
  {
    if (nb !=4 || !memcmp(BM_BAD_READ, &dev->cpldctrl,4))
    {

      fprintf(stderr, "Could not read cpldctrl from RADIANT\n"); 
      radiant_close(dev); 
      return 0; 
    }
  }

  uint32_t check_bits = (1u << 31) | (1 <<15); 

  if ((dev->cpldctrl & check_bits)  != check_bits) 
  {
    fprintf(stderr, "CPLD_CTRL programmed bits not set! Check programming?\n"); 
    radiant_close(dev); 
    return 0; 
  }

  if (dev->rad_dateversion_int > 227) 
  {
    nb = radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_TRIGINEN, 4, (uint8_t*) &dev->triginen); 
    if (nb!=4) 
    {
      fprintf(stderr, "Could not read TRIGINEN from RADIANT\n"); 
      radiant_close(dev); 
      return 0; 
    }
    nb = radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_TRIGEN0, 4, (uint8_t*) &dev->trigAen); 
    if (nb!=4) 
    {
      fprintf(stderr, "Could not read TRIGINEN0 from RADIANT\n"); 
      radiant_close(dev); 
      return 0; 
    }
    nb = radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_TRIGEN1, 4, (uint8_t*) &dev->trigBen); 
    if (nb!=4) 
    {
      fprintf(stderr, "Could not read TRIGINEN1 from RADIANT\n"); 
      radiant_close(dev); 
      return 0; 
    }

  }


  if (read_bm_gpios(dev))
  {
    radiant_close(dev); 
    return 0; 

  }


  //Set up the trigger interrupt fd and fdset 
  if (trig_gpio > 0) 
  {
    char buf[512]; 
    export_gpio_if_not_exported(trig_gpio); 
   
    //set the edge to rising 
    sprintf(buf, "/sys/class/gpio/gpio%d/edge", trig_gpio); 
    int edge_fd = open(buf, O_RDWR); 

    if (edge_fd <=0) 
    {
      fprintf(stderr, "Could not open %s for writing, trig gpio won't be set up\n", buf); 
    }
    else
    {
      write(edge_fd,"rising",strlen("rising")); 
      close(edge_fd); 

      sprintf(buf,"/sys/class/gpio/gpio%d/value", trig_gpio); 
      dev->interrupt_fd = open(buf,O_RDWR); 

      if (dev->interrupt_fd > 0) 
      {
        int locked_interrupt = flock(dev->interrupt_fd, LOCK_EX | LOCK_NB);
        if (locked_interrupt) 
        {
          dev->interrupt_fd = 0; 
          fprintf(stderr,"Could not lock interrupt gpio\n"); 
        }
        else
        {
        memset(&dev->interrupt_fdset,0,sizeof(dev->interrupt_fdset)); 
        dev->interrupt_fdset.fd = dev->interrupt_fd; 
        dev->interrupt_fdset.events = POLLPRI; 
        }
      }
      else
      {
        fprintf(stderr, "Could not open %s for writing, trig gpio won't be set up\n", buf); 
      }
    }
  }

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
//    dev->cleanup_spi_gpio = gpionum;
  }
  else dev->cleanup_spi_gpio = 0; 


  dev->readout_mask = 0xffffff; 
  dev->run = 0; 

  dev->nbuffers_per_readout = 1; 
  dev->readout_nsamp = dev->nbuffers_per_readout * RADIANT_NSAMP_PER_BUF ; 
  dev->peds = 0; 
  dev->calram = CALRAM_MODE_NONE; 
  dev->triggers_per_cycle = 128; 

  // We don't know what the vbias is since there is no readback implemented
  // so you have to set it if you want to know what it is :) , 
  dev->vbias[0] = -1; 
  dev->vbias[1] = -1; 

  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
  {
    dev->thresh[i] = RADIANT_THRESHOLD_UNKNOWN; 
  } 

  return dev; 
}


typedef enum 
{
  QGPIO_BIT_SEL_CAL = 1, 
  QGPIO_BIT_ATT_LE = 2, 
  QGPIO_BIT_BIST = 4, 
  QGPIO_BIT_LED = 8, 
  QGPIO_BIT_TRIG_EN = 16, 
  QGPIO_BIT_LAB_EN = 32, 
  QGPIO_BIT_TRIG_EN_STARTUP = 64, 
  QGPIO_BIT_LAB_EN_STARTUP = 128 
} e_quad_bits; 

typedef enum
{
  SGPIO_BIT_CAL_FIL0 = 1, 
  SGPIO_BIT_SIG_LE = 2, 
  SGPIO_BIT_CAL_FIL1 = 4, 
  SGPIO_BIT_N_CAL_FIL1 = 8, 
  SGPIO_BIT_CALPULSE  =16, 
  SGPIO_BIT_N_CALPULSE = 32, 
  SGPIO_BIT_SG_ENABLE = 64, 
  SGPIO_BIT_SG_MUXOUT= 128
} e_sig_bits; 


int radiant_dump(radiant_dev_t *dev, FILE * stream, int flags) 
{
  if (!dev) return -1; 
  if (!stream) stream =stdout ; 


  fprintf(stream,"=====RADIANT DUMP of handle at 0x%p======\n", dev); 
  fprintf(stream,"    BM_DATE_VERSION: %u.%u.%u (%u/%u/%u) \n", dev->bm_dateversion.major, dev->bm_dateversion.minor, dev->bm_dateversion.rev, 
                      dev->bm_dateversion.year, dev->bm_dateversion.month, dev->bm_dateversion.day ); 
  fprintf(stream,"    RAD_DATE_VERSION: %u.%u.%u (%u/%u/%u) = %d \n", dev->rad_dateversion.major, dev->rad_dateversion.minor, dev->rad_dateversion.rev, 
                      dev->rad_dateversion.year, dev->rad_dateversion.month, dev->rad_dateversion.day , dev->rad_dateversion_int); 


  if (flags & RADIANT_DUMP_UPDATE_GPIOS || dev->paranoid_about_gpios) 
  {
    read_bm_gpios(dev); 
  }

  for (int i = 0; i < 6; i++) 
  {
    fprintf(stream,
                 "    QUADGPIO%d:  SEL_CAL: %d, ATT_LE: %d, BIST: %d, LED: %s, TRIG_EN: %d,\n"
                 "                 LAB_EN: %d, TRIG_EN_STARTUP: %d, LAB_EN_STARTUP: %d\n", 
                 i, !!(dev->gpio_status[i] & QGPIO_BIT_SEL_CAL), !!(dev->gpio_status[i] & QGPIO_BIT_ATT_LE) , 
                 !!(dev->gpio_status[i] & QGPIO_BIT_BIST), (dev->gpio_status[i] & QGPIO_BIT_LED) ? "GREEN" : "RED", 
                 !!(dev->gpio_status[i] & QGPIO_BIT_TRIG_EN), !!(dev->gpio_status[i] & QGPIO_BIT_LAB_EN),
                 !!(dev->gpio_status[i] & QGPIO_BIT_TRIG_EN_STARTUP), !!(dev->gpio_status[i] & QGPIO_BIT_LAB_EN_STARTUP)); 
  }

  uint8_t sgpio_status = dev->gpio_status[6]; 
  fprintf(stream, "    SIGGPIO: CAL_FIL0: %d, SIG_LE %d, CAL_FIL1: %d, N_CAL_FIL1: %d \n"
                  "             CAL_PULSE: %d, N_CAL_PULSE: %d, SG_ENABLE: %d, SG_MUXOUT: %d \n", 
                  !!(sgpio_status & SGPIO_BIT_CAL_FIL0), !!(sgpio_status & SGPIO_BIT_SIG_LE), 
                  !!(sgpio_status & SGPIO_BIT_CAL_FIL1), !!(sgpio_status & SGPIO_BIT_N_CAL_FIL1), 
                  !!(sgpio_status & SGPIO_BIT_CALPULSE), !!(sgpio_status & SGPIO_BIT_N_CALPULSE), 
                  !!(sgpio_status & SGPIO_BIT_SG_ENABLE),!!(sgpio_status & SGPIO_BIT_SG_MUXOUT));

  rno_g_radiant_voltages_t voltages; 
  radiant_bm_analog_read_all(dev, &voltages);
  fprintf(stream, "    ANALOG: V1.0: %f, V1.8: %f, V2.5: %f, LEFTMON: %f, RIGHTMON: %f\n", 
      voltages.V_1_0*3.3/65535,voltages.V_1_8*3.3/65535,voltages.V_2_5*3.3/65535,voltages.V_LeftMon*3.3/65535,voltages.V_RightMon*3.3/65535); 
  uint8_t pulse_cfg[4]; 
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_PULSECTRL, 4, pulse_cfg); 
  int pulse_period = pulse_cfg[0] |  (pulse_cfg[1] << 8) | (pulse_cfg[2] << 16) | (( pulse_cfg[3] & 0x3f) << 24);
  pulse_period*=5; 
  printf("    PULSECTRL: PERIOD %d ns, SHARPEN: %d, DISABLE: %d\n", pulse_period, !!(pulse_cfg[3] & ( 1<<6)), !!(pulse_cfg[3] & (1 <<7))); 


  fprintf(stream, "    CPLDCTRL (cached): %x\n", dev->cpldctrl); 

  uint8_t cont[4]; 
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_LAB_CTRL_CONTROL,4,cont); 
  fprintf(stream, "    LAB4D_CONTROLLER_CONTROL: %x %x %x %x \n", cont[3], cont[2], cont[1], cont[0]); 


  radiant_dma_config_t dma_cfg; 
  radiant_get_dma_config(dev, &dma_cfg);  

  fprintf(stream, "    DMACONFIG:  dma_enable: %d, dma_busy; %d, ext_dma_req_enable: %d, dma_direction: %d\n", dma_cfg.dma_enable, dma_cfg.dma_busy, dma_cfg.ext_dma_req_enable, dma_cfg.dma_direction); 
  fprintf(stream, "                byte_mode: %d, byte_target; %d, enable_spi_receive: %d, cycle_delay: %d\n", dma_cfg.byte_mode, dma_cfg.byte_target_in_byte_mode, dma_cfg.enable_spi_receive, dma_cfg.cycle_delay); 
  fprintf(stream, "                tx_full_flag_thresh: %d, tx_full_flag_value; %d, tx_ful_flag_enable: %d\n", dma_cfg.tx_full_flag_threshold, dma_cfg.tx_full_flag_value, dma_cfg.tx_full_flag_enable); 
  uint32_t curr_descr_num, curr_descr_val, curr_descr_count, txn_count, txfifo_count, spicc_count ; 
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_SPIDMA_CUR_DESCR_NUM,4, (uint8_t*) &curr_descr_num); 
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_SPIDMA_TXN_COUNT,4, (uint8_t*) &txn_count); 
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_SPIDMA_CUR_DESCR_COUNT,4, (uint8_t*) &curr_descr_count); 
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_SPIDMA_CUR_DESCR_VAL,4, (uint8_t*) &curr_descr_val); 
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_SPIDMA_TXFIFOCNT,4, (uint8_t*) &txfifo_count); 
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_SPIDMA_SPICC_COUNT,4, (uint8_t*) &spicc_count); 
  fprintf(stream, "    DMA_CUR_DESCR_NUM  (0x%x): %u\n",RAD_REG_SPIDMA_CUR_DESCR_NUM, curr_descr_num); 
  fprintf(stream, "    DMA_CUR_DESCR_VAL  (0x%x): 0x%x\n",RAD_REG_SPIDMA_CUR_DESCR_VAL, curr_descr_val); 
  fprintf(stream, "    DMA_CUR_DESCR_COUNT(0x%x): %u\n", RAD_REG_SPIDMA_CUR_DESCR_COUNT, curr_descr_count); 
  fprintf(stream, "    DMA_TXN_COUNT      (0x%x): %u\n", RAD_REG_SPIDMA_TXN_COUNT, txn_count); 
  fprintf(stream, "    DMA_TXFIFO_COUNT:  (0x%x): %u\n", RAD_REG_SPIDMA_TXFIFOCNT, txfifo_count); 
  fprintf(stream, "    DMA_SPI_CC_COUNT:  (0x%x): %u\n", RAD_REG_SPIDMA_SPICC_COUNT, spicc_count); 

  for (int i = 0; i < 32; i++) 
  {
    struct fpga_dma_descr dma; 
    uint32_t addr = RAD_REG_SPIDMA_DESCR_BASE + i * RAD_REG_SPIDMA_DESCR_INCR; 
    radiant_get_mem(dev, DEST_FPGA, addr,4,(uint8_t*) &dma); 
    printf ("     DMA_DESC_%02d (at 0x%x):   addr: 0x%x , incr: %d, len: %d, last:%d\n", i, addr, dma.address << 2, dma.increment, dma.cyclecount+1, dma.last); 
  }

  uint32_t ppssel;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_PPSSEL,4,(uint8_t*) &ppssel); 
  fprintf(stream, "    PPSSEL:  (0x%x): 0x%x\n", RAD_REG_PPSSEL, ppssel); 


  uint32_t evctrl;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_EV_CTRL,4,(uint8_t*) &evctrl); 
  fprintf(stream, "    EVCNTRL:  (0x%x): 0x%x\n", RAD_REG_EV_CTRL, evctrl); 

  uint32_t pps;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_EV_PPS,4,(uint8_t*) &pps); 
  fprintf(stream, "    PPSCNT:  (0x%x): 0x%x\n", RAD_REG_EV_PPS, pps); 

 
  uint32_t sysclkcnt;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_EV_SYSCLK,4,(uint8_t*) &sysclkcnt); 
  fprintf(stream, "    SYSCLKCNT:  (0x%x): 0x%x\n", RAD_REG_EV_SYSCLK, sysclkcnt); 

  uint32_t lastclkcnt;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_EV_LASTSYSCLK,4,(uint8_t*) &lastclkcnt); 
  fprintf(stream, "    LASTCLKCNT:  (0x%x): 0x%x\n", RAD_REG_EV_LASTSYSCLK, lastclkcnt); 

  uint32_t oeb;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_THRESH_OEB,4,(uint8_t*) &oeb); 
  fprintf(stream, "    TRIGTHRESHOEB:  (0x%x): 0x%x\n", RAD_REG_TRIG_THRESH_OEB, oeb); 

  uint32_t ovldcfg;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_OVLDCFG,4,(uint8_t*) &ovldcfg); 
  fprintf(stream, "    OVLDCFG:  (0x%x): 0x%x\n", RAD_REG_TRIG_OVLDCFG, ovldcfg); 

  uint32_t ovldctrl;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_OVLDCTRL,4,(uint8_t*) &ovldctrl); 
  fprintf(stream, "    OVLDCTRLSTAT:  (0x%x): 0x%x\n", RAD_REG_TRIG_OVLDCTRL, ovldctrl); 

  uint32_t ovldchcfg;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_OVLDCHCFG,4,(uint8_t*) &ovldchcfg); 
  fprintf(stream, "    OVLDCHCFG:  (0x%x): 0x%x\n", RAD_REG_TRIG_OVLDCHCFG, ovldchcfg); 

  uint32_t mastern;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_MASTEREN,4,(uint8_t*) &mastern); 
  fprintf(stream, "    TRIGMASTEREN:  (0x%x): 0x%x\n", RAD_REG_TRIG_MASTEREN, mastern); 

  uint32_t triginen;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_TRIGINEN,4,(uint8_t*) &triginen); 
  fprintf(stream, "    TRIGINEN:  (0x%x): 0x%x\n", RAD_REG_TRIG_TRIGINEN, triginen); 

  uint32_t trigen0;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_TRIGEN0,4,(uint8_t*) &trigen0); 
  fprintf(stream, "    TRIGEN0:  (0x%x): 0x%x\n", RAD_REG_TRIG_TRIGEN0, trigen0); 

  uint32_t trigmaskb0;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_TRIGMASKB0,4,(uint8_t*) &trigmaskb0); 
  fprintf(stream, "    TRIGMASKB0:  (0x%x): 0x%x\n", RAD_REG_TRIG_TRIGMASKB0, trigmaskb0); 

  uint32_t trigwin0;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_TRIGWINDOW0,4,(uint8_t*) &trigwin0); 
  fprintf(stream, "    TRIGWIN0:  (0x%x): 0x%x\n", RAD_REG_TRIG_TRIGWINDOW0, trigwin0); 

  uint32_t trigthresh0;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_TRIGTHRESH0,4,(uint8_t*) &trigthresh0); 
  fprintf(stream, "    TRIGTHRESH0:  (0x%x): 0x%x\n", RAD_REG_TRIG_TRIGTHRESH0, trigthresh0); 

  uint32_t trigen1;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_TRIGEN1,4,(uint8_t*) &trigen1); 
  fprintf(stream, "    TRIGEN1:  (0x%x): 0x%x\n", RAD_REG_TRIG_TRIGEN1, trigen1); 

  uint32_t trigmaskb1;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_TRIGMASKB1,4,(uint8_t*) &trigmaskb1); 
  fprintf(stream, "    TRIGMASKB1:  (0x%x): 0x%x\n", RAD_REG_TRIG_TRIGMASKB1, trigmaskb1); 

  uint32_t trigwin1;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_TRIGWINDOW1,4,(uint8_t*) &trigwin1); 
  fprintf(stream, "    TRIGWIN1:  (0x%x): 0x%x\n", RAD_REG_TRIG_TRIGWINDOW1, trigwin1); 

  uint32_t trigthresh1;
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_TRIG_TRIGTHRESH1,4,(uint8_t*) &trigthresh1); 
  fprintf(stream, "    TRIGTHRESH1:  (0x%x): 0x%x\n", RAD_REG_TRIG_TRIGTHRESH1, trigthresh1); 


  uint32_t scalperiod; 
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_SCALER_PERIOD,4,(uint8_t*) &scalperiod); 
  fprintf(stream, "    SCALPERIOD:  (0x%x): 0x%x\n", RAD_REG_SCALER_PERIOD, scalperiod); 



  return 0;
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

  if (dev->interrupt_fd > 0) 
  {
    flock(dev->interrupt_fd,LOCK_UN); 
    close (dev->interrupt_fd); 
  }

  if (dev->cleanup_spi_gpio) 
  {
    char buf[128]; 
    sprintf(buf,"echo in > /sys/class/gpio/gpio%d/direction", dev->cleanup_spi_gpio); 
    system(buf); 
  }

  free(dev); 
}


//Convenience method useful for cobs decoding 
static int read_until_zero(radiant_dev_t * bd, double timeout) 
{

  if (!bd) return -1; 
  int nread = 0; 

  struct timespec start; 

  if (timeout > 0) 
  {
    clock_gettime(CLOCK_MONOTONIC,&start); 
  }

  struct pollfd pfd = {.fd = bd->uart_fd, .events = POLLIN}; 

  poll(&pfd, 1, timeout > 0 ?  ceil(timeout*1000) : -1); 

  while (1) 
  {
    //read one byte at a time 
    int this_read =  read(bd->uart_fd, bd->reg_buf_encoded + nread, 1); 
    if (!this_read) 
    {
      if (timeout > 0) 
      {
        struct timespec now; 
        clock_gettime(CLOCK_MONOTONIC,&now); 
        double elapsed = now.tv_sec - start.tv_sec + 1e-9 * (now.tv_nsec - start.tv_nsec); 

        if (elapsed > timeout) 
        {
          return -1-nread; 
        }
        poll(&pfd, 1, ceil((timeout-elapsed)*1000)); 
      }
      else
      {
        poll(&pfd, 1, -1); 
      }


      continue; 
    }
    //did we get a zero? if so we did it! 
    if (!bd->reg_buf_encoded[nread]) 
    {
      return nread+1; 
    }
    else
    {
      nread++; 
    }
  }

  return -1; 
}





static int radiant_real_set_mem(radiant_dev_t * bd, radiant_dest_t dest, uint32_t addr, uint8_t len, const uint8_t * bytes, int acked) 
{

  if (!bd) return -1; 

#ifdef RADIANT_SET_DBG
  printf("DBG:Writing %s  to 0x%x on %s:", acked ? "acked" : "unacked" , addr, dest == DEST_MANAGER ? "RDBM" : "RDNT"); 
  for (int i = 0; i < len; i++) printf(" 0x%x", bytes[i]); 
  printf("\n"); 
#endif

  // set up unencoded packet 
  //                //address[21:16]          // dest           //write
  bd->reg_buf[0] = ((addr >> 16 ) & 0x1f) | ( (dest &1) <<6 ) | (1 << 7); 
  bd->reg_buf[1] =  (addr >> 8) & 0xff; //addr [15:8]
  bd->reg_buf[2] = (addr) & 0xff; 

  //copy the data in 
  memcpy(bd->reg_buf+3, bytes, len); 

  int encoded_len = cobs_encode_buf(len+3, bd->reg_buf, sizeof(bd->reg_buf_encoded), bd->reg_buf_encoded); 
  
  //now write to serial 
  int written = write(bd->uart_fd, bd->reg_buf_encoded,encoded_len); 

  if (written != encoded_len) 
  {
    fprintf(stderr,"Wrote only %d bytes instead of %d in radiant_set_mem\n", written, encoded_len); 
    return -1; 
  }

  if (acked) 
  {
    uint8_t expected[3] = { bd->reg_buf[0], bd->reg_buf[1], bd->reg_buf[2]}; 

    //read until we get a 0
    int rd = read_until_zero(bd, bd->read_timeout ); 
    if (rd < 0) 
    {
      fprintf(stderr,"Timed out waiting for ack radiant_set_mem: %d\n", rd); 

    }

    int decoded_len = cobs_decode_buf(rd, bd->reg_buf_encoded, sizeof(bd->reg_buf), bd->reg_buf); 
    int expected_len = sizeof(expected); 

    if (memcmp(expected, bd->reg_buf, expected_len))
    {
      int i = 0;
      fprintf(stderr, "expected buf mismatch in radiant_set mem.\n"); 
      fprintf(stderr, "  expected: "); 
      for (i = 0; i < expected_len; i++) fprintf(stderr," %02x ", expected[i]);
      fprintf(stderr, "\n  got    : "); 
      for (i = 0; i < decoded_len; i++) fprintf(stderr," %02x ", bd->reg_buf[i]);
      return -decoded_len; 
    }
  }

  return len; 
}

int radiant_set_mem(radiant_dev_t * bd, radiant_dest_t dest, uint32_t addr, uint8_t len, const uint8_t * bytes) 
{
  return radiant_real_set_mem(bd,dest,addr,len,bytes,1);

}

int radiant_set_mem_unacked(radiant_dev_t * bd, radiant_dest_t dest, uint32_t addr, uint8_t len, const uint8_t * bytes) 
{
  return radiant_real_set_mem(bd,dest,addr,len,bytes,0);
}



int radiant_get_mem(radiant_dev_t * bd, radiant_dest_t dest, uint32_t addr, uint8_t len, uint8_t * bytes) 
{
  if (!bd) return -1; 

  // set up unencoded packet 
  //                //address[21:16]          // dest          
  bd->reg_buf[0] = ((addr >> 16 ) & 0x1f) | ( (dest &1) <<6 ) ; 
  bd->reg_buf[1] =  (addr >> 8) & 0xff; //addr [15:8]
  bd->reg_buf[2] = (addr) & 0xff; 
  bd->reg_buf[3] = len-1; 

  int encoded_len = cobs_encode_buf(4, bd->reg_buf, sizeof(bd->reg_buf_encoded), bd->reg_buf_encoded); 
  
  //now write to serial 
  int written = write(bd->uart_fd, bd->reg_buf_encoded,encoded_len); 

  if (written != encoded_len) 
  {
    fprintf(stderr,"Wrote only %d bytes instead of %d in radiant_get_mem(0x%p, %s,0x%x, %u,0x%p)\n", written, encoded_len, 
              bd, dest == DEST_MANAGER ? "DEST_MANAGER" : "DEST_FPGA",addr,len,bytes); 
    return -1; 
  }


  uint8_t expected[3] = { bd->reg_buf[0], bd->reg_buf[1], bd->reg_buf[2]}; 
  int expected_len = sizeof(expected); 

  //read until we get a 0
  int rd = read_until_zero(bd, bd->read_timeout); 
  if (rd < 0) 
  {
    fprintf(stderr,"Timed out in radiant_get_mem(0x%p, %s, 0x%x,%u, 0x%p)\n", 
              bd, dest == DEST_MANAGER ? "DEST_MANAGER" : "DEST_FPGA",addr,len,bytes); 
    return -1; 
  }
  //decode 
  int decoded_len = cobs_decode_buf(rd, bd->reg_buf_encoded, sizeof(bd->reg_buf), bd->reg_buf); 

  if (decoded_len != len+3) 
  {
    fprintf(stderr, "Length mismatch. Got %d bytes, expected %d\n", decoded_len, len+3); 
    return -1; 
  }


  if (memcmp(expected, bd->reg_buf, expected_len))
  {
    int i = 0;
    fprintf(stderr, "expected buf mismatch in radiant_get mem.\n"); 
    fprintf(stderr, "  expected: "); 
    for (i = 0; i < expected_len; i++) fprintf(stderr," %02x ", expected[i]);
    fprintf(stderr, "\n  got    : "); 
    for (i = 0; i < decoded_len; i++) fprintf(stderr," %02x ", bd->reg_buf[i]);
    return -decoded_len; 
  }

  //fill in the bytes 

  for (int i = 0; i < len; i++) 
  {
    bytes[i] = bd->reg_buf[i+3]; 
  }

  return len; 
}

int radiant_get_status(radiant_dev_t * bd, uint8_t *status)
{
  if (!status || !bd) return -1; 
  int status_4bytes; 
  int ret = radiant_get_mem(bd, DEST_MANAGER, BM_REG_STATUS, 4, (uint8_t*) &status_4bytes); 
  if (ret==4)  
  {
    *status = status_4bytes & 0xff; 
    return 0; 
  }
  return -1; 
}

int radiant_bm_set_ctrl(radiant_dev_t * bd, uint8_t ctrl) 
{
  if (!bd) return -1; 
  int ctrl_4bytes = ctrl; 
  int nb = radiant_set_mem(bd, DEST_MANAGER, BM_REG_CONTROL, 4,(uint8_t*)  &ctrl_4bytes); 
  return nb ==4 ? 0 : -1; 
}

int radiant_bm_get_ctrl(radiant_dev_t * bd, uint8_t * ctrl) 
{
  if (!bd || !ctrl) return -1; 
  unsigned ctrl_4bytes ; 
  int nb = radiant_get_mem(bd, DEST_MANAGER, BM_REG_CONTROL, 4, (uint8_t*) &ctrl_4bytes); 
  if (nb == 4 && ctrl_4bytes != ffffffff) 
  {
    *ctrl = ctrl_4bytes & 0xff;
    return  0; 
  }
  return -1; 
}

int radiant_bm_analog_read(radiant_dev_t * bd, radiant_bm_analog_rd_t what, float *v, uint16_t *raw) 
{
  if (!bd || (!v && !raw)) return -1; 
  uint32_t addr; 
  int val; 
  switch(what) 
  {
    case RADIANT_BM_ANALOG_V10: 
      addr = BM_REG_ANAV10; break;
    case RADIANT_BM_ANALOG_V18: 
      addr = BM_REG_ANAV18; break;
    case RADIANT_BM_ANALOG_V25: 
      addr = BM_REG_ANAV25; break;
    case RADIANT_BM_ANALOG_LEFTMON: 
      addr = BM_REG_ANALEFT; break;
    case RADIANT_BM_ANALOG_RIGHTMON: 
      addr = BM_REG_ANARIGHT; break;
    default: 
      return -1; 
  }

  int ret = radiant_get_mem(bd, DEST_MANAGER, addr, sizeof(val), (uint8_t*) &val); 
  if (ret == 4) 
  {
    if (v) 
      *v = 3.3 *val/65535.; 
    if (raw) 
      *raw = (uint16_t) val; 
    return 0;
  }

  return -1; 
}

int radiant_bm_analog_read_all(radiant_dev_t * dev, rno_g_radiant_voltages_t *v)
{
  radiant_bm_analog_read(dev, RADIANT_BM_ANALOG_V10, 0, &v->V_1_0);
  radiant_bm_analog_read(dev, RADIANT_BM_ANALOG_V18, 0, &v->V_1_8);
  radiant_bm_analog_read(dev, RADIANT_BM_ANALOG_V25, 0, &v->V_2_5);
  radiant_bm_analog_read(dev, RADIANT_BM_ANALOG_LEFTMON, 0, &v->V_LeftMon);
  radiant_bm_analog_read(dev, RADIANT_BM_ANALOG_RIGHTMON, 0, &v->V_RightMon);
  return 0; 
}


int radiant_enable_cal_mode(radiant_dev_t * bd, int quad) 
{
  if (!bd) return -1; 
  if (quad < 0 || quad > 5) return -1; 


  if (bd->paranoid_about_gpios) read_bm_gpios(bd); 

  //check if we are already enabled 
  if ( bd->gpio_status[quad] & QGPIO_BIT_SEL_CAL) 
  {
    //we are already enabled! do nothing and say we did. 
    return 0; 
  }

  //check if we need to disable another quad in our half

  int other_start = quad < 3 ? 0 : 3; 

  for (int other = other_start; other < other_start +3; other++) 
  {
    if (bd->gpio_status[other] & QGPIO_BIT_SEL_CAL) 
    {
      if (radiant_disable_cal_mode(bd, other))
      {
        fprintf(stderr,"FAIL. Aborting.\n"); 
        return -1; 
      }
    }
  }


  //set the bit 
  bd->gpio_status[quad] |= QGPIO_BIT_SEL_CAL; 

  return write_bm_gpio(bd, quad); 
}

int radiant_disable_cal_mode(radiant_dev_t * bd, int quad) 
{

  if (!bd) return -1; 
  if (quad < 0 || quad > 5) return -1; 

  if (bd->paranoid_about_gpios) read_bm_gpios(bd); 

  // check if already disabled
  if (!(bd->gpio_status[quad] & QGPIO_BIT_SEL_CAL)) 
  {
    //we are already disabled! do nothing and say we did. 
    return 0; 
  }

  //clear the bit
  bd->gpio_status[quad] &= ~QGPIO_BIT_SEL_CAL; 
  return write_bm_gpio(bd, quad); 

}


int radiant_configure_cal(radiant_dev_t *bd, const radiant_cal_config_t * cfg) 
{
  if (!bd) return -1; 

  // clear the bits we might change
  uint8_t cal_data  = bd->gpio_status[BM_REG_SIGPIO_IDX] & ~( SGPIO_BIT_CALPULSE | SGPIO_BIT_N_CALPULSE);  

  if (cfg->pulse_type == RADIANT_CAL_PULSE)
  {
    cal_data |= SGPIO_BIT_CALPULSE; 
  }
  else
  {
    cal_data |= SGPIO_BIT_N_CALPULSE;  
    cal_data &= ~(SGPIO_BIT_CAL_FIL0 | SGPIO_BIT_N_CAL_FIL1 | SGPIO_BIT_CAL_FIL1);
    switch (cfg->band) 
    {
      case  RADIANT_CAL_100_300: 
        cal_data |= SGPIO_BIT_CAL_FIL0 | SGPIO_BIT_N_CAL_FIL1; 
        break;
      case  RADIANT_CAL_50_100:
        cal_data |= SGPIO_BIT_N_CAL_FIL1; 
        break; 
      case RADIANT_CAL_600_PLUS: 
        cal_data |= SGPIO_BIT_CAL_FIL0 | SGPIO_BIT_CAL_FIL1; 
        break;
      case  RADIANT_CAL_300_600:
        cal_data |= SGPIO_BIT_CAL_FIL1; 
        break; 
      default:
        fprintf(stderr,"UNKNOWN CALBAND: %d\n", cfg->band); 
        return 1; 
    }
  }

  bd->gpio_status[BM_REG_SIGPIO_IDX] = cal_data; 
  int write_ok =  write_bm_gpio(bd, BM_REG_SIGPIO_IDX); 

  if (write_ok) return write_ok; 
  uint8_t bytes[4]; 

  int pulse_period = cfg->pulse_settings.pulse_period_ns / 5; 
  if (pulse_period <=0) pulse_period = 200000; //use 1 Khz if 0
  bytes[0] =pulse_period & 0xff;
  bytes[1] = (pulse_period >> 8 ) & 0xff; 
  bytes[2] = (pulse_period >> 16 ) & 0xff; 
  bytes[3] = (pulse_period >> 24 ) & 0x3f; 
  bytes[3] |= cfg->pulse_settings.pulse_sharpen << 6; 
  bytes[3] |= cfg->pulse_settings.pulse_disable << 7; 

  return radiant_set_mem(bd, DEST_FPGA, RAD_REG_TRIG_PULSECTRL, sizeof(bytes),bytes); 

}

int radiant_enable_cal(radiant_dev_t * bd, int enable) 
{
  if (!bd) return -1; 

  //clear the bits we might change
  uint8_t cal_data  = bd->gpio_status[BM_REG_SIGPIO_IDX] & ~(SGPIO_BIT_SG_ENABLE);  
  if (enable) 
  {
    cal_data |= SGPIO_BIT_SG_ENABLE ;
  }
  bd->gpio_status[BM_REG_SIGPIO_IDX] = cal_data; 
  return write_bm_gpio(bd, 6); 
}

int radiant_cal_enabled(radiant_dev_t *bd, int *enabled) 
{
  *enabled = !! ( bd->gpio_status[BM_REG_SIGPIO_IDX] & SGPIO_BIT_SG_ENABLE); 
  return 0; 
}

static adf4350_dev adf4351; 
static int already_setup_adf4351 = 0; 
static adf4350_init_param adf4351_param =
{ 
    
    //device settings 
    .clkin = 10000000,  //10 MHz
    .channel_spacing = 1000,  // 1 kHz????
    .power_up_frequency = 150000000, //150 MHz
    .reference_div_factor=  0, // let it decide
    .reference_doubler_enable = 0, 
    .reference_div2_enable = 0, 
    //r2
    .phase_detector_polarity_positive_enable = 1, 
    .lock_detect_precision_6ns_enable = 0, 
    .lock_detect_function_integer_n_enable = 0, //this will be set automagically 
    .charge_pump_current = 2500, //??!
    .muxout_select = 0, 
    .low_spur_mode_enable = 0, 
    //r3
    .cycle_slip_reduction_enable = 0, 
    .charge_cancellation_enable = 0,
    .anti_backlash_3ns_enable = 0, 
    .band_select_clock_mode_high_enable = 0, 
    .clk_divider_12bit = 150 , //?!?? 
    .clk_divider_mode = 0, 
    //r4
    .aux_output_enable = 0, 
    .aux_output_fundamental_enable = 0, 
    .mute_till_lock_enable =0,
    .output_power  = 3, 
    .aux_output_power = 0,
};



int radiant_set_frequency(radiant_dev_t * bd, float freq_MHz, float * actual_freq_MHz) 
{

  if (!bd) return -1; 
  if (!already_setup_adf4351) 
  {
    adf4350_setup(&adf4351, adf4351_param ); 
    already_setup_adf4351 = 1; 
  }

  int64_t actual = adf4350_set_freq(&adf4351, freq_MHz * 1e6); 
  if (actual_freq_MHz) *actual_freq_MHz = actual/1e6; 

  //now set the registers.... 
  for (int ireg = 5; ireg>=0; ireg--) 
  {
    uint32_t reg = adf4351.regs[ireg] | ireg; 

    //this is probably the same... since we're little endian? 
    uint8_t bytes[4] = { reg & 0xff, (reg >> 8) & 0xff, (reg >> 16) & 0xff, (reg > 24) & 0xff}; 
//    printf("reg%d = %u\n",ireg, reg);  // | ireg is important... to actualy write! 

    bd->gpio_status[BM_REG_SIGPIO_IDX] &= ~SGPIO_BIT_SIG_LE; //clear the latch
    write_bm_gpio(bd,BM_REG_SIGPIO_IDX); 
    if (sizeof(bytes) != radiant_set_mem(bd,DEST_MANAGER, BM_REG_SPIOUT_MSB, sizeof(bytes), bytes))  
    {
      return 1; 
    }
    //set the latch
    bd->gpio_status[BM_REG_SIGPIO_IDX] |= SGPIO_BIT_SIG_LE; //set the latch
    write_bm_gpio(bd,BM_REG_SIGPIO_IDX); 
    bd->gpio_status[BM_REG_SIGPIO_IDX] &= ~SGPIO_BIT_SIG_LE; //clear the latch
    write_bm_gpio(bd,BM_REG_SIGPIO_IDX); 
  }

  return 0; 
}


int radiant_set_attenuator(radiant_dev_t * bd, int channel, radiant_atten_t which, uint8_t value) 
{
  if (!bd) return -1; 
  //figure out which quad 
  int quad = channel / 4; 
  int ch = channel % 4; 

  //clamp to max atten 
  if (value > 127) value = 127; 

  uint8_t addr = (which == RADIANT_ATTEN_TRIG) + 2* ch; 
  uint8_t bytes[4] = {value, addr,0,0}; // littlendian order
  //clear LE 
  bd->gpio_status[quad] &= ~QGPIO_BIT_ATT_LE; 
  write_bm_gpio(bd, quad); 
  radiant_set_mem(bd, DEST_MANAGER, BM_REG_SPIOUT_LSB, sizeof(bytes), bytes); 
  bd->gpio_status[quad] |= QGPIO_BIT_ATT_LE; //set LE
  write_bm_gpio(bd, quad); 
  bd->gpio_status[quad] &= ~QGPIO_BIT_ATT_LE;  //clear LE 
  write_bm_gpio(bd, quad); 

  return 0; 
}


int radiant_fill_dma_config(radiant_dma_config_t * cfg, radiant_dma_mode_preset_t preset) 
{
  if (!cfg) return -1; 
  //zero out
  memset(cfg,0,sizeof(radiant_dma_config_t)); 

  //We are always using 32-bit words right now, so always want the endianness swwap 
  cfg->endianness = 1; 

  switch(preset)
  {
    case RADIANT_DMA_EVENT_MODE: 
      cfg->ext_dma_req_enable = 1; 
      __attribute__((fallthrough)); 
    case RADIANT_DMA_CAL_MODE: 
      cfg->tx_full_flag_enable = 1;
      cfg->tx_full_flag_threshold=512; 
      cfg->dma_enable =1 ;
      return 0;
    case RADIANT_DMA_LAB4D_MODE:
      cfg->cycle_delay=12; 
      __attribute__((fallthrough)); 
    case RADIANT_DMA_CAL_WRITE_MODE: 
      cfg->dma_enable=1; 
      cfg->dma_direction=1; 
      cfg->enable_spi_receive=1; 
      return 0; 
    case RADIANT_DMA_CPLD_MODE:
      cfg->dma_enable=1; 
      cfg->dma_direction=1; 
      cfg->byte_mode=1; 
      cfg->enable_spi_receive=1; 
      cfg->cycle_delay=2; 
      return 0; 
    default: 
      return -1; 
  }
}

int radiant_get_dma_config(radiant_dev_t *bd, radiant_dma_config_t *cfg) 
{
  return 4!=radiant_get_mem(bd, DEST_FPGA, RAD_REG_SPIDMA_CONFIG, 4 , (uint8_t*) cfg); 
}

int radiant_configure_dma(radiant_dev_t *bd, const radiant_dma_config_t *cfg) 
{
  return 4!=radiant_set_mem(bd, DEST_FPGA, RAD_REG_SPIDMA_CONFIG, 4 , (uint8_t*) cfg); 
}

int radiant_dma_control(radiant_dev_t *bd, const radiant_dma_ctrl_t ctrl) 
{
  if (!bd) return -1; 
  uint8_t bytes[4] = {*((uint8_t*) &ctrl),0,0,0}; 
  return 4!=radiant_set_mem(bd, DEST_FPGA, RAD_REG_SPIDMA_CONTROL, 4 , bytes); 
}

int radiant_dma_engine_reset(radiant_dev_t *bd) 
{
  radiant_dma_ctrl_t engine_reset = {.engine_reset=1}; 
  return radiant_dma_control(bd, engine_reset); 
}

int radiant_dma_tx_reset(radiant_dev_t *bd) 
{
  radiant_dma_ctrl_t tx_reset = {.tx_reset=1}; 
  return radiant_dma_control(bd, tx_reset); 
}

int radiant_dma_rx_reset(radiant_dev_t *bd) 
{
  radiant_dma_ctrl_t rx_reset = {.rx_reset=1}; 
  return radiant_dma_control(bd, rx_reset); 
}

int radiant_dma_request(radiant_dev_t *bd) 
{
  radiant_dma_ctrl_t req = {.dma_req=1}; 
  return radiant_dma_control(bd, req); 
}

int radiant_dma_txn_count_reset(radiant_dev_t *bd) 
{
  if (!bd) return -1; 
  uint8_t zeros[4] = {0}; 
  return 4!= radiant_set_mem(bd, DEST_FPGA, RAD_REG_SPIDMA_TXN_COUNT, 4,zeros); 
}

int radiant_dma_txn_count(radiant_dev_t *bd, uint32_t* count) 
{
  if (!bd) return -1; 
  return 4!= radiant_get_mem(bd, DEST_FPGA, RAD_REG_SPIDMA_TXN_COUNT,4, (uint8_t*) count); 
}



int radiant_dma_set_descriptor(radiant_dev_t* bd, uint8_t idescr, radiant_dma_desc_t descr)
{
  if (!bd) return -1; 

  if (idescr >= 32) return -1; 
  if (descr.addr & 0x3 ) return -2; 
  if (descr.length > 4096) return -3; 
  if (descr.addr > ( 1 << 20)) return -4; 

  struct fpga_dma_descr mem; 
  static_assert(sizeof(mem) == 4, "dma descr struct is wrong size"); 
  mem.address = descr.addr >> 2; 
  mem.increment = !!(descr.incr); 
  mem.cyclecount = descr.length-1; 
  mem.last = !!(descr.last); 

  int ret = radiant_set_mem(bd, DEST_FPGA, RAD_REG_SPIDMA_DESCR_BASE + idescr*RAD_REG_SPIDMA_DESCR_INCR, 4 , (uint8_t*) &mem); 
//  printf("%d,%x: %x %u %u %u=%d\n", idescr, RAD_REG_SPIDMA_DESCR_BASE + idescr*RAD_REG_SPIDMA_DESCR_INCR,  mem.address<<2, mem.increment, mem.cyclecount+1, mem.last,ret); 
  return 4!=ret; 
}


int radiant_dma_setup_event(radiant_dev_t*bd, uint32_t mask) 
{
  if (!bd) return -1; 
  mask &= 0xffffff; // 24 bits 
  if (!mask) return -1; 

  bd->readout_mask = mask; 
  
  radiant_dma_txn_count_reset(bd); 
  radiant_dma_engine_reset(bd); 
  radiant_dma_tx_reset(bd); 
  radiant_dma_rx_reset(bd); 
  

  int idesc = 0; 
  //set the first one for the event fifo. this looks like the right place 
  radiant_dma_desc_t desc = {.addr=RAD_REG_EV_FIFO_BASE, .incr=1, .length=sizeof(struct radiant_fw_event_header)/sizeof(uint32_t)}; 
  radiant_dma_set_descriptor(bd, idesc++, desc); 
  int last = 31 - __builtin_clz(mask); 


  for (int i = 0; i < 24; i++) 
  {
    if (mask & (1 << i)) 
    {
      radiant_dma_desc_t this_desc = {.addr=RAD_REG_LAB_RAM_BASE+i*RAD_REG_LAB_RAM_INCR, .incr=0, .length=bd->readout_nsamp/2}; 
      desc.addr = RAD_REG_LAB_RAM_BASE  + RAD_REG_LAB_RAM_INCR*i; 
      if (i == last) this_desc.last = 1; 
      radiant_dma_set_descriptor(bd, idesc++, this_desc); 
    }
  }

  radiant_dma_config_t dma_cfg; 
  radiant_fill_dma_config(&dma_cfg, RADIANT_DMA_EVENT_MODE); 
  radiant_configure_dma(bd, &dma_cfg); //this should probably already have been done but whatever... 
  return 0; 
}



int radiant_internal_trigger(radiant_dev_t * bd, int howmany, int block) 
{

  if (!bd) return -1; 
  if (!howmany) return -1; 
  if (howmany > 256 || howmany < 0) howmany = 256; 

  // inferred from python code. TODO: learn more
  uint32_t mem = 2 | ((howmany-1) << 8); 

  int ret =  4!=radiant_set_mem(bd, DEST_FPGA, RAD_REG_LAB_CTRL_TRIGGER, 4,(uint8_t*)  &mem); 

  if (block) 
  {
    int busy = 1; 
    while (busy) 
    {
      radiant_get_mem(bd, DEST_FPGA, RAD_REG_LAB_CTRL_READOUT, 4, (uint8_t *) &mem); 
//      printf("DBG: blocking for force trigger, RAD_REG_LAB_CTRL_READOUT = 0x%x\n", mem); 
      busy = (mem & 0x21); 
    }
  }

  return ret; 
}


int radiant_soft_trigger(radiant_dev_t *bd) 
{
  if (!bd) return -1; 

  //if version is pre 0.2.27, then use internal trigger
  if (bd->rad_dateversion_int <= 227) return radiant_internal_trigger(bd,1,0); 

  uint32_t mem = 1; 
  return 4!=radiant_set_mem(bd, DEST_FPGA, RAD_REG_TRIG_OVLDCTRL, 4, (uint8_t*) &mem); 
}

int radiant_trigger_busy(radiant_dev_t *bd, int *bsy, int * pending) 
{
  if (!bd) return -1; 

  if (bd->rad_dateversion_int <= 227) return -1; 

  uint32_t mem = 0; 
  if (4!=radiant_get_mem(bd, DEST_FPGA, RAD_REG_TRIG_OVLDCTRL, 4, (uint8_t*) &mem)) return 1; 
  if (bsy) *bsy = mem & 1; 
  if (pending) *pending = (mem >> 1) & 1; 
  return 0; 
}


int radiant_cpu_clear(radiant_dev_t *bd) 
{
  if (!bd || bd->rad_dateversion_int <=227) return -1; 

  uint32_t mem = 2; 
  return 4!=radiant_set_mem(bd, DEST_FPGA, RAD_REG_TRIG_OVLDCTRL, 4, (uint8_t*) &mem); 
}

int radiant_cpu_clear_pending(radiant_dev_t *bd, int *pending) 
{
  if (!bd) return -1; 

  if (bd->rad_dateversion_int <= 227) return -1; 

  uint32_t mem = 0;  
  if (4!=radiant_get_mem(bd, DEST_FPGA, RAD_REG_TRIG_OVLDCTRL, 4, (uint8_t*) &mem)) return 1; 

  *pending = mem >> 1; 
  return 0;
}




const uint32_t enable_mask = RADIANT_TRIG_EN | RADIANT_TRIG_EXT | RADIANT_TRIG_PPS | RADIANT_TRIGOUT_EN | RADIANT_TRIGOUT_SOFT | RADIANT_TRIGOUT_PPS | RADIANT_TRIG_CPUFLOW; 

int radiant_trigger_enable(radiant_dev_t *bd, int enables, uint32_t disable_mask) 
{
  if (!bd || bd->rad_dateversion_int <=227) return -1; 

  uint32_t mem = 0; 


  if (enables == RADIANT_TRIG_DISMASK_QUERY) 
  {
    if (4!=radiant_get_mem(bd, DEST_FPGA, RAD_REG_TRIG_OVLDCHCFG, 4, (uint8_t*) &mem)) 
    {
      return RADIANT_TRIG_DISMASK_QUERY; 
    }

    return (int) mem; 
  }

  //set the new disable mask 
  if (4!=radiant_set_mem(bd, DEST_FPGA, RAD_REG_TRIG_OVLDCHCFG, 4, (uint8_t*) &disable_mask)) 
  {
      return 1; 
  }

  //Get the enables 
  if (4!=radiant_get_mem(bd, DEST_FPGA, RAD_REG_TRIG_OVLDCFG, 4, (uint8_t*) &mem)) 
  {
    return  enables == RADIANT_TRIG_QUERY ? RADIANT_TRIG_QUERY : 1; 
  }

  if (enables == RADIANT_TRIG_QUERY) 
  {
    return mem & enable_mask; 
  }

  //Clear the enable mask 
  mem &=~enable_mask; 

  //set the new mask 
  mem |= (enables & enable_mask); 

  //set the number of buffers

  const uint32_t buffer_bits = 0x3 << 17; 
  mem&=~buffer_bits; 
  mem |= (bd->nbuffers_per_readout-1) << 17; 

  return  4!=radiant_set_mem(bd, DEST_FPGA, RAD_REG_TRIG_OVLDCFG, 4, (uint8_t*) &mem);


}

int radiant_trigout_set_length(radiant_dev_t * bd, int ns) 
{
  if (!bd || bd->rad_dateversion_int <=227) return -1; 
  if (ns < 0 || ns >=320) return -2; 

  uint32_t mem = 0; 
  if (4!=radiant_get_mem(bd, DEST_FPGA, RAD_REG_TRIG_OVLDCFG, 4, (uint8_t*) &mem)) return 1; 
  int cycles = ns/10; 
  const uint32_t trigout_len_mask = (0x1f << 24); 
  mem &=~trigout_len_mask; 
  mem |= cycles << 24; 
  return (4!=radiant_set_mem(bd, DEST_FPGA, RAD_REG_TRIG_OVLDCFG, 4, (uint8_t*) &mem)) ; 
}

int radiant_trigout_get_length(radiant_dev_t * bd, int * ns) 
{
  if (!bd || bd->rad_dateversion_int <=227) return -1; 
  if (!ns) return -1; 

  uint32_t mem = 0; 
  if (4!=radiant_get_mem(bd, DEST_FPGA, RAD_REG_TRIG_OVLDCFG, 4, (uint8_t*) &mem)) return 1; 
  int cycles = (mem >> 24) & 0x1f; 
  *ns= cycles*10; 
  return 0; 
}





int radiant_labs_clear(radiant_dev_t *dev) 
{

  if (!dev) return -1; 
  uint8_t ctrl[4]; 
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_LAB_CTRL_CONTROL,4,ctrl); 

  if (ctrl[3] & 4 ) 
  {
    radiant_labs_stop(dev); 
    radiant_get_mem(dev, DEST_FPGA, RAD_REG_LAB_CTRL_CONTROL,4,ctrl); 
  }
  ctrl[2] |=1; 
  radiant_set_mem(dev, DEST_FPGA, RAD_REG_LAB_CTRL_CONTROL,4,ctrl); 
  ctrl[2] &=~1; 
  radiant_set_mem(dev, DEST_FPGA, RAD_REG_LAB_CTRL_CONTROL,4,ctrl); 
  return 0; 
}

int radiant_labs_start(radiant_dev_t *dev) 
{
  if (!dev) return -1; 

  uint8_t ctrl[4]; 
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_LAB_CTRL_CONTROL,4,ctrl); 
  while (!(ctrl[0] & 4) ) // @???!?? 
  {
//    printf("%x %x %x %x\n", ctrl[0],ctrl[1],ctrl[2],ctrl[3]); 
    ctrl[0] |= 2;  
//    printf("%x %x %x %x\n", ctrl[0],ctrl[1],ctrl[2],ctrl[3]); 
    radiant_set_mem(dev, DEST_FPGA, RAD_REG_LAB_CTRL_CONTROL,4,ctrl); 
    radiant_get_mem(dev, DEST_FPGA, RAD_REG_LAB_CTRL_CONTROL,4,ctrl); 
  }
  return 0; 
}


int radiant_labs_stop(radiant_dev_t *dev) 
{
  if (!dev) return -1; 
  uint8_t ctrl[4]; 
  radiant_get_mem(dev, DEST_FPGA, RAD_REG_LAB_CTRL_CONTROL,4,ctrl); 
  while ((ctrl[0] & 4)) // @???!?? 
  {
    ctrl[0] &= ~2;  
    radiant_set_mem(dev, DEST_FPGA, RAD_REG_LAB_CTRL_CONTROL,4,ctrl); 
    radiant_get_mem(dev, DEST_FPGA, RAD_REG_LAB_CTRL_CONTROL,4,ctrl); 
  }

  //make we sure in single-buffer mode! 
  radiant_set_nbuffers_per_readout(dev,1); 

  return 0; 
}


int radiant_check_avail(radiant_dev_t * bd) 
{
  if (!bd) return -1; 
  uint32_t mem = 0; 
  if (4!= radiant_get_mem( bd, DEST_FPGA, RAD_REG_SPIDMA_CONFIG, 4, (uint8_t*) &mem)) return -1; 
  return  !!(mem & (1<<30)); 
}

//not exposed for now since only used for pedestals right now... 
static int calram_zero(radiant_dev_t * bd) 
{
  if (!bd) return -1; 
  uint32_t ctrl; 
  radiant_get_mem(bd, DEST_FPGA, RAD_REG_LAB_CTRL_CONTROL, sizeof(ctrl), (uint8_t*) &ctrl) ; 
  if (ctrl & 0x4)  // TODO: python code has this as 0x2 but I'm pretty sure that's wrong
  {
    return 1; 
  }

  uint32_t mode = 0x2; 
  ctrl = 0x3; 
  radiant_set_mem(bd, DEST_FPGA, RAD_REG_CALRAM_GLOBAL_CONTROL, 4, (uint8_t*) &ctrl); 
  radiant_set_mem(bd, DEST_FPGA, RAD_REG_CALRAM_GLOBAL_MODE, 4, (uint8_t*) &mode); 


  radiant_labs_start(bd); 
 
  uint8_t oldmode = bd->calram; 
  bd->calram = CALRAM_MODE_ZERO; 
  for (int i = 0; i < 4; i++) radiant_internal_trigger(bd,1,1); 
  bd->calram = oldmode; 
  radiant_labs_stop(bd); 
  return 0; 
}


// may expose this later if necessary
static int calram_mode(radiant_dev_t *bd, calram_mode_t mode) 
{
  if (!bd) return -1; 

  uint32_t ctrl = 0; 
  uint32_t mde = 0; 
  switch(mode) 
  {
    case CALRAM_MODE_NONE: 
      break; 
    case CALRAM_MODE_PED:
      ctrl=2; 
      break; 
    default: 
        return -1; 
  }

   if (4!=radiant_set_mem(bd,DEST_FPGA,RAD_REG_CALRAM_GLOBAL_CONTROL, 4, (uint8_t*) &ctrl)) return 1; 
   if (4!=radiant_set_mem(bd,DEST_FPGA,RAD_REG_CALRAM_GLOBAL_MODE, 4, (uint8_t*) &mde)) return 1; 

   if (mode == CALRAM_MODE_PED) 
   {
     ctrl=1; 
     if (4!=radiant_set_mem(bd,DEST_FPGA,RAD_REG_CALRAM_GLOBAL_CONTROL, 4, (uint8_t*) &ctrl)) return 1; 
   }

   bd->calram = mode; 
   return 0; 
}


int radiant_compute_pedestals(radiant_dev_t *bd, uint32_t mask, uint16_t ntriggers, rno_g_pedestal_t * ped) 
{
  if (!bd) return -1; 
  if (!mask) return 0; 

  radiant_labs_stop(bd); 


 

  //clear the calram ?
  if (calram_zero(bd)) return -1; 
  if (calram_mode(bd,CALRAM_MODE_PED)) return -2; 

  radiant_labs_start(bd); 

  uint16_t nleft = ntriggers * 4; //* 4 to get each buffer 
  while (nleft > 0) 
  {
    //need 4 triggers / cycle to fill
    int todo = nleft < bd->triggers_per_cycle ? nleft : bd->triggers_per_cycle; 
    radiant_internal_trigger(bd, todo, 1); 
    if (bd->sleep_per_cycle) 
    {
      struct timespec slp = bd->sleep_per_cycle_ts; 
      while (nanosleep(&slp,&slp)); 
    }
    nleft-=todo; 
  }
  radiant_labs_stop(bd); 
  

  radiant_dma_engine_reset(bd); 
  radiant_dma_tx_reset(bd); 

  int ifinal = 31 - __builtin_clz(mask); 
  int itx = 0; 

  radiant_dma_config_t dma; 
  radiant_fill_dma_config(&dma, RADIANT_DMA_CAL_MODE); 
  radiant_configure_dma(bd, &dma); 


  // note that the spidev bufsiz would have to be increased even more to get all of the pedestals in, so we'll read them out one at a time after setting up the descriptors
  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
  {
    if (! (mask & (1 <<i))) continue; 
    radiant_dma_desc_t desc = {.addr = RAD_REG_CALRAM_BASE + sizeof(uint32_t) *RNO_G_PEDESTAL_NSAMPLES * i, .length = RNO_G_PEDESTAL_NSAMPLES, .incr = 1, .last = (i == ifinal) };
    radiant_dma_set_descriptor(bd, itx++, desc); 
  }



  usleep(10000); 
  radiant_dma_request(bd); 
  //wait a bit? 
  usleep(10000); 

  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
  {
    if (! (mask & (1 << i))) 
    {
      memset(ped->pedestals[i], 0, sizeof(ped->pedestals[i])); 
      continue; 
    }
    else
    {
      uint16_t N = RNO_G_PEDESTAL_NSAMPLES * sizeof(uint32_t); 
      uint8_t * memptr[1] = { (uint8_t*) bd->pedram} ; 
      int nrd = radiant_read(bd, 1, &N, memptr); 
      if (nrd != N)
      {
        fprintf(stderr, "Bad news bears: %u %d\n", nrd, N); 
      }
      else
      {
        for (unsigned j = 0; j < RNO_G_PEDESTAL_NSAMPLES; j++) 
        {
          ped->pedestals[i][j] = bd->pedram[j] / ntriggers; 
        }
      }
    }
  }
  ped->vbias[0] = bd->vbias[0];
  ped->vbias[1] = bd->vbias[1];
  ped->nevents = ntriggers; 
  ped->when = time(0); 

  // take out of calram mode? 
  calram_zero(bd); // TODO: IS THIS NEEDED? Dan has this, but why? 
  calram_mode(bd,CALRAM_MODE_NONE); 

  radiant_dma_txn_count_reset(bd); 
  radiant_dma_engine_reset(bd); 
  radiant_dma_tx_reset(bd); 


  return 0; 
}


int radiant_set_dc_bias (radiant_dev_t * bd, uint16_t l16, uint16_t r16) 
{
  if (!bd) return -1; 
  uint32_t l = l16; 
  uint32_t r = r16; 
  // set to reasonable value if out of range
  if (l > 4095) l = 1000; 
  if (r > 4095) r = 1000; 
  bd->vbias[0] = l;
  bd->vbias[1] = r; 
  if (4!=radiant_set_mem(bd, DEST_MANAGER, BM_REG_VPEDLEFT, 4, (uint8_t*) &l)) return 1; 
  return (4!=radiant_set_mem(bd, DEST_MANAGER, BM_REG_VPEDRIGHT, 4, (uint8_t*) &r)); 
}


int radiant_set_td_bias(radiant_dev_t *bd, int ich, uint16_t val) 
{ 
  if (!bd) return -1; 
  if (ich < 0 || ich >= RNO_G_NUM_RADIANT_CHANNELS) return -1; 
  //clamp 
  if (val>4095) val = 4095;

  uint32_t v32 = val; 
  return 4!=radiant_set_mem(bd, DEST_MANAGER, BM_REG_TDBIAS_BASE + ich * BM_REG_TDBIAS_INCR, 4, (uint8_t*) &v32); 
}


void radiant_set_pedestals(radiant_dev_t* bd, const rno_g_pedestal_t * ped) 
{
  if (bd)
  {
    bd->peds = ped; 
  }
}

const rno_g_pedestal_t *  radiant_get_pedestals(radiant_dev_t* bd) 
{
  if (!bd) return 0; 
  return bd->peds; 
}

int radiant_set_nbuffers_per_readout(radiant_dev_t *bd, int nbuffers) 
{
  if (!bd) return -1; 
  if (nbuffers <1 || nbuffers >2 )
  {
    return 0x90991e5; //they do nothing 
  }

  uint32_t mem = nbuffers== 1 ? RADIANT_TRIG_ONEBUFMODE : RADIANT_TRIG_TWOBUFMODE; 
  int  ret = (4!=radiant_set_mem(bd, DEST_FPGA, RAD_REG_LAB_CTRL_TRIGGER, 4, (uint8_t*) &mem)); 

  if (!ret) 
  {
    bd->nbuffers_per_readout = nbuffers; 
    bd->readout_nsamp = nbuffers * RADIANT_NSAMP_PER_BUF; 
  }
  return ret; 
}


/** how is this different from labc_stop/start?  */ 
static int radiant_run_mode(radiant_dev_t * bd, int enable) 
{

  if (!bd) return -1; 
    uint32_t ctrl; 
    if (4!=radiant_get_mem(bd, DEST_FPGA, RAD_REG_LAB_CTRL_CONTROL, 4, (uint8_t*) &ctrl)) return 1; 

    if (enable) ctrl|= 2; 
    else ctrl&=~2; 
    return (4!=radiant_set_mem(bd, DEST_FPGA, RAD_REG_LAB_CTRL_CONTROL, 4, (uint8_t*) &ctrl)); 
}


int radiant_reset_readout_fifo(radiant_dev_t * bd, int force, int reset_readout) 
{
  if (!bd) return -1; 
  if (!force) 
  {
    uint32_t ctrl; 
    if (4!=radiant_get_mem(bd, DEST_FPGA, RAD_REG_LAB_CTRL_CONTROL, 4, (uint8_t*) &ctrl) || ctrl & 2 ) 
    {
      fprintf(stderr,"CANNOT RESET FIFO, LAB4 in run mode or cannot be read\n"); 
      return 1; 
    }
  }

  // Ok, if we didn't force this should be extraneous no? 
  if (reset_readout) 
  {
    radiant_run_mode(bd,0); 
  }

  uint32_t rdout; 
  if (4!=radiant_get_mem(bd, DEST_FPGA, RAD_REG_LAB_CTRL_READOUT, 4, (uint8_t*) &rdout)) return 1; 

  rdout |= 2; 
  if (reset_readout)
  {
    rdout |= 4; 
  }
  else
  {
    rdout&=~4; 
  }
    return (4!=radiant_set_mem(bd, DEST_FPGA, RAD_REG_LAB_CTRL_READOUT, 4, (uint8_t*) &rdout)); 
}


int radiant_reset_fifo_counters(radiant_dev_t * bd) 
{
  if (!bd) return -1; 
  uint32_t evctrl = 4; 
  return (4!=radiant_set_mem(bd, DEST_FPGA, RAD_REG_EV_CTRL, 4, (uint8_t*) &evctrl)); 
}

int radiant_sync(radiant_dev_t * bd) 
{
  if (!bd) return -1; 
  uint32_t evctrl = 2; 
  return (4!=radiant_set_mem(bd, DEST_FPGA, RAD_REG_EV_CTRL, 4, (uint8_t*) &evctrl)); 
}

int radiant_get_pending(radiant_dev_t * bd, radiant_pending_t * pend) 
{
  if (!bd) return -1; 
  if (!pend) return 1; 
  uint32_t evctrl = 0; 

  if (4!=radiant_get_mem(bd, DEST_FPGA, RAD_REG_EV_CTRL, 4, (uint8_t*) &evctrl)) return 1; 

  pend->pending = (evctrl >> 16) & 0x3f; 
  pend->fifo_empty = evctrl & (1 << 15); 
  pend->pending_empty = evctrl & (1 << 14); 
  return 0; 
}

int radiant_current_pps(radiant_dev_t * bd, uint32_t *pps, uint32_t *sysclk_last_pps, uint32_t *sysclk_last_last_pps) 
{
  if (!bd) return -1; 
  //just read all 3 at once, it won't take any longer really  
  uint32_t mem[3]={0,0,0}; 
  if (12!=radiant_get_mem(bd, DEST_FPGA, RAD_REG_EV_PPS, 12, (uint8_t*) mem )) return 1; 

  if (pps) 
  {
    *pps = mem[0]; 
  }

  if (sysclk_last_pps) 
  {
    *sysclk_last_pps = mem[1]; 
  }

  if (sysclk_last_last_pps) 
  {
    *sysclk_last_last_pps = mem[2]; 
  }

  return 0; 
}


int radiant_set_global_trigger_mask(radiant_dev_t *bd, uint32_t mask)
{
  if (!bd) return -1; 

  mask &= 0xffffff; 

  //check that we actually ahve to set it 
  radiant_get_global_trigger_mask(bd,0,1); //this will force update bd->triginen 

  if (mask != bd->triginen) 
  {
    //have to temporarily disable the trigger if it's enabled 
    uint32_t was_enabled = 0; 

    if (radiant_get_l0_enable(bd,&was_enabled)) return 1; 

    if (was_enabled) 
    {
      if (radiant_set_l0_enable(bd,0)) return 1; 
    }

    
    //set the global trigger enables
    if ( 4!= radiant_set_mem(bd, DEST_FPGA, RAD_REG_TRIG_TRIGINEN, 4, (uint8_t*) &mask)) return 1; 
    bd->triginen = mask; 

    if (was_enabled) 
    {
      if (radiant_set_l0_enable(bd,1)) return 1; 
    }
  }
  return 0; 
}

int radiant_get_global_trigger_mask(radiant_dev_t *bd, uint32_t * mask, int force)
{

  if (!bd) return -1; 
  if (force) 
  {
    if ( 4!= radiant_get_mem(bd, DEST_FPGA, RAD_REG_TRIG_TRIGINEN, 4, (uint8_t*) &bd->triginen)) return 1; 
  }

  if (mask) 
  {
    *mask = bd->triginen; 
  }

  return 0; 
}

int radiant_get_l0_enable(radiant_dev_t * bd, uint32_t * en) 
{
  if (!bd) return -1; 
  return (4!= radiant_get_mem(bd, DEST_FPGA, RAD_REG_TRIG_MASTEREN, 4, (uint8_t*) en)); 
}

int radiant_set_l0_enable(radiant_dev_t *bd, uint32_t en) 
{
  if (!bd) return -1; 
  en &=1; 
  return (4!= radiant_set_mem(bd, DEST_FPGA, RAD_REG_TRIG_MASTEREN, 4, (uint8_t*) &en)) ; 
}


int radiant_configure_rf_trigger(radiant_dev_t *bd, radiant_trig_sel_t which, 
                                 uint32_t contributing_channels, 
                                 uint8_t required_coincident_channels, 
                                 float coincidence_window_ns)
{
  if (!bd || bd->rad_dateversion_int <=227) return -1; 
  int update_global_bit = contributing_channels >> 31; 

  //see if  trigger is enabled
  uint32_t trig_enabled = 0; 
  if (radiant_get_l0_enable(bd,&trig_enabled)) return 1; 

  //disable it if it's on
  if (trig_enabled) 
  {
    if (radiant_set_l0_enable(bd,0)) return 1; 
  }

  contributing_channels &=0xffffff; 
  if (!required_coincident_channels) contributing_channels = 0; 

  // enable this trigger or not depending on if there are any channels enabled 
  uint32_t mem = (!!contributing_channels) << 31; 
  
  if ( 4!= radiant_set_mem(bd, DEST_FPGA, 
      which == RADIANT_TRIG_A ? RAD_REG_TRIG_TRIGEN0 : RAD_REG_TRIG_TRIGEN1, 
      4, (uint8_t*) &mem)) return 1; 
  
  //set the mask 
  if ( 4!= radiant_set_mem(bd, DEST_FPGA, 
      which == RADIANT_TRIG_A ? RAD_REG_TRIG_TRIGMASKB0 : RAD_REG_TRIG_TRIGMASKB1, 
      4, (uint8_t*) &contributing_channels)) return 1; 
 
  if (which == RADIANT_TRIG_A) bd->trigAen = contributing_channels; 
  else bd->trigBen = contributing_channels; 


  //Check that the global trigger is enabled for all channels, complain if not
  if ( (~bd->triginen) & contributing_channels) 
  {

    if (update_global_bit) 
    {
      radiant_set_global_trigger_mask(bd, bd->triginen | contributing_channels); 
    }
    else
    {
      fprintf(stderr,"WARNING: RF trigger %d has channel enabled not enabled in global trigger mask!\n", which);;
      fprintf(stderr,"  triginen: 0x%x, triginen%d: 0x%x\n", bd->triginen, which, contributing_channels);  
    }
  }



  //set the threshhold

  if (required_coincident_channels) required_coincident_channels-=1; 
  if ( 4!= radiant_set_mem(bd, DEST_FPGA, 
      which == RADIANT_TRIG_A ? RAD_REG_TRIG_TRIGTHRESH0 : RAD_REG_TRIG_TRIGTHRESH1, 
      4, (uint8_t*) &required_coincident_channels)) return 1; 
 

  //set the window. First calculate the cycles we wnat 

  if (coincidence_window_ns < 17.5) coincidence_window_ns = 17.5; 
  if (coincidence_window_ns > 327.5) coincidence_window_ns = 327.5; 
  int cycles = round(coincidence_window_ns/2.5) - 7; 

  uint32_t win[4] = {0}; 
  int iwin = 0;
  while (cycles > 0 && iwin < 4) 
  {
    win[iwin] = cycles > 0x1f ? 0x1f : cycles; 
    cycles-=win[iwin]; 
    iwin++; 
  }

  mem = (win[0] | (win[1] << 5) |( win[2] << 10) | (win[3] << 15)); 
  if ( 4!= radiant_set_mem(bd, DEST_FPGA, 
      which == RADIANT_TRIG_A ? RAD_REG_TRIG_TRIGWINDOW0 : RAD_REG_TRIG_TRIGWINDOW1, 
      4, (uint8_t*) &mem)) return 1; 
 

  //now reenable the trigger if it was enabled, or if we have enabled a trigger
  if (trig_enabled || contributing_channels) 
  {
    if (radiant_set_l0_enable(bd,1)) return 1; 
  }

  return 0; 
}


int radiant_set_trigger_thresholds_float(radiant_dev_t * bd, int ichan_start, int ichan_end, const float  * threshold_V) 
{
  if (ichan_start < 0 || ichan_end > 24 || ichan_start > ichan_end || !bd || !threshold_V) return -1; 

  uint32_t vals[24]; 

  for (int ichan = ichan_start; ichan <=ichan_end; ichan++) 
  {
    if (threshold_V[ichan] > 2.5) vals[ichan] = 16777215; 
    else if (threshold_V[ichan] < 0 ) vals[ichan] = 0; 
    else vals[ichan] = (threshold_V[ichan] / 2.5 * 16777215); 
  }

  return radiant_set_trigger_thresholds(bd,ichan_start, ichan_end, vals+ichan_start); 
}

int radiant_set_trigger_thresholds(radiant_dev_t * bd, int ichan_start, int ichan_end, const uint32_t  * vals) 
{
  if (ichan_start < 0 || ichan_end >= RNO_G_NUM_RADIANT_CHANNELS || ichan_start > ichan_end || !bd || !vals) return -1; 

  //if any are 0, let's just disable? 

  uint32_t oeb = 0;
  if (4!= radiant_get_mem(bd,DEST_FPGA, RAD_REG_TRIG_THRESH_OEB, 4, (uint8_t*) &oeb)) return 1; 
  uint32_t new_oeb = oeb;
  for (int i = ichan_start; i <= ichan_end; i++) 
  {
    if (vals[i-ichan_start]) 
    {
      new_oeb &= ~(1 << i); 
    }
    else
    {
      new_oeb |= (1 << i); 
    }

    //cache the threshold 
    bd->thresh[i] = vals[i-ichan_start]; 
    if (4 != radiant_set_mem(bd,DEST_FPGA, RAD_REG_TRIG_THRESH_BASE+RAD_REG_TRIG_THRESH_INCR*i, 4, (uint8_t*) &vals[i-ichan_start])) return 1; 
  }

  //otherwise make sure they are enabled
  if (new_oeb != oeb)
  {
//    printf("new_oeb: %x  old_oeb: %x\n", new_oeb, oeb) ; 
    if (4!= radiant_set_mem(bd,DEST_FPGA, RAD_REG_TRIG_THRESH_OEB, 4, (uint8_t*) &new_oeb)) return 1; 
  }

  return 0;



}


int radiant_get_trigger_thresholds(radiant_dev_t * bd, int ichan_start, int ichan_end, uint32_t  * vals) 
{
  if (ichan_start < 0 || ichan_end > 24 || ichan_start > ichan_end || !bd || !vals) return -1; 


  int nchan = (ichan_end-ichan_start)+1; 
  memcpy(vals, bd->thresh+ichan_start, 4*nchan); 
  return 0; 
// no readback yet
//  return 4*nchan!=radiant_get_mem(bd, DEST_FPGA, RAD_REG_TRIG_THRESH_BASE+RAD_REG_TRIG_THRESH_INCR * ichan_start, 4*nchan, (uint8_t*) (vals)); 
}

int radiant_get_trigger_thresholds_float(radiant_dev_t * bd, int ichan_start, int ichan_end, float  * threshold_V) 
{
  uint32_t vals[24] = {0}; 
  int ret = radiant_get_trigger_thresholds(bd, ichan_start, ichan_end, vals+ichan_start); 
  if (ret) return ret; 

  for (int ichan = ichan_start; ichan <=ichan_end; ichan++) 
  {
    threshold_V[ichan-ichan_start] = vals[ichan]*2.5/(1677215); 
  }

  return 0; 
}

int radiant_get_scaler_period(radiant_dev_t * bd, float *period) 
{
  if (!bd) return -1; 
  uint32_t mem = 0; 
  if (4!=radiant_get_mem(bd, DEST_FPGA, RAD_REG_SCALER_PERIOD, 4, (uint8_t*) &mem)) return 1; 

  if (mem & (1u << 31)) *period = 0; 
  else
  {
    mem &=(0x7fffffff); 
    *period = mem *1e-6/2.5;  // 400 ns period! 
  }
  return 0; 

}



int radiant_set_scaler_period(radiant_dev_t * bd, float period) 
{
  if (!bd) return -1; 
  if (period < 0) return -1; 
  uint32_t mem = 0; 
  if (period==0)
  {
     mem = 1u << 31; 
  }
  else
  {
    const uint32_t max_uperiod = ( 1u << 31) -1; 
    const float max_period = max_uperiod * 1e-6/2.5; 
    if (period > max_period) period = max_period; 
    uint32_t uperiod = period*1e6*2.5; 
    if (uperiod > max_uperiod) uperiod = max_uperiod; //just in case
    mem = uperiod; 
  }

  return 4!=radiant_set_mem(bd, DEST_FPGA, RAD_REG_SCALER_PERIOD, 4, (uint8_t*) &mem); 
}

int radiant_set_prescaler(radiant_dev_t * bd, int scaler, uint8_t prescale_m1) 
{
  if (scaler < 0 || scaler > 31 || !bd) return -1; 

  uint32_t mem = (scaler << 24) | prescale_m1; 
  if(4!=radiant_set_mem(bd, DEST_FPGA, RAD_REG_SCALER_PRESCALECTL, 4, (uint8_t*) &mem)) return 1; 
  bd->prescal[scaler] = prescale_m1; 
  return 0;
}

int radiant_get_scalers(radiant_dev_t * bd, int sc0, int sc1, uint16_t * scalers) 
{
  if (sc0 < 0 || sc1 > 31 || !bd || sc0 > sc1 || !scalers) return -1; 

  int nscalers = (sc1-sc0)+1; 
  return 2*nscalers != radiant_get_mem(bd, DEST_FPGA, RAD_REG_SCALER_SCAL_BASE + sc0*2, nscalers*2, (uint8_t*) scalers); 
}


int radiant_read_daqstatus(radiant_dev_t * bd, rno_g_daqstatus_t * ds) 
{
  if (!bd || !ds) return -1; 

  struct timespec start; 
  struct timespec end; 

  memcpy(ds->radiant_prescalers, bd->prescal, RNO_G_NUM_RADIANT_CHANNELS); 

  clock_gettime(CLOCK_REALTIME,&start); 
  //TODO just cache these... no need to readback  
  //TODO: check return values
  radiant_get_scaler_period(bd,&ds->radiant_scaler_period); 
  radiant_get_trigger_thresholds(bd,0, RNO_G_NUM_RADIANT_CHANNELS-1, ds->radiant_thresholds); 
  radiant_bm_analog_read_all(bd, &ds->radiant_voltages); 
  radiant_get_scalers(bd,0, RNO_G_NUM_RADIANT_CHANNELS-1, ds->radiant_scalers); 
  clock_gettime(CLOCK_REALTIME,&end); 

  ds->when_radiant = 0.5 * ( start.tv_nsec*1e-9 + end.tv_nsec*1e-9+start.tv_sec + end.tv_sec); 

  return 0;
}


int radiant_set_pps_config(radiant_dev_t *bd, radiant_pps_config_t cfg) 
{
  if (!bd) return -1; 
  static_assert(sizeof(cfg) == sizeof(uint32_t), "radiant pps config wrong size"); 
  return sizeof(cfg)==radiant_set_mem(bd, DEST_FPGA, RAD_REG_PPSSEL, sizeof(cfg), (uint8_t*) &cfg); 
}

int radiant_get_pps_config(radiant_dev_t *bd, radiant_pps_config_t * cfg) 
{
  static_assert(sizeof(*cfg) == sizeof(uint32_t), "radiant pps config wrong size"); 
  return sizeof(cfg)==radiant_get_mem(bd, DEST_FPGA, RAD_REG_PPSSEL, sizeof(*cfg), (uint8_t*) cfg); 
}


int radiant_get_fw_version( const radiant_dev_t* bd, const radiant_dest_t which, 
    uint8_t *major, uint8_t * minor, uint8_t * rev, 
    uint8_t * year, uint8_t * month, uint8_t * day) 
{

  if (!bd) return -1; 

  const date_version_t * dv = which == DEST_FPGA ? &bd->rad_dateversion : &bd->bm_dateversion; 

  if (major) *major = dv->major; 
  if (minor) *minor = dv->minor; 
  if (rev) *rev = dv->rev; 
  if (year) *year = dv->year; 
  if (month) *month = dv->month; 
  if (day) *day = dv->day; 

  return 0; 
}

uint16_t radiant_get_sample_rate(const radiant_dev_t * bd) 
{
  if (!bd) return -1; 
  return 3200; 
}

void radiant_set_internal_triggers_per_cycle(radiant_dev_t * bd, uint16_t n, double sleep) 
{

  n = n > 256 ? 256 : n; 
  if (!n) n =128; //the default; 
  bd->triggers_per_cycle = n; 
  if (sleep < 0) sleep = 0; 
  bd->sleep_per_cycle = sleep; 
  bd->sleep_per_cycle_ts = (struct timespec) {.tv_sec = ((int) sleep), .tv_nsec = round(1e9*(sleep-floor(sleep))) };
}

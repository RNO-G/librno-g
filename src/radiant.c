#include "radiant.h" 
#include "cobs.h" 
#include <time.h> 
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

  // buffers for cobs writing (one per device) 
  uint8_t reg_buf [258]; 
  uint8_t reg_buf_encoded [260]; 
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
    xfers[i].rx_buf = (uintptr_t) bufs[i-3]; 
    xfers[i].len =  2*N[i-3]; 
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
    uint16x8_t vandme = vdupq_n_u16 (andme); 

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
      vst1q_u16( v+i*32,v1);
      vst1q_u16(v+i*32+8,v2);
      vst1q_u16(v+i*32+16,v3);
      vst1q_u16(v+i*32+24,v4);
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
  if (fwhd.magic!= 0x5244 )
  {
    fprintf(stderr,"Bad magic :%d\n", fwhd.magic);
    return -0x5244; 
  }


  // Start filling in the metadata 
  memset(hd,0,sizeof(*hd)); 
  hd->event_number |= (fwhd.counter[1]);
  hd->event_number |= (fwhd.counter[0]) << 16;
  hd->trigger_time |= (fwhd.time[1]);
  hd->trigger_time |= (fwhd.time[0]) << 16;
  hd->run_number = bd->run; 

  int num_status = fwhd.nstatus[1]; 
  num_status |= (fwhd.nstatus[0]) << 16; 

  int num_ev = fwhd.nev[1]; 
  num_ev |= (fwhd.nev[0]) << 16; 

  wf->event_number = hd->event_number; 
  wf->run_number = bd->run; 

  int nsamples = (num_ev-12)/24; 
  wf->radiant_nsamples = nsamples; 

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

    for (int w = 0; w < RNO_G_NUM_RADIANT_WINDOWS; w++) 
    {
      uint16_t val = wf->radiant_waveforms[ichan][RNO_G_RADIANT_WINDOW_SIZE*w];
      if (val & 0x2000)//this is the start window
      {
        hd->radiant_start_windows[ichan] = high_window ? w+RNO_G_NUM_RADIANT_WINDOWS: w; 
        nrotate = w * RNO_G_RADIANT_WINDOW_SIZE; 
        break; 
      }
      else if (val & 0x1000) //this is the stop window 
      {
        //figure out the number of windows we must have read... this might just be constant! 
        int num_windows =  nsamples >> 7; 
        int start = (w-num_windows) % RNO_G_NUM_RADIANT_WINDOWS ;
        hd->radiant_start_windows[ichan] = start + (high_window ? RNO_G_NUM_RADIANT_WINDOWS : 0);  
        nrotate = start * RNO_G_RADIANT_WINDOW_SIZE; 
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
      andall16(wf->radiant_waveforms[ichan], 0x0fff, RNO_G_RADIANT_WINDOW_SIZE); 
      andall16(wf->radiant_waveforms[ichan] + (nsamples-RNO_G_RADIANT_WINDOW_SIZE), 0x0fff, RNO_G_RADIANT_WINDOW_SIZE); 
    }
  }

  return ret; 
}


radiant_dev_t * radiant_open(const char *spi_device, const char * uart_device) 
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
  int spi_clock = 48000000; 
  ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ,&spi_clock); 
  
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
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

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

  //Write two 0s to make sure we're sycnrhonzied
  uint16_t zero = 0; 
  write(uart_fd,&zero,sizeof(zero)); 


  dev = calloc(sizeof(radiant_dev_t),1); 
  dev->spi_fd = spi_fd; 
  dev->uart_fd = uart_fd; 
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


//Convenience method useful for cobs decoding 
static int read_until_zero(radiant_dev_t * bd, double timeout) 
{

  int nread = 0; 

  struct timespec start; 

  if (timeout > 0) 
  {
    clock_gettime(CLOCK_MONOTONIC,&start); 
  }

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
        double elapsed = now.tv_sec - start.tv_sec + 1e9 * (now.tv_nsec - start.tv_nsec); 

        if (elapsed > timeout) 
        {
          return -nread; 
        }
      }

      usleep(1000); //1 ms

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




int radiant_set_mem(radiant_dev_t * bd, radiant_dest_t dest, uint32_t addr, uint8_t len, const uint8_t * bytes) 
{
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

  uint8_t expected[4] = { bd->reg_buf[0], bd->reg_buf[1], bd->reg_buf[2], len}; 


  //read until we get a 0
  int rd = read_until_zero(bd, 0); 
  int decoded_len = cobs_decode_buf(rd, bd->reg_buf_encoded, sizeof(bd->reg_buf), bd->reg_buf); 

  if (!memcmp(expected, bd->reg_buf, sizeof(expected)))
  {
    int i = 0;
    fprintf(stderr, "expected buf mismatch in radiant_set mem.\n"); 
    fprintf(stderr, "  expected: "); 
    for (i = 0; i < sizeof(expected); i++) fprintf(stderr," %02x ", expected[i]);
    fprintf(stderr, "\n  got    : "); 
    for (i = 0; i < decoded_len; i++) fprintf(stderr," %02x ", bd->reg_buf[i]);
    return -decoded_len; 
  }

  return len; 
}

int radiant_get_mem(radiant_dev_t * bd, radiant_dest_t dest, uint32_t addr, uint8_t len, uint8_t * bytes) 
{
  // set up unencoded packet 
  //                //address[21:16]          // dest          
  bd->reg_buf[0] = ((addr >> 16 ) & 0x1f) | ( (dest &1) <<6 ) ; 
  bd->reg_buf[1] =  (addr >> 8) & 0xff; //addr [15:8]
  bd->reg_buf[2] = (addr) & 0xff; 
  bd->reg_buf[3] = 4; 

  int encoded_len = cobs_encode_buf(4, bd->reg_buf, sizeof(bd->reg_buf_encoded), bd->reg_buf_encoded); 
  
  //now write to serial 
  int written = write(bd->uart_fd, bd->reg_buf_encoded,encoded_len); 

  if (written != encoded_len) 
  {
    fprintf(stderr,"Wrote only %d bytes instead of %d in radiant_set_mem\n", written, encoded_len); 
    return -1; 
  }


  uint8_t expected[3] = { bd->reg_buf[0], bd->reg_buf[1], bd->reg_buf[2]}; 

  //read until we get a 0
  int rd = read_until_zero(bd, 0); 
  //decode 
  int decoded_len = cobs_decode_buf(rd, bd->reg_buf_encoded, sizeof(bd->reg_buf), bd->reg_buf); 

  if (decoded_len != len+3) 
  {
    fprintf(stderr, "Lenght mismatch. Got %d bytes, expected %d\n", decoded_len, len+3); 
    return -1; 
  }


  if (!memcmp(expected, bd->reg_buf, sizeof(expected)))
  {
    int i = 0;
    fprintf(stderr, "expected buf mismatch in radiant_get mem.\n"); 
    fprintf(stderr, "  expected: "); 
    for (i = 0; i < sizeof(expected); i++) fprintf(stderr," %02x ", expected[i]);
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


#include <stdlib.h> 
#include <time.h> 
#include <linux/spi/spidev.h> 
#include <string.h> 
#include <unistd.h>
#include <sys/ioctl.h> 
#include <sys/types.h> 
#include <sys/file.h> 
#include <stdio.h> 
#include <stdint.h> 
#include <sys/mman.h>

#define NSAMP 2048
#define NCHAN 24

#define MODE SPI_MODE_0

static uint16_t fake_event[NCHAN][NSAMP]; 



//#define TURBO_HACK 

//from UBOOT files 
#define SPI0_BASE 0x48030000
#define SPI1_BASE 0x481a0000
#define SPI_SIZE 0x1000  //? 
#define CH0CONF_OFFSET 0x12c  
#define TURBO_BIT 19 

int main(int nargs, char ** args) 
{

  int fd = open ("/dev/spidev0.0",O_RDWR); 
  int spi_clock = 48000000; 
  ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ,&spi_clock); 

  int mode = MODE; 
  ioctl(fd, SPI_IOC_WR_MODE,&mode); 

  int ntimes = 25; 
  if (nargs > 1) ntimes = atoi(args[1]); 

#ifdef TURBO_HACK

  int mem = open("/dev/mem",O_RDWR); 
  if ( mem > 0) 
  {
    volatile void * addr = mmap(0,SPI_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem, SPI0_BASE); 
    if (!addr) 
    {
      fprintf(stderr,"mmap failed :(\n"); 
    }
    else
    {
      volatile unsigned int * ch0conf = addr + CH0CONF_OFFSET; 
      printf("ch0conf: %x\n", *ch0conf);
      printf("ch0conf turbo bit: %x\n", (*ch0conf) & (1 << TURBO_BIT));
      printf("oring\n"); 
      *ch0conf |= (1 << TURBO_BIT); 
      printf("ch0conf turbo bit: %x\n", !!((*ch0conf) & (1 << TURBO_BIT)));
    }
    munmap((void*) addr, SPI_SIZE); 
    close(mem); 
  }
  else
  {
    fprintf(stderr,"Can't touch /dev/mem\n"); 
  }


#endif 

  struct spi_ioc_transfer xfers[NCHAN] = {0}; 
  int total_read = 0;

  for (int i = 0; i< NCHAN; i++) 
  {
    xfers[i].rx_buf = (uintptr_t) &fake_event[i][0]; 
    xfers[i].tx_buf = 0; 
    xfers[i].bits_per_word = 32; 
//    xfers[i].speed_hz = 48000000; 
    xfers[i].cs_change = 0; 
    xfers[i].len = NSAMP*2; 
  }


  struct timespec start;
  struct timespec end;
  clock_gettime(CLOCK_MONOTONIC,&start);
  for (int i = 0; i < ntimes; i++) 
  {
    total_read += ioctl(fd, SPI_IOC_MESSAGE(NCHAN), xfers); 
  }
  clock_gettime(CLOCK_MONOTONIC,&end);

  double dt = (end.tv_sec - start.tv_sec) + 1e-9 * (end.tv_nsec - start.tv_nsec); 

  printf("Read %d bytes (%d \"events\") in %g seconds (%g MB/s, %g Hz)\n",  total_read,ntimes, dt, total_read/dt/(1 << 20), ntimes/dt); 

  close(fd); 

}


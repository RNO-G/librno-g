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

#define NSAMP 2048
#define NCHAN 24


static uint16_t fake_event[NCHAN][NSAMP]; 


int main(int nargs, char ** args) 
{

  int fd = open ("/dev/spidev1.0",O_RDWR); 
  int spi_clock = 480000000; 
  ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ,&spi_clock); 

  int ntimes = 25; 
  if (nargs > 1) ntimes = atoi(args[1]); 



  struct spi_ioc_transfer xfers[NCHAN] = {0}; 
  int total_read = 0;

  for (int i = 0; i< NCHAN; i++) 
  {
    xfers[i].rx_buf = (uintptr_t) &fake_event[i][0]; 
    xfers[i].tx_buf = 0; 
    xfers[i].bits_per_word = 32; 
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


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


static uint16_t fake_event[24][2048]; 

int main(int nargs, char ** args) 
{

  int fd = open ("/dev/spidev1.0",O_RDWR); 
  int spi_clock = 48000000; 
  ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ,&spi_clock); 

  int ntimes = 25; 
  if (nargs > 1) ntimes = atoi(args[1]); 



  struct spi_ioc_transfer xfers[24] = {0}; 

  for (int i = 0; i< 24; i++) 
  {
    xfers[i].rx_buf = (uintptr_t) &fake_event[i][0]; 
    xfers[i].tx_buf = 0; 
    xfers[i].len = 4096; 
  }


  struct timespec start;
  struct timespec end;
  clock_gettime(CLOCK_MONOTONIC,&start);
  for (int i = 0; i < ntimes; i++) 
  {
    ioctl(fd, SPI_IOC_MESSAGE(24), xfers); 
  }
  clock_gettime(CLOCK_MONOTONIC,&end);

  double dt = (end.tv_sec - start.tv_sec) + 1e-9 * (end.tv_nsec - start.tv_nsec); 

  printf("Read %d bytes (%d \"events\") in %g seconds (%g MB/s, %g Hz)\n",  24*4096*ntimes,ntimes, dt, 24*4096*ntimes/dt/(1 << 20), ntimes/dt); 

  close(fd); 

}


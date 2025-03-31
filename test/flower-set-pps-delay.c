#include <stdio.h>
#include "flower.h" 
#include <stdlib.h>
#include <math.h>


int main (int nargs, char** args) 
{
  if (nargs < 2) 
  {
    fprintf(stderr,"Usage: flower-set-pps-delay delay_us\n\ndelay_us will be rounded to nearest 0.1 us"); 
    return 1; 
  }
  double delay = atof(args[1]);
  uint32_t delay_int = round(delay * 10);
  if (delay_int >= (1 << 24))
  {
    fprintf(stderr,"Delay too large\n"); 
    return 1;
  }

  flower_dev_t * flwr = flower_open("/dev/spidev1.0",-61); 
  flower_set_delayed_pps_delay(flwr,delay_int);
  flower_close(flwr); 
  return 0;
}

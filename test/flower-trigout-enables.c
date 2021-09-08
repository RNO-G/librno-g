#include "flower.h" 
#include <stdio.h> 
#include <stdlib.h> 


int main(int nargs, char ** args) 
{
  if (nargs < 3) 
  {
    printf("Usage: flower-trigout-enables sysout auxout\n"); 
    return 0; 
  }

 flower_trigout_enables_t cfg; 
 cfg.enable_sysout = atoi(args[1]); 
 cfg.enable_auxout = atoi(args[2]); 
 flower_dev_t * flwr = flower_open("/dev/spidev1.0",-61); 
 flower_set_trigout_enables(flwr, cfg); 
 flower_close(flwr); 

}

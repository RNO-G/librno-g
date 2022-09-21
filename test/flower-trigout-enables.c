#include "flower.h" 
#include <stdio.h> 
#include <stdlib.h> 


int main(int nargs, char ** args) 
{
  if (nargs < 2) 
  {
    printf("Usage: flower-trigout-enables sysout_rf [auxout_rf=0] [sysout_pps = 0] [auxout_pps = 0]\n"); 
    return 0; 
  }

 flower_trigout_enables_t cfg = {0}; 
 cfg.enable_rf_sysout = atoi(args[1]); 
 if (nargs > 2) 
   cfg.enable_rf_sysout = atoi(args[2]); 

 if (nargs > 3) 
   cfg.enable_pps_sysout = atoi(args[3]); 
 if (nargs > 4) 
   cfg.enable_pps_auxout = atoi(args[4]); 
 flower_dev_t * flwr = flower_open("/dev/spidev1.0",-61); 
 flower_set_trigout_enables(flwr, cfg); 
 flower_close(flwr); 

}

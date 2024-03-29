#include "flower.h" 
#include <stdio.h> 
#include <stdlib.h> 


int main(int nargs, char ** args) 
{
  if (nargs < 5) 
  {
    printf("Usage: flower-trigger-enables enablecoinc enablepps enableext enablephased\n"); 
    return 0; 
  }

 flower_trigger_enables_t cfg; 
 cfg.enable_coinc = atoi(args[1]); 
 cfg.enable_pps = atoi(args[2]); 
 cfg.enable_ext = atoi(args[3]); 
 cfg.enable_ext = atoi(args[4]); 

 flower_dev_t * flwr = flower_open("/dev/spidev1.0",-61); 
 flower_set_trigger_enables(flwr, cfg); 
 flower_close(flwr); 

}

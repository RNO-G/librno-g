#include "flower.h" 
#include <stdio.h> 
#include <stdlib.h> 


int main(int nargs, char ** args) 
{
  if (nargs < 4) 
  {
    printf("Usage: flower-configure-trigger window ncoinc vpp_mode\n"); 
    return 0; 
  }
 rno_g_lt_simple_trigger_config_t cfg; 
 rno_g_lt_phased_trigger_config_t phased_cfg; 

 cfg.window = atoi(args[1]); 
 cfg.num_coinc = atoi(args[2]); 
 cfg.vpp_mode = atoi(args[3]); 
 flower_dev_t * flwr = flower_open("/dev/spidev1.0",-61); 
 flower_configure_trigger(flwr, cfg, phased_cfg); 
 flower_close(flwr); 

}

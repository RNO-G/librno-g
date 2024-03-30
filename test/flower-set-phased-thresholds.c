#include <stdio.h>
#include <stdlib.h>
#include "flower.h" 
#include <string.h> 

int main (int nargs, char ** args) 
{
  if (nargs < 2) 
  {
    printf("Usage: flower-set-phased-thresholds TH [servo_frac = 0.75]\n"); 
    return 0; 
  }

  
  float frac = 0.75; 
  //if (nargs > 3) 
  //  frac = atof(args[5]); 

  uint16_t phased_trig_thresh[16]; 
  uint16_t phased_servo_thresh[16]; 
  for (int i = 1; i <=16; i++) 
  {
    phased_trig_thresh[i-1] = atoi(args[1]); 
    phased_servo_thresh[i-1] = frac * trig_thresh[i-1]; 
  }

  flower_dev_t * flwr = flower_open("/dev/spidev1.0",-61); 
  flower_set_phased_thresholds(flwr, phased_trig_thresh, phased_servo_thresh, 0xffff); 
  flower_close(flwr); 
}

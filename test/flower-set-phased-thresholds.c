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

  uint16_t phased_trig_thresh[9]; 
  uint16_t phased_servo_thresh[9]; 
  for (int i = 0; i <9; i++) 
  {
    phased_trig_thresh[i] = atoi(args[1]); 
    phased_servo_thresh[i] = frac * phased_trig_thresh[i]; 
  }

  flower_dev_t * flwr = flower_open("/dev/spidev1.0",-61); 
  flower_set_phased_thresholds(flwr, phased_trig_thresh, phased_servo_thresh, 0xffff); 
  flower_close(flwr); 
}

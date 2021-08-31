#include <stdio.h>
#include <stdlib.h>
#include "flower.h" 
#include <string.h> 

int main (int nargs, char ** args) 
{
  if (nargs < 5) 
  {
    printf("Usage: flower-set-thresholds TH0 TH1 TH2 HT2 [servo_frac = 0.75]"); 
    return 0; 
  }

  
  float frac = 0.75; 
  if (nargs > 5) 
    frac = atof(args[5]); 

  uint8_t trig_thresh[4]; 
  uint8_t servo_thresh[4]; 
  for (int i = 1; i <=4; i++) 
  {
    trig_thresh[i-1] = atoi(args[i]); 
    servo_thresh[i-1] = frac * trig_thresh[i-1]; 
  }

  flower_dev_t * flwr = flower_open("/dev/spidev1.0",-61); 
  flower_set_thresholds(flwr, trig_thresh, servo_thresh, 0xf); 
  flower_close(flwr); 
}

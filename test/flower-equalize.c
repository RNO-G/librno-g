#include <stdio.h>
#include "flower.h" 
#include <stdlib.h> 


int main (int nargs, char ** args) 
{
  float rms = 3; 
  if (nargs > 1) rms = atof(args[1]); 
  if (!rms) rms =3; 

  flower_dev_t * flwr = flower_open("/dev/spidev1.0",-61); 
  flower_equalize(flwr, rms, 0 , FLOWER_EQUALIZE_VERBOSE); 
  flower_close(flwr); 
  return 0; 
}

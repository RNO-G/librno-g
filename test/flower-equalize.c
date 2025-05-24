#include <stdio.h>
#include "flower.h" 
#include <stdlib.h> 


int main (int nargs, char ** args) 
{
  float rms = 3; 
  int do_fine_gain_adjust = 0;
  if (nargs > 1) rms = atof(args[1]); 
  if (nargs > 2) do_fine_gain_adjust = atoi(args[2]);
  if (!rms) rms =3; 

  flower_dev_t * flwr = flower_open("/dev/spidev1.0",-61); 
  flower_equalize(flwr, rms, 0 , FLOWER_EQUALIZE_VERBOSE, do_fine_gain_adjust, 0); 
  flower_close(flwr); 
  return 0; 
}

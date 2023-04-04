#include "radiant.h" 
#include <stdio.h>
#include <string.h> 
#include <unistd.h> 
#include <stdlib.h> 
#include <time.h> 




int main(int nargs, char ** args) 
{
  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant", 46,-61); //not sure if right gpio yet 
  int triggers_per_cycle = 128; 
  float bias_v = 1.5 ; 
  double sleep_per_cycle = 0;
  if (nargs > 1) triggers_per_cycle = atoi(args[1]); 
  if (nargs > 2) sleep_per_cycle = atof(args[2]); 
  if (nargs > 3) bias_v = atof(args[3]); 
  if (!rad) return 1; 
//  printf("!!!--------------------  AT PROGRAM START -------------!!!\n"); 
 // radiant_dump(rad,stdout,0);
  radiant_set_nbuffers_per_readout(rad,1); 
  int bias = 4095 * bias_v /3.3; 
  if (bias > 3072) bias = 3072; 
  if (bias < 0) bias = 0; 
  radiant_set_dc_bias(rad,bias,bias); 

  rno_g_pedestal_t ped; 
  radiant_set_internal_triggers_per_cycle(rad,triggers_per_cycle,sleep_per_cycle); 
  struct timespec start; 
  struct timespec end; 
  clock_gettime(CLOCK_REALTIME, &start); 
  radiant_compute_pedestals(rad, 0xffffff, 512, &ped); 
  clock_gettime(CLOCK_REALTIME, &end); 
  fprintf(stderr,"Time to compute pedestals, %d triggers at a time is %f, bias=%d\n", triggers_per_cycle, end.tv_sec - start.tv_sec + 1e-9 * (end.tv_nsec - start.tv_nsec),bias); 
  radiant_close(rad); 
}



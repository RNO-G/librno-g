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
  if (nargs > 1) triggers_per_cycle = atoi(args[1]); 
  if (!rad) return 1; 
//  printf("!!!--------------------  AT PROGRAM START -------------!!!\n"); 
 // radiant_dump(rad,stdout,0);
  radiant_set_nbuffers_per_readout(rad,1); 
  radiant_set_dc_bias(rad,1861,1861); 

  rno_g_pedestal_t ped; 
  radiant_set_internal_triggers_per_cycle(rad,triggers_per_cycle); 
  struct timespec start; 
  struct timespec end; 
  clock_gettime(CLOCK_REALTIME, &start); 
  radiant_compute_pedestals(rad, 0xffffff, 512, &ped); 
  clock_gettime(CLOCK_REALTIME, &end); 
  fprintf(stderr,"Time to compute pedestals, %d triggers at a time is %f\n", triggers_per_cycle, end.tv_sec - start.tv_sec + 1e-9 * (end.tv_nsec - start.tv_nsec)); 
  radiant_close(rad); 
}



#include "radiant.h" 
#include <stdio.h>
#include <string.h> 
#include <unistd.h> 




int main(void) 
{
  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant", 46,-61); //not sure if right gpio yet 
  if (!rad) return 1; 
//  printf("!!!--------------------  AT PROGRAM START -------------!!!\n"); 
 // radiant_dump(rad,stdout,0);
  radiant_set_nbuffers_per_readout(rad,1); 
  radiant_set_dc_bias(rad,1861,1861); 

  rno_g_pedestal_t ped; 
  radiant_compute_pedestals(rad, 0xffffff, 512, &ped); 
//  radiant_dump(rad,stdout,0);
  rno_g_pedestal_dump(stdout,&ped); 
  radiant_close(rad); 
}



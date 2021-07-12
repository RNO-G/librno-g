#include <stdio.h>
#include "flower.h" 


int main (void) 
{
  flower_dev_t * flwr = flower_open("/dev/spidev1.0",-61); 
  rno_g_daqstatus_t ds; 
  flower_fill_daqstatus(flwr,&ds); 
  flower_close(flwr); 
  rno_g_daqstatus_dump_flower(stdout,&ds); 
  return 0; 
}

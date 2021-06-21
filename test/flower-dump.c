#include <stdio.h>
#include "flower.h" 


int main (void) 
{
  flower_dev_t * flwr = flower_open("/dev/spidev1.0",-61); 
  flower_dump(stdout,flwr); 
  flower_close(flwr); 
  return 0; 
}

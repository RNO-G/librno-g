#include <stdio.h>
#include <stdlib.h>
#include "flower.h" 
int main (int nargs, char ** args) 
{
  flower_dev_t * flwr = flower_open("/dev/spidev1.0",-61); 
 
  int gain_code = 0; 
  if (nargs> 1) gain_code = atoi(args[1]); 

  uint8_t codes[RNO_G_NUM_LT_CHANNELS] = {gain_code,gain_code,gain_code,gain_code}; 

  uint8_t data[RNO_G_NUM_LT_CHANNELS][256]; 
  uint8_t * data_ptr[RNO_G_NUM_LT_CHANNELS] = {data[0],data[1],data[2],data[3] }; 

  printf("Gaincode = %d\n", gain_code); 
  flower_set_gains(flwr,codes); 
  flower_read_waveforms(flwr, 256, data_ptr); 

  for (int i = 0 ; i < RNO_G_NUM_LT_CHANNELS; i++) 
  {
    printf("\n\nCH%d:", i); 
    for (int j = 0; j < 256; j++) 
    {
      if (j % 64 == 0) 
        printf("\n\t"); 
      printf("%03d,",data[i][j]); 
    }
    printf("\n\n"); 
  }

  flower_close(flwr); 


}


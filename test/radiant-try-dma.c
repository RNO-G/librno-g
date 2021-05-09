#include "radiant.h" 
#include <stdio.h>
#include <string.h> 
#include <unistd.h> 




int main(void) 
{
  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant", 46,-61); //not sure if right gpio yet 
  if (!rad) return 1; 
  printf("!!!--------------------  AT PROGRAM START -------------!!!\n"); 
  radiant_dump(rad,stdout,0);


  radiant_dma_txn_count_reset(rad); 
  radiant_dma_engine_reset(rad); 
  radiant_dma_tx_reset(rad); 

  uint8_t mem[1024]; 
  uint8_t mem2[1024]; 
  uint16_t nbytes[2] = {1024,1024};
  uint8_t * memptr[2]= {mem,mem2}; 
  radiant_dma_desc_t desc = {.addr =0, .length=1024, .incr=1, .last=0}; 
  radiant_dma_set_descriptor(rad, 0, desc); 
  radiant_dma_desc_t desc2 = {.addr =1024, .length=1024, .incr=1, .last=1}; 
  radiant_dma_set_descriptor(rad, 1, desc2); 

  radiant_dma_config_t dma_cfg; 
  radiant_fill_dma_config(&dma_cfg, RADIANT_DMA_EVENT_MODE); 
  radiant_configure_dma(rad, &dma_cfg); 


  printf("!!!------ AFTER CONFIGURING, BEFORE DMA REQ-------------!!!\n"); 
  radiant_dump(rad,stdout,0);

  radiant_dma_ctrl_t ctrl= {.dma_req = 1}; 
  radiant_dma_control(rad,ctrl); 

  radiant_read(rad, 2, nbytes,memptr); 
  printf("!!!------ AFTER DMA REQ AND TRYING TO READ-------------!!!\n"); 
  radiant_dump(rad,stdout,0);
  radiant_close(rad); 
}



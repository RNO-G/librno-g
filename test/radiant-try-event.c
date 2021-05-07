#include "radiant.h" 
#include <stdio.h>
#include <unistd.h> 




int main(void) 
{
  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant", 46); //not sure if right gpio yet 
  if (!rad) return 1; 

  printf("At start\n"); 
  radiant_dump(rad,stdout,0); 
  radiant_labs_stop(rad); 
  printf("After stop\n"); 
  radiant_dump(rad,stdout,0); 


  radiant_dma_config_t dma_cfg = {.dma_enable=1, .dma_busy=1}; 
  radiant_configure_dma(rad, &dma_cfg); 
  printf("After cargo culting dma_cfg to 0x3\n"); 
  radiant_dump(rad,stdout,0); 

  radiant_dma_ctrl_t engine_reset = {.engine_reset=1}; 
  radiant_dma_control(rad, engine_reset); 
  usleep(10000);//??
  radiant_dma_ctrl_t rx_reset = {.rx_reset = 1}; 
  radiant_dma_control(rad, rx_reset); 
  usleep(10000);//??
  radiant_dma_ctrl_t tx_reset = {.tx_reset = 1}; 
  radiant_dma_control(rad, tx_reset); 
  usleep(10000);//??

  printf("After DMA reset\n"); 
  radiant_dump(rad,stdout,0); 
  radiant_dma_setup_event(rad, 0xffffff, 1024); 
  printf("After DMA setup\n"); 
  radiant_dump(rad,stdout,0); 

  radiant_labs_start(rad); 
  printf("After lab start\n"); 
  radiant_dump(rad,stdout,0); 
  radiant_force_trigger(rad,1); 

  rno_g_header_t hd = {0}; 
  rno_g_waveform_t wf = {0}; 
//  int pollret = radiant_poll_trigger_ready(rad, 500); 
 // printf("pollret is %d\n", pollret); 
  sleep(1); 


  //why is this necessary after a force trigger? 
  radiant_dma_ctrl_t begin_dma = {.dma_req = 1}; 
  radiant_dma_control(rad, begin_dma); 


  radiant_read_event(rad, &hd, &wf); 

  rno_g_header_dump(stdout, &hd);
  rno_g_waveform_dump(stdout, &wf);


  radiant_close(rad); 


}



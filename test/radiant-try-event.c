#include "radiant.h" 
#include <stdio.h>
#include <string.h> 
#include <unistd.h> 




int main(void) 
{
  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant", 46, -61); //not sure if right gpio yet 
  if (!rad) return 1; 

  rno_g_pedestal_t ped; 
  radiant_compute_pedestals(rad, 0xffffff, 512, &ped); 
  rno_g_pedestal_dump(stdout,  &ped); 
  radiant_set_pedestals(rad,&ped); 

  FILE * fev = fopen("event.csv","w"); 
//  printf("At start\n"); 
//  radiant_dump(rad,stdout,0); 
  radiant_labs_stop(rad); 
////  printf("After stop\n"); 
//  radiant_dump(rad,stdout,0); 

//  radiant_dump(rad,stdout,0); 
  radiant_dma_setup_event(rad, 0xffffff); 
//  printf("After DMA setup\n"); 
//  radiant_dump(rad,stdout,0); 

  radiant_labs_start(rad); 
 // printf("After lab start\n"); 

  for (int i = 0; i < 10; i++) 
  {

//    radiant_dump(rad,stdout,0); 
    radiant_force_trigger(rad,1,1); 

    rno_g_header_t hd;
    rno_g_waveform_t wf; 
    memset(&hd,0xab,sizeof(hd));
    memset(&wf,0xab,sizeof(wf));
  //  int pollret = radiant_poll_trigger_ready(rad, 500); 
   // printf("pollret is %d\n", pollret); 

    usleep(100000);

    radiant_dma_request(rad); 
    usleep(10000);


    radiant_read_event(rad, &hd, &wf); 

    rno_g_header_dump(stdout, &hd);
    rno_g_waveform_dump(stdout, &wf);
    rno_g_header_dump(fev, &hd);
    rno_g_waveform_dump(fev, &wf);
  }

//  printf("At end\n"); 
//  radiant_dump(rad,stdout,0); 
 
  fclose(fev); 

  radiant_close(rad); 


}



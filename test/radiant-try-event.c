#include "radiant.h" 
#include <stdio.h>
#include <string.h> 
#include <zlib.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <time.h> 


static int instrumented_poll(radiant_dev_t * rad, int timeout) 
{
  struct timespec start;
  struct timespec stop;
  clock_gettime(CLOCK_REALTIME,&start);
  int ret = radiant_poll_trigger_ready(rad,timeout); 
  clock_gettime(CLOCK_REALTIME,&stop);

  printf("Time spent in poll: %g\n", stop.tv_sec - start.tv_sec + 1e-9*(stop.tv_nsec - start.tv_nsec)); 

  return ret; 


}


int main(int nargs, char ** args) 
{
  int N = 100; 
  int nbuffers = 2; 
  int gzipped = 0;
  if (nargs > 1) N = atoi(args[1]); 
  if (nargs > 2) nbuffers = atoi(args[2]); 
  if (nargs > 3) gzipped = atoi(args[3]); 


  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant", 46, -61); //not sure if right gpio yet 
  if (!rad) return 1; 

  printf("Setting DC bias to 1550\n"); 
  //set reasonable dc bias
  radiant_set_dc_bias(rad,1550,1550); 
  sleep(1); 



  rno_g_file_handle_t ph; 
  rno_g_init_handle(&ph, gzipped ? "/data/test/peds.dat.gz" : "/data/test/peds.dat", "w"); 


  printf("Computing pedestals...\n");
  rno_g_pedestal_t ped; 
  rno_g_header_t hd;
  rno_g_waveform_t wf; 
  radiant_compute_pedestals(rad, 0xffffff, 512, &ped); 

  // csv for debugging purposes for now 
  FILE * fpedscsv = fopen("peds.csv","w"); 
  radiant_set_pedestals(rad,&ped); 

  printf("Writing out pedestals...\n");

  rno_g_pedestal_dump(fpedscsv,  &ped); 
  fclose(fpedscsv); 
  rno_g_pedestal_write(ph, &ped); 
  rno_g_close_handle(&ph); 


  radiant_labs_stop(rad); 

  //this seems to clear the extra triggers too? 
  radiant_reset_counters(rad); 

  radiant_set_nbuffers_per_readout(rad, nbuffers); 
  radiant_dma_setup_event(rad, 0xffffff); 
  radiant_labs_start(rad); 


  printf("\nStarting %s readout using %d buffer%s!\n", gzipped ? "gzipped" : "uncompresed", nbuffers, nbuffers == 2 ? "s" : ""); 

  rno_g_file_handle_t hh;
  rno_g_file_handle_t eh;

  rno_g_init_handle(&hh, gzipped ? "/data/test/header.dat.gz" : "/data/test/header.dat", "w");
  rno_g_init_handle(&eh, gzipped ? "/data/test/wfs.dat.gz" : "/data/test/wfs.dat", "w");

  struct timespec start;
  struct timespec stop;
  clock_gettime(CLOCK_REALTIME,&start);
  for (int i = 0; i < N; i++) 
  {
    printf("====%d=====\n",i); 


    radiant_force_trigger(rad,1,0); 

    while(!instrumented_poll(rad,10)); 

    radiant_read_event(rad, &hd, &wf); 

    rno_g_header_dump(stdout, &hd);
    rno_g_header_write(hh, &hd); 
    rno_g_waveform_write(eh, &wf); 
  }

  clock_gettime(CLOCK_REALTIME,&stop);
  double time = stop.tv_sec - start.tv_sec + 1e-9*(stop.tv_nsec - start.tv_nsec); 
  printf("Elapsed time: %g, (%g Hz)\n",  time, N/time); 
  printf("  Note: for fastest speed, don't let this print to your terminal!!!\n"); 


  //we can stop the labs too , which by side effect also reverts to 1-buffer readout
  radiant_labs_stop(rad); 

  //set up DMA for one buffer, just in case someone else is relying on it 
  radiant_dma_setup_event(rad, 0xffffff); 

  rno_g_close_handle(&eh); 
  rno_g_close_handle(&hh); 
 
  radiant_close(rad); 
}



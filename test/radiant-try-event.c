#include "radiant.h" 
#include <stdio.h>
#include <string.h> 
#include <zlib.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <time.h> 

//#define ZIPPED  


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
  if (nargs > 1) N = atoi(args[1]); 
  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant", 46, -61); //not sure if right gpio yet 
  if (!rad) return 1; 

#ifdef ZIPPED
  gzFile pf = gzopen("/data/test/peds.gz","w"); 
  rno_g_file_handle_t ph = { .type=RNO_G_GZIP, .handle.gz = pf}; 
#else
  FILE * pf = fopen("/data/test/peds.dat","w"); 
  rno_g_file_handle_t ph = { .type=RNO_G_RAW, .handle.raw = pf}; 
#endif


  printf("Computing pedestals...\n");
  rno_g_pedestal_t ped; 
  radiant_compute_pedestals(rad, 0xffffff, 512, &ped); 
//  rno_g_pedestal_dump(stdout,  &ped); 
  radiant_set_pedestals(rad,&ped); 

  printf("Writing out pedestals...\n");
  rno_g_pedestal_write(ph, &ped); 

#ifdef ZIPPED
  gzclose(pf); 
#else
  fclose(pf);
#endif 

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


  //flush all events

  rno_g_header_t hd;
  rno_g_waveform_t wf; 


  //flush 
  printf("Flushing extant triggers\n"); 
  while (radiant_poll_trigger_ready(rad,100))
  {
    radiant_read_event(rad, &hd, &wf); 
  }

#ifdef ZIPPED
  gzFile hf = gzopen("/data/test/header.gz","w"); 
  rno_g_file_handle_t hh = { .type=RNO_G_GZIP, .handle.gz = hf}; 
  gzFile ef = gzopen("/data/test/wfs.gz","w"); 
  rno_g_file_handle_t eh = { .type=RNO_G_GZIP, .handle.gz = ef }; 
#else

  FILE* hf = fopen("/data/test/header.dat","w"); 
  rno_g_file_handle_t hh = { .type=RNO_G_RAW, .handle.raw = hf}; 
  FILE* ef = fopen("/data/test/wfs.dat","w"); 
  rno_g_file_handle_t eh = { .type=RNO_G_RAW, .handle.raw = ef }; 
#endif





  struct timespec start;
  struct timespec stop;
  clock_gettime(CLOCK_REALTIME,&start);
  for (int i = 0; i < N; i++) 
  {
    printf("%d\n",i); 

//    radiant_dump(rad,stdout,0); 

    radiant_force_trigger(rad,1,0); 

    while(!instrumented_poll(rad,10)); 

    radiant_read_event(rad, &hd, &wf); 

    rno_g_header_dump(stdout, &hd);
    rno_g_header_write(hh, &hd); 
    rno_g_waveform_write(eh, &wf); 
//    rno_g_waveform_dump(stdout, &wf);
//    rno_g_header_dump(fev, &hd);
//    rno_g_waveform_dump(fev, &wf);
  }

  clock_gettime(CLOCK_REALTIME,&stop);
  double time = stop.tv_sec - start.tv_sec + 1e-9*(stop.tv_nsec - start.tv_nsec); 
  printf("Elapsed time: %g, (%g Hz)\n",  time, N/time); 

//  printf("At end\n"); 
//  radiant_dump(rad,stdout,0); 
 
#ifdef ZIPPED
  gzclose(ef);
  gzclose(hf);
#else
  fclose(ef);
  fclose(hf);
#endif 

  radiant_close(rad); 


}



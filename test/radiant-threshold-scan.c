#include "radiant.h" 
#include <stdio.h>
#include <string.h> 
#include <zlib.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <time.h> 
#include <signal.h> 



int main(int nargs, char ** args) 
{
  int bias = 0; 
  int nbuffers = 2;
  double start = 0.5; 
  double stop =1.5; 
  double step = 0.01; 
  double period = 1; 

  if (nargs > 1) start = atof(args[1]); 
  if (nargs > 2) stop = atof(args[2]); 
  if (nargs > 3) step = atof(args[3]); 
  if (nargs > 4) period = atof(args[4]); 

  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant", 46, -61);

  if (!rad) return 1; 

  if (bias) 
  {
    printf("Setting DC bias to %u\n",bias); 
    //set reasonable dc bias
    radiant_set_dc_bias(rad,bias,bias); 
    sleep(1); 
  }


  float thresh[24]; 
  rno_g_daqstatus_t ds; 

  double T = start; 

  radiant_labs_stop(rad); 

  //Start with well known prescaler
  for (int i = 0; i < 24; i++) 
  {
//    radiant_set_prescaler(rad,i,0); 
  }

  //start with well known period
  radiant_set_scaler_period(rad,period); 

  radiant_reset_counters(rad); 
  radiant_set_nbuffers_per_readout(rad, nbuffers); 
  radiant_dma_setup_event(rad, 0xffffff); 
  radiant_configure_rf_trigger(rad, RADIANT_TRIG_A, 0xffffff, 23, 100.); 
  radiant_labs_start(rad); 

  radiant_trigger_enable(rad, RADIANT_TRIG_EN,0); 

  while ( (step >= 0 && T <= stop) || ( step < 0 && T>=stop))
  {
    printf("====THRESHOLD IS %f V====\n", T); 
    for (int i = 0; i < 24; i++) 
    {
      thresh[i]=T; 
    }
    radiant_set_trigger_thresholds_float(rad,0,23,thresh); 
    sleep(2); 

    radiant_read_daqstatus(rad, &ds);

    rno_g_daqstatus_dump(stdout, &ds); 
    int all_ok = 1; 
    for (int i = 0; i < 24; i++) 
    {
      if (ds.scalers[i] > 32767 && ds.prescalers[i] < 255)
      {
        all_ok = 0;
        int to = 1.5*(ds.prescalers[i]+1); 
        if (to > 255) to = 255; 
        printf("Increasing prescaler for CH %d to %d\n", i, to); 
        radiant_set_prescaler(rad,i,to); 
      }
    }

    if (!all_ok) 
      continue; 
    T+=step; 
  }


  radiant_labs_stop(rad); 
  radiant_close(rad);

}

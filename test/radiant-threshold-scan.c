#include "radiant.h" 
#include <stdio.h>
#include <string.h> 
#include <zlib.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <time.h> 
#include <signal.h> 
#include <math.h> 

volatile int quit = 0; 

void sighandler(int sig) 
{
  printf("Got signal %d, stopping\n", sig); 
  quit =1; 
}


int main(int nargs, char ** args) 
{
  double start = 0.5; 
  double stop =1.5; 
  double step = 0.01; 
  double period = 1; 
  double goal = 10000; 
  int N = 5; 

  if (nargs > 1) start = atof(args[1]); 
  if (nargs > 2) stop = atof(args[2]); 
  if (nargs > 3) step = atof(args[3]); 
  if (nargs > 4) period = atof(args[4]); 
  if (nargs > 5) goal = atof(args[5]); 

  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant", 46, -61);

  if (!rad) return 1; 

  float thresh[24]; 
  rno_g_daqstatus_t ds; 

  double T = start; 

  //set up the output file 
  rno_g_file_handle_t h; 
  rno_g_init_handle(&h,"thscan.dat","w"); 

  //Start with well known prescaler
  for (int i = 0; i < 24; i++) 
  {
    radiant_set_prescaler(rad,i,0); 
  }

  //start with well known period
  if (period == 0)  
  {

    // force the internal pps, for now... TODO: maybe set it back at tne end if it was different? *shrugs* 
    radiant_pps_config_t pps; 
    radiant_get_pps_config(rad, &pps); 
    if (!pps.use_internal_pps)
    {
      pps.use_internal_pps = 1; 
      radiant_set_pps_config(rad,pps); 
    }
  }

  radiant_set_scaler_period(rad,period); 
  if (period == 0) period = 1; //this is PPS


  //ok I don't need all of this but this will set TRIGINEN properly at least! 
  radiant_reset_counters(rad); 
  radiant_set_global_trigger_mask(rad,0xffffff); 
  radiant_set_l0_enable(rad,1); 


  int thisN = N; 
  float goal_adj_scalers[RNO_G_NUM_RADIANT_CHANNELS] = {0}; 
  float goal_thresh[RNO_G_NUM_RADIANT_CHANNELS] = {0}; 

  radiant_dump(rad,stdout,0); 

  signal(SIGINT, sighandler); 

  while ( (step >= 0 && T <= stop) || ( step < 0 && T>=stop))
  {
    if (quit) break; 

    printf("====THRESHOLD IS %f V====\n", T); 
    for (int i = 0; i < 24; i++) 
    {
      thresh[i]=T; 
    }
    radiant_set_trigger_thresholds_float(rad,0,23,thresh); 
    usleep(3*period*1e6+5000); 
    radiant_read_daqstatus(rad, &ds);
    rno_g_daqstatus_dump(stdout, &ds); 
    rno_g_daqstatus_write(h,&ds); 

    int all_ok = 1; 
    for (int i = 0; i < 24; i++) 
    {
      if (ds.radiant_scalers[i] > 32767 && ds.radiant_prescalers[i] < 255)
      {
        all_ok = 0;
        int to = 1.5*(ds.radiant_prescalers[i]+1); 
        if (to > 255) to = 255; 
        printf("Increasing prescaler for CH %d to %d\n", i, to); 
        radiant_set_prescaler(rad,i,to); 
      }

      float adj_scaler  = ds.radiant_scalers[i] / period * (1+ds.radiant_prescalers[i]); 
      if (goal && adj_scaler && ( goal_thresh[i] == 0 || ( fabs(adj_scaler - goal) < fabs(goal_adj_scalers[i] - goal))))
      {
        printf("Setting best thresh for %d to %f (adj_scaler=%f, prev=%f)\n", i, T, adj_scaler, goal_adj_scalers[i]); 

        goal_adj_scalers[i] = adj_scaler; 
        goal_thresh[i] = T; 
      }
    }

    if (!all_ok || thisN--) 
      continue; 
    T+=step; 
    thisN = N; 
  }

  radiant_set_scaler_period(rad,1.); 
  //end with well known prescaler
  for (int i = 0; i < 24; i++) 
  {
    radiant_set_prescaler(rad,i,0); 
  }

  if (goal && !quit) 
  {
    printf("Setting thresholds closet to goal (%f)\n", goal); 
    radiant_set_trigger_thresholds_float(rad,0,23,goal_thresh); 
    sleep(3); 
    radiant_read_daqstatus(rad, &ds);
    rno_g_daqstatus_dump(stdout, &ds); 
  }

  rno_g_close_handle(&h); 

  radiant_close(rad);
}

#include "radiant.h" 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> 


double start = 0.7; 
double step = 0.01; 
double pulse_freq = 1000; 
double scaler_period = 0.1; 
int prescaler = 0;
int debug = 0;


int parse_args(int nargs, char ** args) 
{
  for (int iarg = 1; iarg < nargs; iarg++) 
  {
    if (!strcmp(args[iarg],"--start") && iarg < nargs-1)
    {
      start = atof(args[iarg+1]); 
      iarg++;
    }
    else if (!strcmp(args[iarg],"--step") && iarg < nargs-1)
    {
      step = atof(args[iarg+1]); 
      iarg++;
    }
    else if (!strcmp(args[iarg],"--pulse_freq") && iarg < nargs-1)
    {
      pulse_freq = atof(args[iarg+1]); 
      iarg++;
    }
    else if (!strcmp(args[iarg],"--scaler_period") && iarg < nargs-1)
    {
      scaler_period = atof(args[iarg+1]); 
      iarg++;
    }
    else if (!strcmp(args[iarg],"--scaler_prescaler") && iarg < nargs-1)
    {
      prescaler = atoi(args[iarg+1]); 
      iarg++;
    }
    else if (!strcmp(args[iarg],"--debug"))
    {
      debug =1; 
    }
    else
    {
      printf("radiant-check-trigger [--start STARTTHRESH=%g] [--step STEPSIZE=%g] [--pulse_freq FREQ=%g] [--scaler_period PERIOD=%g] [--scaler_prescaler PRESCALER=%d]\n", start, step, pulse_freq, scaler_period, prescaler); 
      return 1;
    }
  }
  return 0;
}

int main(int nargs, char ** args) 
{
  if (parse_args(nargs, args))
  {
    return 1; 
  }

  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant", 46, -61);

  if (!rad) return 1; 

  //Start with well known prescaler
  for (int i = 0; i < 24; i++) 
  {
    radiant_set_prescaler(rad,i,prescaler); 
  }
  //start with well known period
  if (scaler_period == 0)  
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

  radiant_set_scaler_period(rad,scaler_period); 
  if (!scaler_period) scaler_period = 1; //this is pps; 

  //set up trigger
  radiant_reset_fifo_counters(rad); 
  radiant_set_global_trigger_mask(rad,0xffffff); 
  radiant_set_l0_enable(rad,1); 

  double first_thresh[24]; 
  int first_thresh_scal[24];
  float thresh[24]; 
  for (int i = 0; i < 24; i++) 
  {
    first_thresh[i] = -1; 
    first_thresh_scal[i] = 0; 
    thresh[i] = start; 
  }

  //set up pulser
  radiant_cal_config_t cfg = 
  {
    .band = RADIANT_CAL_600_PLUS, 
    .pulse_settings = 
    {
      .pulse_period_ns = (1./pulse_freq)/5, 
      .pulse_sharpen = 0,
      .pulse_disable = 0
    }, 
    .pulse_type = RADIANT_CAL_PULSE 
  };
  
  radiant_configure_cal(rad, &cfg); 


  printf("Searching for trigger pulse, start threshold = %g, step = %g, pulse_freq = %g, scaler_period = %g\n", start, step, pulse_freq, scaler_period); 

  for (int iquadpair = 0; iquadpair < 3; iquadpair++) 
  {

     //enable the right quads 
     radiant_enable_cal_mode(rad, iquadpair);
     radiant_enable_cal_mode(rad, iquadpair+3);
     if (debug) radiant_dump(rad,stdout,0); 
     
     float current_thresh = start; 
     int ndone = 0; 

     while (ndone < 8) 
     {
        //set thresholds
        for (int ichan = 0; ichan < 4; ichan++) 
        {
           int chan0 = iquadpair * 4 + ichan;
           int chan1 = (iquadpair+3) * 4 + ichan;
           if (!first_thresh_scal[chan0])
           {
             thresh[chan0] = current_thresh; 
           }
           if (!first_thresh_scal[chan1])
           {
             thresh[chan1] = current_thresh; 
           }
        }
        radiant_set_trigger_thresholds_float(rad,0,23,thresh); 

        //wait a bit
        usleep(1e6 * (3 * scaler_period + 0.01)); 

        //measure thresholds
        rno_g_daqstatus_t ds = {0}; 
        radiant_read_daqstatus(rad, &ds); 

        //check thresholds 
        for (int ichan = 0; ichan < 4; ichan++) 
        {
           int chan0 = iquadpair * 4 + ichan;
           int chan1 = (iquadpair+3) * 4 + ichan;

           int scal0 = ds.radiant_scalers[chan0] * (1 + ds.radiant_prescalers[chan0]) / scaler_period;
           int scal1 = ds.radiant_scalers[chan1] * (1 + ds.radiant_prescalers[chan1]) / scaler_period;
           if (debug) printf("ch %d: %d at %g\n", chan0, scal0, thresh[chan0]);
           if (debug) printf("ch %d: %d at %g\n", chan1, scal1, thresh[chan1]);

           if (scal0 > 2 * pulse_freq ) 
           {
              printf("Uh oh, we measured %d but wanted %g on channel %d\n", scal0, pulse_freq, chan0); 
           }

           if (scal1 > 2 * pulse_freq ) 
           {
              printf("Uh oh, we measured %d but wanted %g on channel %d\n", scal1, pulse_freq, chan1); 
           }

           if (scal0 >= pulse_freq && !first_thresh_scal[chan0]) 
           {
              first_thresh[chan0] =current_thresh; 
              first_thresh_scal[chan0] = scal0; 
              ndone++;
           }
           if (scal1 >= pulse_freq && !first_thresh_scal[chan1]) 
           {
              first_thresh_scal[chan1] = scal1; 
              first_thresh[chan1] = current_thresh; 
              ndone++;
           }
        }

        current_thresh += step; 
        if (current_thresh > 2.5) 
        {
           fprintf(stderr,"WARNING: reached 2.5 V threshold. Aborting loop\n"); 
           break;
        }
     }
  }

  //disable all quads
  for (int iquad = 0; iquad < 6; iquad++) radiant_disable_cal_mode(rad, iquad); 

  //disable the pulser switch to be back to the sine wave
  cfg.pulse_type = RADIANT_CAL_SINE; 
  radiant_configure_cal(rad,&cfg); 


  //print out results 
  for (int ichan = 0; ichan < 24; ichan++) 
  {
    printf("CHAN %d, reached %d Hz at threshold of %f V\n", ichan, first_thresh_scal[ichan], first_thresh[ichan]);
  }
  radiant_close(rad); 
}

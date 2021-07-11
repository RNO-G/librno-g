#include "radiant.h" 
#include <stdio.h>
#include <string.h> 
#include <zlib.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <time.h> 
#include <signal.h> 


static int instrumented_poll(radiant_dev_t * rad, int timeout, int verbose) 
{
  struct timespec start;
  struct timespec stop;
  clock_gettime(CLOCK_REALTIME,&start);
  int ret = radiant_poll_trigger_ready(rad,timeout); 
  clock_gettime(CLOCK_REALTIME,&stop);

  if (verbose) printf("Time spent in poll: %g\n", stop.tv_sec - start.tv_sec + 1e-9*(stop.tv_nsec - start.tv_nsec)); 

  return ret; 


}

double get_now() 
{
  struct timespec spec; 
  clock_gettime(CLOCK_REALTIME, &spec); 
  return spec.tv_sec + 1e-9 * spec.tv_nsec; 
}

int usage() 
{
  printf("radiant-try-event [-N NEVENTS=100] [-b buffers=2] [-M TRIGMASK=0x37b000] [-W TRIGWINDOW=20] [-T THRESH=0.2] [-C MINCOINCIDENT =3] [-B BIAS=1100]  [-z] [-f] [-I INTERVAL=0] [-p]  [-c] [-h]\n"); 
  printf("  -N NEVENTS number of events\n"); 
  printf("  -b BUFFERS number of buffers\n"); 
  printf("  -M TRIGMASK  trigger mask used (default 0x37b000)\n"); 
  printf("  -W TRIGWINDOW  ns for trig window\n") ;
  printf("  -T TRIGTHRESH  V (0 to keep whatever is current) \n") ;
  printf("  -C MINCOINCIDENT  minimum concident\n") ;
  printf("  -B BIAS  the DAC count for the bias\n") ;
  printf("  -z gzip poutput\n"); ;
  printf("  -f force triggers\n"); 
  printf("  -I force trigger interval (default 0 means as fast as possible) \n"); 
  printf("  -p PPS triggers enabled\n"); 
  printf("  -i Use internal PPS\n"); 
  printf("  -P poll amount = 0.1\n"); 
  printf("  -x external triggers\n"); 
  printf("  -c trigger clear mode\n"); 
  printf("  -v verbose\n"); 
  printf("  -h this message\n"); 
  exit(1); 

}


volatile int quit = 0; 
void sighandler(int sig) 
{
  printf("Got signal %d, quitting\n", sig); 
  quit =1; 
}

int last_daqstatus = 0;
int maybe_dump_daqstatus(radiant_dev_t * rad, rno_g_file_handle_t h,  rno_g_daqstatus_t* ds )
{

  int now = time(0); 
  if (now > last_daqstatus+10)
  {
    radiant_read_daqstatus(rad, ds); 
    rno_g_daqstatus_write(h, ds); 
    rno_g_daqstatus_dump(stdout,ds);
    last_daqstatus = now; 
    return 1;
  }
  return 0; 
}

int main(int nargs, char ** args) 
{
  int N = 100; 
  int nbuffers = 2; 
  int gzipped = 0;
  int verbose = 0; 
  int enable_pps = 0; 
  int enable_ext = 0; 
  int do_sync = 0; 
  double force_interval = 0; 
  int force = 0; 
  int clearmode = 0; 
  uint32_t trigmask = 0x37b000; 
  float trigwindow = 20; 
  float trigthresh = 0; 
  uint8_t mincoincident = 3; 
  uint16_t bias = 1100; 
  double last_force = 0; 
  double poll_amount = 0.01; 
  int internal_pps = 0; 

  for (int i = 1; i < nargs; i++) 
  {

    int last = (i == (nargs-1)); 

    if (!strcmp(args[i],"-h"))
    {
      usage(); 
    }

    else if (!strcmp(args[i],"-f"))
    {
      force = 1; 
    }
    else if (!strcmp(args[i],"-p"))
    {
      enable_pps = 1; 
    }
     else if (!strcmp(args[i],"-i"))
    {
      internal_pps = 1; 
    }
 
    else if (!strcmp(args[i],"-z"))
    {
      gzipped = 1; 
    }
    else if (!strcmp(args[i],"-v"))
    {
      verbose = 1; 
    }
    else if (!strcmp(args[i],"-x"))
    {
      enable_ext = 1; 
    }

    else if (!strcmp(args[i],"-S"))
    {
      do_sync = 1; 
    }

    else if (!strcmp(args[i],"-c"))
    {
      clearmode = 1; 
    }

    else if(!last) 
    {
      if (!strcmp(args[i],"-N"))
      {
        N = atoi(args[++i]);
      }
      else if (!strcmp(args[i],"-b"))
      {
        nbuffers = atoi(args[++i]);
      }
 
      else if (!strcmp(args[i],"-M"))
      {
         trigmask = strtol(args[++i], 0, 0);
      }
      else if (!strcmp(args[i],"-W"))
      {
        trigwindow = atof(args[++i]); 
      }
      else if (!strcmp(args[i],"-I"))
      {
        force_interval = atof(args[++i]); 
      }
      else if (!strcmp(args[i],"-T"))
      {
        trigthresh = atof(args[++i]); 
      }
      else if (!strcmp(args[i],"-C"))
      {
        mincoincident = atoi(args[++i]); 
      }
      else if (!strcmp(args[i],"-P"))
      {
        poll_amount = atoi(args[++i]); 
      }
 
      else usage(); 
    }
    else 
    {
      usage(); 
    }

  }

  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant", 46, -61);
  if (!rad) return 1; 


  if (do_sync) radiant_sync(rad); 
  if (bias) 
  {
    printf("Setting DC bias to %u\n",bias); 
    //set reasonable dc bias
    radiant_set_dc_bias(rad,bias,bias); 
  }
  sleep(1); 



  rno_g_file_handle_t ph; 
  rno_g_init_handle(&ph, gzipped ? "/data/test/peds.dat.gz" : "/data/test/peds.dat", "w"); 


  int sig_is_enabled = 0; 
  radiant_cal_enabled(rad,&sig_is_enabled); 

  if (sig_is_enabled) radiant_enable_cal(rad,0); 

  printf("Computing pedestals...\n");
  rno_g_pedestal_t ped; 
  rno_g_header_t hd;
  rno_g_waveform_t wf; 
  rno_g_daqstatus_t ds; 
  radiant_compute_pedestals(rad, 0xffffff, 512, &ped); 

  // csv for debugging purposes for now 
  FILE * fpedscsv = fopen("peds.csv","w"); 
  radiant_set_pedestals(rad,&ped); 

  if (sig_is_enabled) radiant_enable_cal(rad,1); 
  printf("Writing out pedestals...\n");

  rno_g_pedestal_dump(fpedscsv,  &ped); 
  fclose(fpedscsv); 
  rno_g_pedestal_write(ph, &ped); 
  rno_g_close_handle(&ph); 


  radiant_labs_stop(rad); 

  //this seems to clear the extra triggers too? 
  radiant_reset_fifo_counters(rad); 
  radiant_set_nbuffers_per_readout(rad, nbuffers); 
  radiant_dma_setup_event(rad, 0xffffff); 
  int enables = RADIANT_TRIGOUT_EN| RADIANT_TRIGOUT_SOFT | RADIANT_TRIG_EN; 

  printf("\nStarting %s readout using %d buffer%s!\n", gzipped ? "gzipped" : "uncompresed", nbuffers, nbuffers == 2 ? "s" : ""); 
  if (force) 
  {
    printf("Using force triggers\n"); 
  }

  printf("Using RF trigger settings:  MASK: 0x%x, THRESH: %f V, WINDOW: %f ns, MINCOINC: %d\n", trigmask, trigthresh, trigwindow, mincoincident); 

  //let's set the thresholds now (for all channels) 
  
  if (trigthresh) 
  {
    float all_thresh[RNO_G_NUM_RADIANT_CHANNELS]; 
    for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
    {
          all_thresh[i] = trigthresh; 
    }

    radiant_set_trigger_thresholds_float(rad,0, RNO_G_NUM_RADIANT_CHANNELS-1, all_thresh); 
  }

  //and make sure our mask is enabled globally 
  radiant_set_global_trigger_mask(rad,trigmask); 


      //we'll use TRIG A
  radiant_configure_rf_trigger(rad,RADIANT_TRIG_A, trigmask, mincoincident, trigwindow); 

      //make sure TRIG B isn't doing anything
  radiant_configure_rf_trigger(rad,RADIANT_TRIG_B, 0, 0, 0); 



  if (clearmode)
  {
    printf("Clear mode is on\n"); 
    enables |= RADIANT_TRIG_CPUFLOW; 
  }

  if (enable_pps) 
  {
    enables |= RADIANT_TRIG_PPS; 

    radiant_pps_config_t ppsconf; 
    radiant_get_pps_config(rad,&ppsconf); 
    ppsconf.use_internal_pps = internal_pps; 
    radiant_set_pps_config(rad,ppsconf); 
    printf("%s PPS is on\n", internal_pps ? "INTERNAL" : "EXTERNAL" ); 

  }

  if (enable_ext) 
  {
    printf("EXT is on\n"); 
    enables |= RADIANT_TRIG_EXT; 

  }


  radiant_labs_start(rad); 


  rno_g_file_handle_t hh;
  rno_g_file_handle_t eh;
  rno_g_file_handle_t dsh;

  rno_g_init_handle(&hh, gzipped ? "/data/test/header.dat.gz" : "/data/test/header.dat", "w");
  rno_g_init_handle(&eh, gzipped ? "/data/test/wfs.dat.gz" : "/data/test/wfs.dat", "w");
  rno_g_init_handle(&dsh, gzipped ? "/data/test/daqstatus.dat.gz" : "/data/test/daqstatus.dat", "w");


  signal(SIGINT, sighandler); 
  struct timespec start;
  struct timespec stop;

  radiant_trigger_enable(rad,enables,0);
  clock_gettime(CLOCK_REALTIME,&start);

  int i = 0;
  for (i = 0; i < N; i++) 
  {
    if (quit) break;
    printf("====%d=====\n",i); 


    if (maybe_dump_daqstatus(rad,dsh,&ds))
    {
        radiant_dump(rad,stdout,0); 
    }

    while(!instrumented_poll(rad, clearmode ? 0 :  poll_amount*1000, verbose) && !quit) 
    {
      if (maybe_dump_daqstatus(rad,dsh,&ds))
      {
        radiant_dump(rad,stdout,0); 
      }
      int bsy,clear_pending;; 
      radiant_trigger_busy(rad, &bsy, &clear_pending); 
      if (verbose) printf("OVERLORDSTAT is: bsy: %d, clear_pending: %d\n",  bsy, clear_pending); 
      if (clearmode) 
      {
        if (clear_pending) radiant_cpu_clear(rad); 
        else  usleep(1e6*poll_amount); //make up for the short poll 
      }

      if (force) 
      {
        double now = get_now(); 
        if (now > last_force + force_interval) 
        {
          printf("Sending force trigger\n"); 
          radiant_soft_trigger(rad); 
          last_force = now; 
        }
      }
    }

    if (quit) break; 

    radiant_read_event(rad, &hd, &wf); 

    rno_g_header_dump(stdout, &hd);
    rno_g_header_write(hh, &hd); 
    rno_g_waveform_write(eh, &wf); 
  }

  clock_gettime(CLOCK_REALTIME,&stop);
  double time = stop.tv_sec - start.tv_sec + 1e-9*(stop.tv_nsec - start.tv_nsec); 
  printf("Elapsed time: %g, (%g Hz)\n",  time, i/time); 
  printf("  Note: for fastest speed, don't let this print to your terminal!!!\n"); 


  //disable trigger
  radiant_trigger_enable(rad,0,0);

  radiant_labs_stop(rad); 

  rno_g_close_handle(&eh); 
  rno_g_close_handle(&hh); 
  rno_g_close_handle(&dsh); 

  radiant_dma_setup_event(rad, 0xffffff); 
 
  radiant_close(rad); 
}



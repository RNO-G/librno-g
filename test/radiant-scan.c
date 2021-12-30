#include "radiant.h" 
#include <stdio.h>
#include <string.h> 
#include <stdlib.h> 
#include <unistd.h> 



int step = 16; 
int navg = 512; 
float sleepamt = 1; 

const char * outfile = "scan.dat.gz"; 

int usage() 
{
  printf("Usage:  radiant-scan [--out,-o OUTFILE=scan.dat.gz] [--step,-s  STEPSIZE=16]  [--navg,-n NUMAVGSPERSTEP=512] [--sleep,-z seconds_to_sleep=1 \n"); 
  exit(1); 
}

int main(int nargs, char ** args) 
{
  for (int iarg = 1; iarg < nargs; iarg++)
  {
    if (iarg == nargs-1) usage(); 

    if (!strcmp(args[iarg],"--out") || !strcmp(args[iarg],"-o"))
    {
      outfile = args[++iarg]; 
    }

    else if (!strcmp(args[iarg],"--step") || !strcmp(args[iarg],"-s"))
    {
      step = atoi(args[++iarg]); 
    }
    else if (!strcmp(args[iarg],"--navg") || !strcmp(args[iarg],"-n"))
    {
      navg = atoi(args[++iarg]); 
    }
    else if (!strcmp(args[iarg],"--sleep") || !strcmp(args[iarg],"-z"))
    {
      sleepamt = atof(args[++iarg]); 
    }
    else
    {
      usage(); 

    }
  }


  rno_g_file_handle_t h; 
  if (rno_g_init_handle(&h, outfile,"w"))  
  {
    fprintf(stderr,"Trouble opening %s for writing\n", outfile); 
    return 1; 
  }



  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant", 46,-61);
  if (!rad) return 1; 

  int usleepamt = sleepamt * 1e6; 

  rno_g_pedestal_t ped; 
  //Get the station number
  FILE * fstation = fopen("/STATION_ID","r"); 
  if (fstation) 
  {
    int istation = 0; 
    fscanf(fstation,"%d",&istation);
    ped.station = istation; 
    fclose(fstation); 
  }
  else
  {
    ped.station = 0; 
  }

  for (int val = 0; val <3072; val+= step) 
  {
    radiant_set_dc_bias(rad,val,val); 
    printf("BIAS=%d\n", val); 
    usleep(usleepamt); 
    radiant_compute_pedestals(rad, 0xffffff, navg, &ped); 
    rno_g_pedestal_write(h,&ped); 
  }


  printf("Wrote to %s\n", outfile); 
  radiant_close(rad); 
  rno_g_close_handle(&h); 

  return 0; 

}



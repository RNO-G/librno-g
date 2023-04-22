#include "rno-g.h" 
#include "rno-g-wf-utils.h" 
#include <stdlib.h>
#include <stdio.h> 


uint32_t mask = 0xffffff; 


static rno_g_zerocross_stats_t zc[24]; 
int init = 0; 


int main (int nargs, char ** args) 
{
  if (nargs < 4 || nargs % 2 == 1) 
  {
    fprintf(stderr,"Usage: %s mask wf.dat hd.dat  [wf2.dat hd2.dat]\n", args[0]);  
    return 1; 
  }
  
  mask = strtol(args[1],0,0);

  for (int pair = 0; pair < (nargs-1)/2; pair++) 
  {
    rno_g_file_handle_t h_hdr; 
    rno_g_file_handle_t h_wf; 
    rno_g_init_handle(&h_wf, args[2+2*pair],"r"); 
    rno_g_init_handle(&h_hdr, args[3+2*pair],"r"); 

    rno_g_header_t hd; 
    rno_g_waveform_t wf; 
    while (rno_g_header_read(h_hdr, &hd) >  0 && rno_g_waveform_read(h_wf, &wf)> 0)
    {


      if (!init) 
      {
        for (int i = 0; i  <24; i++) 
        {
          if (mask & (1 << i))
          {
            rno_g_zerocross_stats_init(&zc[i], hd.station_number,i); 
          }
        }
        init = 1; 
      }

      for (int i = 0; i  <24; i++) 
      {
        if (mask & (1 << i))
        {
          rno_g_zerocross_stats_process(&zc[i], &hd, &wf); 
        }
      }
    }
  }

  printf("{\n  \"channels\" = [\n"); 
  for (int i = 0; i  <24; i++) 
  {
    if (mask & (1 << i))
    {
      rno_g_zerocross_stats_dump(stdout,&zc[i],4); 
      printf(",\n"); 
    }
  }
  printf("  ]\n}\n"); 
}

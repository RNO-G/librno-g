#include <stdlib.h> 
#include <zlib.h> 
#include "rno-g.h" 
#include <stdio.h> 
#include <math.h>


int main(int nargs, char ** args) 
{

  rno_g_waveform_t wf; 
  for (int i = 1; i < nargs; i++) 
  {

    gzFile f = gzopen(args[i],"r"); 

    rno_g_file_handle_t h = {.type = RNO_G_GZIP, .handle.gz = f}; 

    while (rno_g_waveform_read(h, &wf) > 0)
    {
      printf("EVENT %d\n",wf.event_number); 
      for (int ichan = 0; ichan < RNO_G_NUM_RADIANT_CHANNELS; ichan++) 
      {
        printf(" CH %d:", ichan); 
        int16_t min = 32767; 
        int16_t max = -32768; 
        double sum = 0; 
        double sum2 = 0; 
        int nzeroes = 0; 
        for (int isamp = 0; isamp < wf.radiant_nsamples; isamp++) 
        {
          int16_t val = wf.radiant_waveforms[ichan][isamp]; 
          if (val> max) max = val;
          if (val< min) min = val;
          sum += val; 
          sum2 += val*val; 
          if (!val) nzeroes++; 
        }
        printf (" min: %d, max: %d, avg: %g, rms: %g, nzero: %d\n", min, max, sum/wf.radiant_nsamples, sqrt(sum2/wf.radiant_nsamples -  sum*sum/wf.radiant_nsamples/wf.radiant_nsamples), nzeroes); 

      }
    }

    gzclose(f); 
  }

}

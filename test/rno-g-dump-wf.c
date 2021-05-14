#include <stdlib.h> 
#include <zlib.h> 
#include "rno-g.h" 
#include <stdio.h> 


int main(int nargs, char ** args) 
{

  rno_g_waveform_t wf; 
  for (int i = 1; i <= nargs; i++) 
  {

    gzFile f = gzopen(args[i],"r"); 

    rno_g_file_handle_t h = {.type = RNO_G_GZIP, .handle.gz = f}; 

    while (rno_g_waveform_read(h, &wf))
    {
      rno_g_waveform_dump(stdout,&wf); 
    }

    gzclose(f); 
  }

}

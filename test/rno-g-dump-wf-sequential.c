#include <stdlib.h> 
#include <zlib.h> 
#include "rno-g.h" 
#include <stdio.h> 


int main(int nargs, char ** args) 
{

  rno_g_waveform_t wf; 

  int nseq = atoi(args[1]); 
  if (!nseq) 
  {
    printf("Usage: rno-g-dump-wf-sequential N file.dat file2.dat") ; 
    return 1; 
  }

  FILE * of = 0; 
  int file_counter = 0; 
  int iout = 0; 
  for (int i = 2; i < nargs; i++) 
  {

    gzFile f = gzopen(args[i],"r"); 

    rno_g_file_handle_t h = {.type = RNO_G_GZIP, .handle.gz = f}; 

    while (rno_g_waveform_read(h, &wf) > 0)
    {
      if (!of || iout >= nseq) 
      {
        if (of) fclose(of); 
        char fname[512]; 
        sprintf(fname,"out%05d.csv", file_counter++); 
        printf("Writing to %s\n", fname); 
        of = fopen(fname,"w"); 
        iout = 0; 
      }
      rno_g_waveform_dump(of,&wf); 
      iout++; 
    }

    gzclose(f); 
  }

  fclose(of); 
  return 0;

}

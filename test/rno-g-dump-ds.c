#include <stdlib.h> 
#include <zlib.h> 
#include "rno-g.h" 
#include <stdio.h> 


int main(int nargs, char ** args) 
{

  rno_g_daqstatus_t ds; 
  for (int i = 1; i < nargs; i++) 
  {

    gzFile f = gzopen(args[i],"r"); 
    rno_g_file_handle_t h = {.type = RNO_G_GZIP, .handle.gz = f}; 

    while (rno_g_daqstatus_read(h, &ds) >0)
    {
      rno_g_daqstatus_dump(stdout,&ds); 
    }

    gzclose(f); 
  }

}

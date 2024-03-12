#include <stdlib.h> 
#include <zlib.h> 
#include "rno-g.h" 
#include <stdio.h> 
#include <string.h>



rno_g_data_type_t filter = RNO_G_INVALID_T; 

int main(int nargs, char ** args) 
{

  rno_g_any_t any; 
  int print_version_magic = 0; 
  if (getenv("RNO_G_ENABLE_DEBUG")) print_version_magic = 1; 
  
  //check binary name
  if  (!strcmp(args[0],"rno-g-dump-hdr")) filter = RNO_G_HEADER_T;
  else if  (!strcmp(args[0],"rno-g-dump-wf")) filter = RNO_G_WAVEFORM_T;
  else if  (!strcmp(args[0],"rno-g-dump-ped")) filter = RNO_G_PEDESTAL_T;
  else if  (!strcmp(args[0],"rno-g-dump-ds")) filter = RNO_G_DAQSTATUS_T;


  for (int i = 1; i < nargs; i++) 
  {

    rno_g_file_handle_t h; 
    if (rno_g_init_handle(&h, args[i],"r"))
    {
      continue; 
    }

    while (rno_g_any_read(h, &any) > 0)
    {
      if (print_version_magic) 
      {
        printf("MAGIC: %x, VERSION: %d\n'", h.last->magic, h.last->version); 
      }


	    if (filter && any.what != filter) continue; 

      rno_g_any_dump(stdout,&any); 
    }

    rno_g_close_handle(&h);
  }

}

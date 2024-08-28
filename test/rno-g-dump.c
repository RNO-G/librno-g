#include <stdlib.h>
#include <zlib.h>
#include "rno-g.h"
#include <stdio.h>
#include <string.h>



rno_g_data_type_t filter = RNO_G_INVALID_T;

int main(int nargs, char ** args)
{

  if (nargs < 2) return 0;

  rno_g_any_t any;
  int print_version_magic = 0;

  if (getenv("RNO_G_ENABLE_DEBUG")) print_version_magic = 1;

  //check binary name
  if  (strstr(args[0],"rno-g-dump-hdr")) filter = RNO_G_HEADER_T;
  else if  (strstr(args[0],"rno-g-dump-wf")) filter = RNO_G_WAVEFORM_T;
  else if  (strstr(args[0],"rno-g-dump-ped")) filter = RNO_G_PEDESTAL_T;
  else if  (strstr(args[0],"rno-g-dump-ds")) filter = RNO_G_DAQSTATUS_T;

  int json_mode = 0;

  //get args
  int start_arg = 1;
  int nskip = 0;
  int N  = 0;


  while (args[start_arg][0] == '-')
  {
    if (!strcmp(args[start_arg],"-j") || !strcmp(args[1],"--json")) json_mode=1;
    else if (!strcmp(args[start_arg],"-s") || !strcmp(args[1],"--skip"))
    {
      start_arg++;
      nskip = atoi(args[start_arg]);
    }
    else if (!strcmp(args[start_arg],"-n") || !strcmp(args[1],"--num"))
    {

      start_arg++;
      N = atoi(args[start_arg]);
    }

    start_arg++;
  }


  if (json_mode)
  {
    printf("{ \"packets\" : [\n");
  }

  int npackets = 0;

  for (int i = start_arg; i < nargs; i++)
  {

    rno_g_file_handle_t h;
    if (rno_g_init_handle(&h, args[i],"r"))
    {
      continue;
    }



    while (rno_g_any_read(h, &any) > 0)
    {

      if (nskip)
      {
        nskip--;
        continue;
      }
      if (N && npackets >= N)
      {
        break;
      }

      if (json_mode && npackets++)
      {
        printf("\n,\n");
      }

      if (print_version_magic)
      {
        if (json_mode)
        {
          printf("{\"type\": \"magic\", \"magic\": \"%x\", \"version\": %d },\n", h.last->magic, h.last->version);

        }
        else printf("MAGIC: %x, VERSION: %d\n'", h.last->magic, h.last->version);
      }


	    if (filter && any.what != filter) continue;

      if (json_mode)
      {
        rno_g_any_dump_json(stdout,&any);
      }
      else
      {
        rno_g_any_dump(stdout,&any);
      }
    }

    rno_g_close_handle(&h);
  }

  if (json_mode) printf("]}");

   return 0;
}

#include <stdlib.h> 
#include <zlib.h> 
#include "rno-g.h" 
#include <stdio.h> 
#include <string.h>


int main(int nargs, char ** args) 
{

  rno_g_header_t hd;
  int json_mode = 0;
  if (!strcmp(args[1],"-j"))
  {
    json_mode =1;
    printf("{\n  \"files\" : [\n");
  }
  for (int i = json_mode +1; i < nargs; i++)
  {

    gzFile f = gzopen(args[i],"r");

    rno_g_file_handle_t h = {.type = RNO_G_GZIP, .handle.gz = f}; 

    if (json_mode)
    {
      if (i > 2)
      {
        printf("  ,\n");
      }

      printf(" { \n    \"filename\": \"%s\",\n", args[i]);
      printf("   \"headers\": [");
    }

    int ihd = 0;
    while (rno_g_header_read(h, &hd) > 0)
    {
      if (json_mode)
      {
        printf("%s\n    ", ihd++ ? "," : "");
        rno_g_header_dump_json(stdout, &hd);
      }
      else
      {
        rno_g_header_dump(stdout,&hd); 
      }
    }

    if (json_mode)
    {
      printf("    ]\n  }\n");
    }

    gzclose(f);
  }

  if (json_mode)
  {
    printf("  ]\n}\n");
  }

  return 0;
}

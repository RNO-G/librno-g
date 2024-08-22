#include <stdio.h>
#include <stdlib.h>
#include "flower.h"
#include <time.h>
#include <string.h>
#include <unistd.h>
#define LEN 1024

int main (int nargs, char ** args)
{
  flower_dev_t * flwr = flower_open("/dev/spidev1.0", -61);

  int N = 1;
  int force = 1;
  if (nargs > 1)
  {
    N = atoi(args[1]);
  }

  char hostname[64];

  if (nargs > 2)
  {
    force = atoi(args[2]);
  }

  uint8_t data[RNO_G_NUM_LT_CHANNELS][LEN];
//  memset(data,0x0,sizeof(data));
  uint8_t * data_ptr[RNO_G_NUM_LT_CHANNELS] = {&data[0][0], &data[1][0], &data[2][0], &data[3][0]};
  struct timespec now;

  gethostname(hostname, sizeof(hostname));

  printf("{\n\t\"hostname\" : \"%s\",\n\t\"events\" : [", hostname);
  for (int iev = 0; iev < N; iev++)
  {
    flower_buffer_clear(flwr);
    if (force) flower_force_trigger(flwr);
    clock_gettime(CLOCK_REALTIME, &now);
    int avail = 0;
    flower_waveform_metadata_t meta = {0};
    while (!avail) flower_buffer_check(flwr, &avail);
    flower_read_waveforms(flwr, LEN, data_ptr);
    flower_read_waveform_metadata(flwr,&meta);


    printf("%s\n\t\t{\n\t\t\t\"force\": %s,\n", iev > 0 ? "," : "", force ? "true" : "false");
    printf("\t\t\t\"when\": %09d.%09d,\n", (int) now.tv_sec, (int) now.tv_nsec);
    printf("\t\t\t\"meta\": { \"event_counter\": %u, \"trigger_counter\": %u, \"trigger_type\": \"%s\", \"pps_flag\": %s, \"timestamp\": %llu, \"recent_pps_timestamp\": %llu},\n",
      meta->event_counter, meta->trigger_counter, meta->trigger_type, flower_trigger_type_as_string(meta->trigger_type), meta->pps_flag ? "true" : "false",  meta->timestamp, meta->recent_pps_timestamp);

    for (int i = 0 ; i < RNO_G_NUM_LT_CHANNELS; i++)
    {
      printf("\t\t\t\"ch%d\": [",i);
      for (int j = 0; j < LEN; j++)
      {
        printf("%d", ((int)data_ptr[i][j]) - 128);
        if (j < LEN-1)
          printf(",");

      }
      printf("]%s\n", i < RNO_G_NUM_LT_CHANNELS-1 ? "," : "");
    }
    printf("\t\t}");
  }
  printf("\n\t]\n}");

  flower_close(flwr);
}

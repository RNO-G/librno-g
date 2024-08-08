#define _GNU_SOURCE
#include "rno-g.h" 
#include <stdio.h> 
#include <stddef.h> 
#include <zlib.h> 
#include <stdlib.h>
#include <string.h> 
#include <inttypes.h>
#include <time.h> 

/* IO functions. 
 *
 * All assumes little-endian and similar ABI to Linux/gcc. 
 *
 */

#define HEADER_VER 1
#define WF_VER 4 
#define PED_VER 3 
#define DAQSTATUS_VER 5 

#define HEADER_MAGIC 0xead1 
#define WAVEFORM_MAGIC 0xafd1 
#define PEDESTAL_MAGIC 0x57a1 
#define DAQSTATUS_MAGIC 0xdacc 


typedef struct io_header
{
  uint16_t magic; 
  uint16_t version;
} io_header_t; 


//This is a bit silly, since we usually will save as gzip and have a checksum anyway
static uint32_t cksum(uint32_t start_cksum, int N, const uint8_t *data) 
{
  return adler32(start_cksum, data,N); 
}


static int do_read(rno_g_file_handle_t h, int N, void *data, uint32_t * sum) 
{
  int rd = 0;
  switch  (h.type)
  {
    case RNO_G_RAW: 
      rd = fread(data,1,N,h.handle.raw); 
      break;
    case RNO_G_GZIP: 
      rd = gzread(h.handle.gz, data,N); 
      break;
    default: 
      return 0; 
  }

  if (sum) *sum = cksum(*sum,rd,data); 
  return rd; 
}

static int read_io_header(io_header_t * hd, rno_g_file_handle_t h)
{

  int rd = do_read(h, sizeof (*hd), hd, 0); 

  if (rd == sizeof(*hd) && h.last) 
  {
    h.last->version = hd->version;
    h.last->magic = hd->magic;
  }

  return rd; 
}

static uint32_t init_cksum =1; 

static int do_write(rno_g_file_handle_t h, int N, const void *data, uint32_t * sum) 
{

  if (sum) *sum = cksum(*sum,N,data); 

  switch  (h.type)
  {
    case RNO_G_RAW: 
      return fwrite(data,N,1,h.handle.raw); 
    case RNO_G_GZIP: 
      return gzwrite(h.handle.gz, data,N); 
    default: 
      return 0; 
  }
}







//magic, version, checksum, then we dump the struct
int rno_g_header_write(rno_g_file_handle_t h, const rno_g_header_t*header)
{
  int N = sizeof(*header); 
  io_header_t hd  = { .magic = HEADER_MAGIC, .version = HEADER_VER} ; 
  do_write(h,sizeof(hd),&hd,0);
  uint32_t sum = init_cksum; 
  int wr = do_write(h, N, header, &sum); 
  do_write(h,sizeof(sum), &sum,0); 
  return wr; 
}


int rno_g_header_dump(FILE *f, const rno_g_header_t *header) 
{
  //TODO 
  int ret = 0;
  ret+=fprintf(f,"RUN:, %d, EVENT:, %d\n", header->run_number, header->event_number); 
  ret+=fprintf(f, "   PPS_COUNT:, %d, SYSCLK:, %u, TIME_SINCE_START:,%f\n", header->pps_count, header->sys_clk, header->pps_count + ((double)header->sys_clk - header->sysclk_last_pps) / ( header->sysclk_last_pps - header->sysclk_last_last_pps));
  ret+=fprintf(f, "   READOUT_TIME:, %d.%09d, ELAPSED: %d\n", header->readout_time_secs, header->readout_time_nsecs, header->readout_elapsed_nsecs);
  ret+=fprintf(f, "   START WINDOWS: "); 
  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
  {
    fprintf(f," %u/%u ", header->radiant_start_windows[i][0], header->radiant_start_windows[i][1]); 
  }
  ret+=fprintf(f, "\n"); 
  ret+=fprintf(f, " TRIGTYPE: %s %s %s %s %s %s | RAWTRIGINFO: 0x%x| RAWSTATUS: 0x%x\n", 
      header->trigger_type & RNO_G_TRIGGER_SOFT ? "SOFT":"",
      header->trigger_type & RNO_G_TRIGGER_PPS ? "PPS":"",
      header->trigger_type & RNO_G_TRIGGER_RF_LT_SIMPLE ? "RFLT":"",
      (header->trigger_type & (RNO_G_TRIGGER_RF_RADIANT0 | RNO_G_TRIGGER_RF_RADIANTX)) == (RNO_G_TRIGGER_RF_RADIANT0 | RNO_G_TRIGGER_RF_RADIANTX)? "RFRAD0":"",
      (header->trigger_type & (RNO_G_TRIGGER_RF_RADIANT1 | RNO_G_TRIGGER_RF_RADIANTX)) == (RNO_G_TRIGGER_RF_RADIANT1 | RNO_G_TRIGGER_RF_RADIANTX)? "RFRAD1":"", 
      (header->trigger_type & (RNO_G_TRIGGER_RF_RADIANTX  | RNO_G_TRIGGER_RF_RADIANT0 | RNO_G_TRIGGER_RF_RADIANT1)) == (RNO_G_TRIGGER_RF_RADIANTX ) ? "RFRAD?":"", 
      header->raw_tinfo, header->raw_evstatus
      ); 
  return ret; 
}

int rno_g_header_dump_json(FILE *f, const rno_g_header_t *h)
{
  int ret = fprintf(f,
        "{ \"station_number\": %hhu, "
           "\"event_number\": %u, "
           "\"run_number\": %u, "
           "\"sys_clk\": %u, "
           "\"sysclk_last_pps\": %u, "
           "\"sysclk_last_last_pps\": %u, "
           "\"pps_count\": %u, "
           "\"readout_time\": %f, "
           "\"trigger_type\": \"%s\", "
           "\"radiant_start_windows\" : [",
           h->station_number, h->event_number, h->run_number, h->sys_clk,
           h->sysclk_last_pps, h->sysclk_last_last_pps, h->pps_count,
           h->readout_time_secs + 1e-9 * h->readout_time_nsecs,
           rno_g_trigger_type_to_string(h->trigger_type)
   );

  for (int ichan = 0; ichan < RNO_G_NUM_RADIANT_CHANNELS; ichan++)
  {
    ret += fprintf(f,"[%hhu,%hhu]%c", h->radiant_start_windows[ichan][0], h->radiant_start_windows[ichan][1], ichan == RNO_G_NUM_RADIANT_CHANNELS -1 ? ']' : ',');
  }
  ret += fprintf(f,", \"raw_tinfo\": \"0x%x\", \"raw_evstatus\": \"0x%x\"}", h->raw_tinfo, h->raw_evstatus);

  return ret;
}

typedef struct rno_g_header_v0
{
  uint32_t event_number;  //!< Event number (per run, 0-indexed) 
  uint32_t trigger_number;//!< Trigger  number (per run, 0-indexed), including triggers not read out due to deadtime
  uint32_t run_number;    //!< Run number , assigned at startup 

  uint32_t trigger_mask;  //!< Which channels (or beams?) caused the trigger
  uint32_t trigger_value; //!< Relevant for LT trigger only, probably. Something like the beam power? 
  uint32_t sys_clk;  //!< Trigger time tag  (number of clock cycles since start of run or since PPS? If since start of run, may need to be 64-bit) 
  uint32_t pps_count;     //!< Number of PPS's since start of run
  uint32_t readout_time_secs; // !< readout START time in secondpart
  uint32_t readout_time_nsecs; // !< readout START time, nanosecond part 
  uint32_t readout_elapsed_nsecs; // How long it took to do the readout syscall 
  uint32_t sysclk_last_pps;  //!< sysclk time at last PPS
  uint32_t sysclk_last_last_pps; //!< sysclk time at last last PPS
  uint32_t raw_tinfo;       //!< the raw trigger info. To  be figured out. 
  uint32_t raw_evstatus;    //!< the raw event status flags. To be figured out. 

  uint8_t station_number; //!< The station number. 

  /** Trigger type. See rno_g_trigger_type_t  Or-able */ 
  uint8_t trigger_type; 

  /** Various flags for the event. See rno_g_flags_t orable */ 
  uint8_t flags; 
  uint8_t pretrigger_windows; //!< Number of pretrigger windows? 
  uint8_t radiant_start_windows[RNO_G_NUM_RADIANT_CHANNELS][2]; //!<this encodes buffer number too. There are two of these because of 2048-sample readout works. The second one will be 0xff (255) if we are in 1024-mode. 
  uint16_t radiant_nsamples; //!< Number of samples per channel in RADIANT board (could just keep this in waveform if we wanted)
  uint16_t lt_nsamples; //!< Number of samples per channel in low-threshold board  (could just keep this in waveform if we wanted)
} rno_g_header_v0_t; 





static int real_rno_g_header_read(rno_g_file_handle_t h, rno_g_header_t * header, int version)
{

  uint32_t sum = init_cksum; 
  uint32_t wanted_sum = 0; 
  int rd = 0; 
  int rdsum = 0;
  // here we handle converting to the newest kind of header
  switch (version) 
  {
    case 0: 
    {
      //I THINK we can just get away with zeroing and reading hdv0 amount
      memset(header,0, sizeof(*header)); 
      rd = do_read(h,sizeof(rno_g_header_v0_t),header, &sum); 
      break; 
    }
    case HEADER_VER:
    {
      rd = do_read(h,sizeof(*header),header, &sum); 
      break; 
    }
    default: 
      fprintf(stderr,"Unknown header version %d\n", version); 
      return 0; 
  }

  rdsum = do_read(h,sizeof(wanted_sum), &wanted_sum,0); 
  if (!rdsum || sum!=wanted_sum) 
  {
    fprintf(stderr,"Checksum error. Got %x, wnated %x\n", sum,wanted_sum); 
    return -1; 
  }

  return rd; 
}

int rno_g_header_read(rno_g_file_handle_t h, rno_g_header_t *header)
{
  io_header_t hd; 
  int rd = read_io_header(&hd, h); 
  if (!rd) return 0; 

  if (hd.magic != HEADER_MAGIC)
  {
    //this is not a header! 
    fprintf(stderr,"Wrong magic! Got %hx, expected %hx\n", hd.magic, HEADER_MAGIC); 
    return -1; 
  }

  return real_rno_g_header_read(h, header, hd.version);
}

 //WARNING: watch out for this struct changing! 
#define N_BEFORE_DATA  ( offsetof(rno_g_waveform_t,lt_nsamples) + sizeof(((rno_g_waveform_t*) 0)->lt_nsamples)) 

//magic, version, checksum, then we dump the struct
int rno_g_waveform_write(rno_g_file_handle_t h, const rno_g_waveform_t *wf)
{
  io_header_t hd  = { .magic = WAVEFORM_MAGIC, .version = WF_VER };
  uint32_t sum = init_cksum; 
  do_write(h,sizeof(hd),&hd,0);
  int wr = 0;
  //write up to the waveforms 
  wr+=do_write(h, N_BEFORE_DATA, wf, &sum); 

  for (int ichan = 0; ichan < RNO_G_NUM_RADIANT_CHANNELS; ichan++)
  {
    // 2 since 16-bit data 
    wr += do_write(h, 2*wf->radiant_nsamples, wf->radiant_waveforms[ichan],&sum); 
  }

  for (int ichan = 0; ichan < RNO_G_NUM_LT_CHANNELS; ichan++)
  {
    wr += do_write(h, wf->lt_nsamples, wf->lt_waveforms[ichan],&sum); 
  }

  wr += do_write(h, 2, &wf->radiant_sampling_rate, &sum);

  for (int ichan = 0; ichan < RNO_G_NUM_RADIANT_CHANNELS; ichan++)
  {
    wr += do_write(h, 1, &wf->digitizer_readout_delay[ichan], &sum); 
  }

  wr += do_write(h, 1, &wf->station, &sum);


  do_write(h, sizeof(sum),&sum,0); 

  return wr;
}

//V1, V2, and V3 all have the same structure - but correspond to bug fixes/firmware updates
typedef struct rno_g_waveform_v1
{
  uint32_t event_number; //!< For matching
  uint32_t run_number;   //!< For matching
  uint16_t radiant_nsamples; //!< Number of samples per channel for RADIANT
  uint16_t lt_nsamples; //!< Number of samples per channel for lowthresh
  int16_t radiant_waveforms[RNO_G_NUM_RADIANT_CHANNELS][RNO_G_MAX_RADIANT_NSAMPLES]; //unrolled. 
  uint8_t lt_waveforms[RNO_G_NUM_LT_CHANNELS][RNO_G_MAX_LT_NSAMPLES]; // 8-bit digitizer 
  uint8_t station; 
} rno_g_waveform_v1_t; 

typedef struct rno_g_waveform_v4
{
  uint32_t event_number; //!< For matching
  uint32_t run_number;   //!< For matching
  uint16_t radiant_nsamples; //!< Number of samples per channel for RADIANT
  uint16_t lt_nsamples; //!< Number of samples per channel for lowthresh
  int16_t radiant_waveforms[RNO_G_NUM_RADIANT_CHANNELS][RNO_G_MAX_RADIANT_NSAMPLES]; //unrolled. 
  uint8_t lt_waveforms[RNO_G_NUM_LT_CHANNELS][RNO_G_MAX_LT_NSAMPLES]; // 8-bit digitizer 
  uint16_t sampling_rate;  
  uint8_t digitizer_readout_delay[24];
  uint8_t station;

  //radiant_readout_delay_t radiant_readout_delays;

} rno_g_waveform_v4_t; 


static int real_rno_g_waveform_read(rno_g_file_handle_t h, rno_g_waveform_t * wf, int version)
{
  uint32_t sum = init_cksum; 
  uint32_t wanted_sum = 0; 
  int rdsum;
  int ichan = 0; 
  int rd = 0; 
  // here we handle converting to the newest kind of waveform
  switch (version) 
  {
    case 1:
    case 2:
    case 3: //these might need to be rewritten to make the cases useful - though it means repeated blocks of code
      {
        rd = do_read(h,N_BEFORE_DATA,wf,&sum); 
        for (ichan = 0; ichan < RNO_G_NUM_RADIANT_CHANNELS; ichan++)
        {
          rd+= do_read(h,2*wf->radiant_nsamples, wf->radiant_waveforms[ichan], &sum);
          if ( version < 3) 
          {
            //fix unwrapping bug
            uint16_t tmp[128]; 
            memcpy(tmp, wf->radiant_waveforms[ichan], 128*2); 
            memmove(wf->radiant_waveforms[ichan], wf->radiant_waveforms[ichan]+128, (1024-128)*2); 
            memcpy(wf->radiant_waveforms[ichan]+1024-128,tmp,128*2); 
            memcpy(tmp, wf->radiant_waveforms[ichan]+1024, 128*2); 
            memmove(wf->radiant_waveforms[ichan]+1024, wf->radiant_waveforms[ichan]+128+1024, (1024-128)*2); 
            memcpy(wf->radiant_waveforms[ichan]+2048-128,tmp,128*2); 
          }
        }
         for (ichan = 0; ichan < RNO_G_NUM_LT_CHANNELS; ichan++)
        {
          rd+= do_read(h,wf->lt_nsamples, wf->lt_waveforms[ichan], &sum);
        }

        if (version > 1) 
        {
          rd+= do_read(h,sizeof(wf->station), &wf->station,&sum); 
        }
        else
        {
          wf->station =0; 
        }

        if(version<4)
        {
          
          for(ichan=0;ichan<24;ichan++)
          {
            wf->digitizer_readout_delay[ichan]=0;
          }
          //wf->radiant_readout_delays.rf0_delay=0;
          //wf->radiant_readout_delays.rf1_delay=0;
          //wf->radiant_readout_delays.rf0_delay_mask=0;
          //wf->radiant_readout_delays.rf0_delay_mask=0;
          wf->radiant_sampling_rate=3200; //the change happened to new versions
        }
        
        rdsum = do_read(h, sizeof(wanted_sum),&wanted_sum,0); 


        if (!rdsum || sum!=wanted_sum) 
        {
          fprintf(stderr,"Checksum error. Got %x, wanted %x\n", sum,wanted_sum); 
          return -1; 
        }
        return rd+rdsum; 
      }
      case 4:
      {
        rd = do_read(h,N_BEFORE_DATA,wf,&sum); 
        for (ichan = 0; ichan < RNO_G_NUM_RADIANT_CHANNELS; ichan++)
        {
          rd+= do_read(h,2*wf->radiant_nsamples, wf->radiant_waveforms[ichan], &sum);
          if (version < 3) 
          {
            //fix unwrapping bug
            uint16_t tmp[128]; 
            memcpy(tmp, wf->radiant_waveforms[ichan], 128*2); 
            memmove(wf->radiant_waveforms[ichan], wf->radiant_waveforms[ichan]+128, (1024-128)*2); 
            memcpy(wf->radiant_waveforms[ichan]+1024-128,tmp,128*2); 
            memcpy(tmp, wf->radiant_waveforms[ichan]+1024, 128*2); 
            memmove(wf->radiant_waveforms[ichan]+1024, wf->radiant_waveforms[ichan]+128+1024, (1024-128)*2); 
            memcpy(wf->radiant_waveforms[ichan]+2048-128,tmp,128*2); 
          }
        }
         for (ichan = 0; ichan < RNO_G_NUM_LT_CHANNELS; ichan++)
        {
          rd+= do_read(h,wf->lt_nsamples, wf->lt_waveforms[ichan], &sum);
        }


        if(version>3)
        {
          rd+= do_read(h,sizeof(wf->radiant_sampling_rate), &wf->radiant_sampling_rate,&sum);

          for(ichan=0;ichan<24;ichan++)
          {
            rd+= do_read(h,sizeof(wf->digitizer_readout_delay[0]), &wf->digitizer_readout_delay[ichan],&sum); 
          }

        }
        else
        {
          for(ichan=0;ichan<24;ichan++)
          {
            wf->digitizer_readout_delay[ichan]=0;
          }

          wf->radiant_sampling_rate=3200;
        }
        
        if (version > 1) 
        {
          rd+= do_read(h,sizeof(wf->station), &wf->station,&sum); 
        }
        else
        {
          wf->station =0; 
        }
        
        rdsum = do_read(h, sizeof(wanted_sum),&wanted_sum,0); 

        if (!rdsum || sum!=wanted_sum) 
        {
          fprintf(stderr,"Checksum error. Got %x, wanted %x\n", sum,wanted_sum); 
          return -1; 
        }
        return rd+rdsum; 
      }
    default: 
      fprintf(stderr,"Unknown waveform version %d\n", version); 
  }
  return 0; 
}

int rno_g_waveform_read(rno_g_file_handle_t h, rno_g_waveform_t *wf)
{
  io_header_t hd; 

  int rd = read_io_header(&hd, h); 
  if (!rd) return 0; 

  if (hd.magic != WAVEFORM_MAGIC)
  {
    //this is not a waveform! 
    fprintf(stderr,"Wrong magic! Got %hx, expected %hx\n", hd.magic, WAVEFORM_MAGIC); 
    return -1; 
  }

  return real_rno_g_waveform_read(h, wf, hd.version);
}




int rno_g_init_handle(rno_g_file_handle_t * h, const char * name, const char * mode) 
{

  h->last = malloc(sizeof(int) *2); 
  h->last->version = -1; 
  h->last->magic = -1; 
  //check for gz ending
  char * dot = strrchr(name,'.'); 
  h->handle.raw = NULL;

  if (dot && !strcasecmp(dot, ".gz"))
  {
    h->type = RNO_G_GZIP; 
    h->handle.gz = gzopen(name,mode); 
    if (!h->handle.gz) 
    {
      fprintf(stderr,"Unable to open %s with mode %s\n", name, mode); 

    }
    return h->handle.gz==0;
  }

  h->type = RNO_G_RAW; 
#ifdef ON_BBB
  h->handle.raw = fopen64(name,mode); 
#else
  h->handle.raw = fopen(name,mode); 
#endif
  if (!h->handle.raw) 
  {
     fprintf(stderr,"Unable to open %s with mode %s\n", name, mode); 
  }
  return h->handle.raw == 0; 
} 

int rno_g_close_handle(rno_g_file_handle_t *h)  
{
  if (h->type == RNO_G_GZIP) 
  {
    if(h->handle.gz) 
      return gzclose(h->handle.gz); 
    else return 1; 
  }

  else if (h->type == RNO_G_RAW) 
  {
    if (h->handle.raw) 
      return fclose(h->handle.raw); 
    else return 1; 
  }

  if (h->last) 
    free (h->last); 

  return 0; 
}

int rno_g_waveform_dump(FILE * f, const rno_g_waveform_t * waveform) 
{
  //TODO 
  int ret=0;

  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
  {
    for (int j = 0; j < waveform->radiant_nsamples; j++) 
    {
      ret+=fprintf(f, "%hd%c", waveform->radiant_waveforms[i][j], j == waveform->radiant_nsamples-1?'\n':','); 
    }
  }

  return ret; 
}

int rno_g_waveform_dump_json(FILE *f, const rno_g_waveform_t * w)
{
  int ret = fprintf(f, "{ \"station\": %hhu, \"run_number\": %u, \"event_number\": %u, \"radiant_nsamples\": %hu, \"radiant_sampling_rate\": %hu, \"radiant_waveforms\": [",
      w->station, w->run_number, w->event_number, w->radiant_nsamples, w->radiant_sampling_rate);

  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
  {
    for (int j = 0; j < w->radiant_nsamples; j++)
    {
      ret+=fprintf(f,"%c%hd", j == 0 ? '[' : ',', w->radiant_waveforms[i][j]);
    }
    ret+=fprintf(f,"]%c", i == RNO_G_NUM_RADIANT_CHANNELS -1 ? ']' : ',');
  }

  ret+= fprintf(f,", \"digitizer_readout_delay\": ");

  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
    ret += fprintf(f,"%c%hhu", i == 0 ? '[' : ',', w->digitizer_readout_delay[i]);


  ret += fprintf(f,"] }");

  return ret;
}



int rno_g_pedestal_dump(FILE *f, const rno_g_pedestal_t *pedestal) 
{
  char ctime_buf[32] = {0} ; 
  time_t when = pedestal->when;
  ctime_r(&when, ctime_buf);
  int ret = fprintf(f,"PEDESTAL of %d events recorded at %s\n", pedestal->nevents,ctime_buf); 
  ret += fprintf(f,"VBIAS,%d,%d\n", pedestal->vbias[0], pedestal->vbias[1]); 

  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
  {
    ret += fprintf(f,"CH%d,", i); 
    for (int j = 0; j < RNO_G_PEDESTAL_NSAMPLES; j++) 
    {
      ret+=fprintf(f, "%hd%c", pedestal->pedestals[i][j], j ==RNO_G_PEDESTAL_NSAMPLES-1?'\n':','); 
    }
  }
  return ret; 
}

int rno_g_pedestal_dump_json(FILE *f, const rno_g_pedestal_t *pd)
{
  int ret = fprintf(f,"{ \"station\": %hhu, \"when\": %u, \"nevents\": %u, \"mask\": \"0x%x\", \"vbias\": [ %hhu, %hhu], \"pedestals\": [", 
      pd->station, pd->when, pd->nevents, pd->mask, pd->vbias[0], pd->vbias[1]); 

  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++)
  {
    for (int j = 0; j < RNO_G_PEDESTAL_NSAMPLES; j++)
    {
    ret += fprintf(f,"%c%hu", i == 0 ? '[' : ',', pd->pedestals[i][j]);
    }

    ret+=fprintf(f,"]%c", i == RNO_G_NUM_RADIANT_CHANNELS -1 ? ']' : ',');
  }

  ret += fprintf(f,"] }"); 
  return ret;
}

// TODO: could only read/write unmasked if we wanted to... 

int rno_g_pedestal_write(rno_g_file_handle_t h, const rno_g_pedestal_t * pd) 
{
  io_header_t hd = {.magic = PEDESTAL_MAGIC, .version = PED_VER }; 
  uint32_t sum = init_cksum; 
  do_write(h, sizeof(hd), &hd,0); 
  int wr = do_write(h, sizeof(rno_g_pedestal_t), pd,&sum); 
  do_write(h, sizeof(sum),&sum,0); 
  return wr; 
}

typedef struct rno_g_pedestal_v0
{
  uint32_t when; 
  uint16_t nevents; 
  uint32_t mask : 24; 
  uint8_t flags; //TBD 
  uint16_t pedestals[RNO_G_NUM_RADIANT_CHANNELS][RNO_G_PEDESTAL_NSAMPLES]; 
} rno_g_pedestal_v0_t; 

typedef struct rno_g_pedestal_v1
{
  uint32_t when; 
  uint32_t nevents; 
  uint32_t mask : 24; 
  uint8_t flags; //TBD 
  int16_t vbias[2];  //signed so that we can have -1 as unknown
  uint16_t pedestals[RNO_G_NUM_RADIANT_CHANNELS][RNO_G_PEDESTAL_NSAMPLES]; 
} rno_g_pedestal_v1_t; 

typedef struct rno_g_pedestal_v2
{
  uint32_t when; 
  uint32_t nevents; 
  uint32_t mask : 24; 
  uint8_t flags; //TBD 
  int16_t vbias[2];  //signed so that we can have -1 as unknown
  uint16_t pedestals[RNO_G_NUM_RADIANT_CHANNELS][RNO_G_PEDESTAL_NSAMPLES]; 
  uint8_t station; 
} rno_g_pedestal_v2_t; 



static int real_rno_g_pedestal_read(rno_g_file_handle_t h, rno_g_pedestal_t *pd, int version)
{
  uint32_t sum = init_cksum; 
  uint32_t wanted_sum = 0; 
  int rdsum=0;
  int rd = 0; 

  switch (version) 
  {
    case 0: 
    {
      rno_g_pedestal_v0_t pdv0; 
      rd += do_read(h, sizeof(rno_g_pedestal_v0_t), &pdv0, &sum); 
      pd->when = pdv0.when; 
      pd->nevents = pdv0.nevents; 
      pd->mask = pdv0.mask; 
      pd->flags = pdv0.flags; 
      pd->vbias[0]=-1;
      pd->vbias[1]=-1;
      pd->station = 0;
      pd->run=-1; 
      memcpy(pd->pedestals, pdv0.pedestals, sizeof(pd->pedestals)); 
      break; 
    }
    case 1:
    {
      rno_g_pedestal_v1_t pdv1; 
      rd += do_read(h, sizeof(rno_g_pedestal_v1_t), &pdv1, &sum); 
      pd->when = pdv1.when; 
      pd->nevents = pdv1.nevents; 
      pd->mask = pdv1.mask; 
      pd->flags = pdv1.flags; 
      pd->vbias[0]= pdv1.vbias[0];
      pd->vbias[1]= pdv1.vbias[1]; 
      pd->station = 0; 
      pd->run=-1;
      memcpy(pd->pedestals, pdv1.pedestals, sizeof(pd->pedestals)); 
      break;
    }
    case 2:
    {
      rno_g_pedestal_v2_t pdv2; 
      rd += do_read(h, sizeof(rno_g_pedestal_v2_t), &pdv2, &sum); 
      pd->when = pdv2.when; 
      pd->nevents = pdv2.nevents; 
      pd->mask = pdv2.mask; 
      pd->flags = pdv2.flags; 
      pd->vbias[0]= pdv2.vbias[0];
      pd->vbias[1]= pdv2.vbias[1]; 
      pd->station = pdv2.station; 
      pd->run=-1; 
      memcpy(pd->pedestals, pdv2.pedestals, sizeof(pd->pedestals)); 
      break;
    }

    case PED_VER: 
      rd += do_read(h, sizeof(rno_g_pedestal_t), pd, &sum); 
      break; 
    default: 
      fprintf(stderr,"Unknown pedestal version %d\n", version); 
      return 0; 
  }

  rdsum = do_read(h, sizeof(wanted_sum),&wanted_sum,0); 
  if (!rdsum || sum != wanted_sum) 
  {
     fprintf(stderr,"Checksum error. Got %x, wanted %x\n", sum,wanted_sum); 
     return -1; 
  }

  return rd; 
}
int rno_g_pedestal_read(rno_g_file_handle_t h, rno_g_pedestal_t * pd) 
{
  io_header_t hd; 
  int rd = read_io_header(&hd, h);

  if (!rd) return 0; 



  if (hd.magic != PEDESTAL_MAGIC) 
  {
    fprintf(stderr,"Wrong magic! Got %hx, expected %hx\n", hd.magic, PEDESTAL_MAGIC); 
    return -1; 
  }

  return real_rno_g_pedestal_read(h, pd, hd.version);
}




int rno_g_daqstatus_dump(FILE *f, const rno_g_daqstatus_t* ds) 
{
  return rno_g_daqstatus_dump_radiant(f,ds) + rno_g_daqstatus_dump_flower(f,ds) + rno_g_daqstatus_dump_calpulser(f,ds); 
}

int rno_g_daqstatus_json(FILE *f, const rno_g_daqstatus_t* ds) 
{
  int ret = fprintf(f,"{ \"radiant\": ");
  ret += rno_g_daqstatus_dump_radiant_json(f,ds);
  ret += fprintf(f,", \"flower\": ");
  ret += rno_g_daqstatus_dump_flower_json(f,ds);
  ret += fprintf(f,", \"calpulser\": ");
  ret += rno_g_daqstatus_dump_calpulser_json(f,ds);
  ret += fprintf(f,"}"); 
  return ret; 
}

static const char * calpulse_mode_strings[] ={ "NONE", "PULSE","VCO","VCO2"}; 
static const char * calpulse_output_strings[] ={ "NONE","COAX","FIBER0","FIBER1"}; 
int rno_g_daqstatus_dump_calpulser(FILE *f, const rno_g_daqstatus_t * ds) 
{
  int ret = 0;

  // no calpulser was actually filled in if it's 0
  if (ds->cal.rev=='\0') 
  {
    return 0; 
  }

  if (! ds->cal.enabled) 
  {
    ret+= fprintf(f, "================================\n"); 
    ret+= fprintf(f, "===== CALPULSER REV%c IS OFF====\n)", ds->cal.rev); 
    ret+= fprintf(f, "================================\n"); 
    return ret; 
  }
  ret+= fprintf(f, "===== CALPULSER REV%c IS ENABLED=====\n", ds->cal.rev); 
  ret+= fprintf(f, "   Temperature: %0.3f C\n", ds->cal.T_times_16/16.); 
  ret+= fprintf(f, "   Mode: %s\n", calpulse_mode_strings[ds->cal.mode]); 
  ret+= fprintf(f,"    Attenuation: %g\n", ds->cal.atten_times_2/2.); 
  ret+= fprintf(f,"    Output: %s\n", calpulse_output_strings[ds->cal.out]); 
  return ret; 
}

int rno_g_daqstatus_dump_calpulser_json(FILE *f, const rno_g_daqstatus_t *ds)
{
  if (!ds->cal.enabled)
  {
    return fprintf(f,"{ \"rev\": \"%c\", \"enabled\": false }", ds->cal.rev);
  }
  else
  {
    return fprintf(f,"{ \"rev\": \"%c\", \"enabled\" true, \"temperature\": %0.3f,"
                   " \"mode\": \"%s\", \"attenuation\": %f, \"output\": \"%s\" }",
                   ds->cal.rev, ds->cal.T_times_16/16., calpulse_mode_strings[ds->cal.mode], 
                   ds->cal.atten_times_2/2., calpulse_output_strings[ds->cal.out]);
  }
}


int rno_g_daqstatus_dump_radiant(FILE*f, const rno_g_daqstatus_t *ds) 
{
  int when_radiant = ds->when_radiant; 
  int radiant_ns = (ds->when_radiant-when_radiant)*1e9; 
  struct tm when_tm_radiant; 
  time_t when_radiant_t = when_radiant;
  gmtime_r((time_t*)&when_radiant_t, &when_tm_radiant); 
  int ret=fprintf(f,  "========== RADIANT================\n"); 
  ret    += fprintf(f,"DAQSTATUS, radiant_period="); 
  if (!ds->radiant_scaler_period) 
  {
    ret+=fprintf(f,"PPS"); 
  }
  else
  {
    ret+=fprintf(f,"%f s", ds->radiant_scaler_period);  
  }
  ret += fprintf(f,", recorded at %04d-%02d-%02d %02d:%02d:%02d.%09dZ\n", 
                 when_tm_radiant.tm_year + 1900, 1+when_tm_radiant.tm_mon, when_tm_radiant.tm_mday, when_tm_radiant.tm_hour, 
                 when_tm_radiant.tm_min, when_tm_radiant.tm_sec,  radiant_ns); 
  ret+=fprintf(f,"Voltages: V1: %f, V1.8: %f, V2.5: %f, VLeftMon: %f, VRightMon: %f\n", 
      3.3*ds->radiant_voltages.V_1_0/65535, 
      3.3*ds->radiant_voltages.V_1_8/65535, 
      3.3*ds->radiant_voltages.V_2_5/65535, 
      3.3*ds->radiant_voltages.V_LeftMon/65535, 
      3.3*ds->radiant_voltages.V_RightMon/65535); 

  ret+=fprintf(f,  "==CH==THRESH(V)==SCALER==PRESCALER\n"); 

  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
  {
    if (ds->radiant_thresholds[i] == 0xffffffff) 
    {
      ret+=fprintf(f,  " %02d | ?????? |  %05u  | %03u\n", i, ds->radiant_scalers[i], ds->radiant_prescalers[i]+1); 
    }
    else
    {
      ret+=fprintf(f,  " %02d | %0.4f  |  %05u  | %03u\n", i, ds->radiant_thresholds[i] *2.5/16777215, ds->radiant_scalers[i], ds->radiant_prescalers[i]+1); 
    }
  }

  return ret;
}

int rno_g_daqstatus_dump_radiant_json(FILE *f, const rno_g_daqstatus_t  *ds)  
{
  int ret = 0;
  ret += fprintf(f," { \"when\": %f, \"voltages\": "
                "{ \"V1\": %f, \"V1.8\": %f, \"V2.5\", %f, \"VLeftMon\": %f, \"VRightMon: %f}"
                ", \"period\" : ",  ds->when_radiant,
                ds->radiant_voltages.V_1_0/65535.,
                ds->radiant_voltages.V_1_8/65535.,
                ds->radiant_voltages.V_2_5/65535.,
                ds->radiant_voltages.V_LeftMon/65535.,
                ds->radiant_voltages.V_RightMon/65535.
                );
  if ( !ds->radiant_scaler_period)
    ret += fprintf(f,"\"PPS\",");
  else
    ret += fprintf(f,"%f,", ds->radiant_scaler_period);

  ret+= fprintf(f,"\"thresholds\": [");

  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++)
  {
    if (ds->radiant_thresholds[i] == 0xffffffff)
      ret +=fprintf(f,"null");
    else
      ret +=fprintf(f,"%0.4f",  ds->radiant_thresholds[i] * 2.5/16777215);

    if (i == RNO_G_NUM_RADIANT_CHANNELS -1)
      ret +=fprintf(f,"]");
    else
      ret += fprintf(f,",");
  }

  ret+= fprintf(f,", \"scalers\": [");
  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++)
  {
    ret += fprintf(f,"%hu%c", ds->radiant_scalers[i], i == RNO_G_NUM_RADIANT_CHANNELS -1 ? ']': ',');
  }
  ret+= fprintf(f,", \"prescalers\": [");
  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++)
  {
    ret += fprintf(f,"%hu%c", ds->radiant_prescalers[i], i == RNO_G_NUM_RADIANT_CHANNELS -1 ? ']': ',');
  }

  ret +=fprintf(f,"}");
  return ret;
}


int rno_g_daqstatus_dump_flower(FILE  *f, const rno_g_daqstatus_t * ds) 
{
  int ret = 0; 
  int when_lt = ds->when_lt; 
  int lt_ns = (ds->when_lt - when_lt); 
  struct tm when_tm_lt; 
  time_t when_lt_t = when_lt;
  gmtime_r((time_t*)&when_lt_t, &when_tm_lt); 

  ret+=fprintf(f,  "============FLOWER=============\n"); 
  ret += fprintf(f,", recorded at %04d-%02d-%02d %02d:%02d:%02d.%09dZ\n", 
                 when_tm_lt.tm_year + 1900, 1+when_tm_lt.tm_mon, when_tm_lt.tm_mday, when_tm_lt.tm_hour, 
                 when_tm_lt.tm_min, when_tm_lt.tm_sec,  lt_ns); 
  uint64_t ncycles = ds->lt_scalers.ncycles; 
  ret+=fprintf(f,  "  ncycles: %" PRIu64 " , delay cycle counter: %" PRIu64 ", counter: %hu\n", ncycles, ds->lt_scalers.cycle_counter, ds->lt_scalers.scaler_counter_1Hz); 
  ret+=fprintf(f,  "==CH==SERVO_THR=TRIG_THR=SERVO_SCAL_1HZ==SERVO_SCAL_100HZ==SERVO_SCAL_GTE==TRIG_SCAL_1HZ==TRIG_SCAL_100Hz=TRIG_SCAL_GTE\n"); 
  for (int i = 0; i < RNO_G_NUM_LT_CHANNELS; i++)
  {
    ret+=fprintf(f," %d   | %03d     | %03d   |      %04d   |      %04d         |       %04d     |      %04d    |      %04d      |   %04d        \n",
                     i, ds->lt_servo_thresholds[i], ds->lt_trigger_thresholds[i],
                     ds->lt_scalers.s_1Hz.servo_per_chan[i], ds->lt_scalers.s_100Hz.servo_per_chan[i],ds->lt_scalers.s_1Hz_gated.servo_per_chan[i],
                     ds->lt_scalers.s_1Hz.trig_per_chan[i], ds->lt_scalers.s_100Hz.trig_per_chan[i],ds->lt_scalers.s_1Hz_gated.trig_per_chan[i]);
  }

  ret += fprintf(f,"coinc|         |       |      %04d   |      %04d         |       %04d     |      %04d    |      %04d      |   %04d        \n",
                     ds->lt_scalers.s_1Hz.servo_coinc, ds->lt_scalers.s_100Hz.servo_coinc,ds->lt_scalers.s_1Hz_gated.servo_coinc,
                     ds->lt_scalers.s_1Hz.trig_coinc, ds->lt_scalers.s_100Hz.trig_coinc,ds->lt_scalers.s_1Hz_gated.trig_coinc); 

  return ret; 
}

int rno_g_daqstatus_dump_flower_json(FILE *f, const rno_g_daqstatus_t *ds)
{
  int ret = 0;
  uint64_t ncycles = ds->lt_scalers.ncycles;
  ret += fprintf(f,"{ \"when\": %f, \"ncycles\" : %" PRIu64 ", \"cycle_counter\": %" PRIu64 " , \" scaler_counter_1Hz: %hu, ", 
      ds->when_lt, ncycles, ds->lt_scalers.cycle_counter, ds->lt_scalers.scaler_counter_1Hz);


 
  const rno_g_lt_scaler_group_t *groups[] = {
    &ds->lt_scalers.s_1Hz,
    &ds->lt_scalers.s_1Hz_gated,
    &ds->lt_scalers.s_100Hz,
  };
  const char * group_names[] = { "1Hz", "100Hz","1HzGated" };

  ret += fprintf(f, "\"scalers\" : {");
  for (int i = 0; i < 3; i++)
  {
    ret += fprintf(f, "%c \"%s\": { \"coinc_trig\": %u, \"channel_trig\" : [ %u,%u,%u,%u], \"coinc_servo\": %u, \"channel_servo\": [ %u,%u,%u,%u] } "
        , i == 0  ? ' ' : ',', group_names[i], 
        groups[i]->trig_coinc, groups[i]->trig_per_chan[0], groups[i]->trig_per_chan[1], groups[i]->trig_per_chan[2], groups[i]->trig_per_chan[3],
        groups[i]->servo_coinc, groups[i]->servo_per_chan[0], groups[i]->servo_per_chan[1], groups[i]->servo_per_chan[2], groups[i]->servo_per_chan[3]);

  }

  ret += fprintf(f,"} }"); 
  return ret;
}

int rno_g_daqstatus_write(rno_g_file_handle_t h, const rno_g_daqstatus_t * ds) 
{
  io_header_t hd = {.magic = DAQSTATUS_MAGIC, .version = DAQSTATUS_VER }; 
  uint32_t sum = init_cksum; 
  do_write(h, sizeof(hd), &hd,0); 
  int wr = do_write(h, sizeof(rno_g_daqstatus_t), ds,&sum); 
  do_write(h, sizeof(sum),&sum,0); 
  return wr; 
}

// pre version 5, there was no delay cycle counter 
typedef struct rno_g_lt_scalers_v0
{
  rno_g_lt_scaler_group_t s_1Hz; 
  rno_g_lt_scaler_group_t s_1Hz_gated; 
  rno_g_lt_scaler_group_t s_100Hz; 
  uint64_t ncycles : 48 ;  //this is the 118 MHz clock
  uint16_t scaler_counter_1Hz : 16; 
} rno_g_lt_scalers_v0_t; 


typedef struct rno_g_daqstatus_v1
{
  double when; 
  uint32_t thresholds[RNO_G_NUM_RADIANT_CHANNELS]; 
  uint16_t scalers[RNO_G_NUM_RADIANT_CHANNELS]; 
  uint8_t prescalers[RNO_G_NUM_RADIANT_CHANNELS]; 
  float scaler_period; 
} rno_g_daqstatus_v1_t; 

typedef struct rno_g_daqstatus_v2
{
  double when; 
  uint32_t radiant_thresholds[RNO_G_NUM_RADIANT_CHANNELS]; 
  uint16_t radiant_scalers[RNO_G_NUM_RADIANT_CHANNELS]; 
  uint8_t radiant_prescalers[RNO_G_NUM_RADIANT_CHANNELS]; 
  float radiant_scaler_period; 
  uint8_t  lt_trigger_thresholds[RNO_G_NUM_LT_CHANNELS]; 
  uint8_t  lt_servo_thresholds[RNO_G_NUM_LT_CHANNELS]; 
  rno_g_lt_scalers_v0_t lt_scalers; 
  uint8_t station;
} rno_g_daqstatus_v2_t; 

typedef struct rno_g_daqstatus_v3
{
  double when_radiant; 
  double when_lt; 
  uint32_t radiant_thresholds[RNO_G_NUM_RADIANT_CHANNELS]; 
  uint16_t radiant_scalers[RNO_G_NUM_RADIANT_CHANNELS]; 
  uint8_t radiant_prescalers[RNO_G_NUM_RADIANT_CHANNELS]; 
  float radiant_scaler_period; 
  uint8_t  lt_trigger_thresholds[RNO_G_NUM_LT_CHANNELS]; 
  uint8_t  lt_servo_thresholds[RNO_G_NUM_LT_CHANNELS]; 
  rno_g_lt_scalers_v0_t lt_scalers; 
  uint8_t station;
} rno_g_daqstatus_v3_t; 

typedef struct rno_g_daqstatus_v4
{
  double when_radiant; 
  double when_lt; 
  uint32_t radiant_thresholds[RNO_G_NUM_RADIANT_CHANNELS]; 
  uint16_t radiant_scalers[RNO_G_NUM_RADIANT_CHANNELS]; 
  uint8_t radiant_prescalers[RNO_G_NUM_RADIANT_CHANNELS]; 
  float radiant_scaler_period; 
  uint8_t  lt_trigger_thresholds[RNO_G_NUM_LT_CHANNELS]; 
  uint8_t  lt_servo_thresholds[RNO_G_NUM_LT_CHANNELS]; 
  rno_g_lt_scalers_v0_t lt_scalers; 
  rno_g_radiant_voltages_t radiant_voltages; 
  rno_g_calpulser_info_t cal; 
  uint8_t station;
} rno_g_daqstatus_v4_t; 

static int real_rno_g_daqstatus_read(rno_g_file_handle_t h, rno_g_daqstatus_t *ds, int version)
{
  uint32_t sum = init_cksum; 
  uint32_t wanted_sum = 0; 
  int rd = 0; 
  int rdsum = 0;
  // here we handle converting to the newest kind of daqstatus
  switch (version) 
  {

    case 0: 
    case 1: 
      {
        //zero out struct before copying the parts we know
        memset(ds,0,sizeof(*ds)); 
        rno_g_daqstatus_v1_t dsv1; 
        rd = do_read(h,sizeof(dsv1),&dsv1, &sum); 
        if (version == 0) dsv1.scaler_period /=2.5; 
        ds->when_radiant = dsv1.when; 
        ds->radiant_scaler_period = dsv1.scaler_period; 
        memcpy(ds->radiant_thresholds, dsv1.thresholds, sizeof(ds->radiant_thresholds));
        memcpy(ds->radiant_scalers, dsv1.scalers, sizeof(ds->radiant_scalers));
        memcpy(ds->radiant_prescalers, dsv1.prescalers, sizeof(ds->radiant_prescalers));
        break;
      }
    case 2:
      {
        memset(ds,0,sizeof(*ds)); 
        rno_g_daqstatus_v2_t dsv2; 
        rd = do_read(h, sizeof(dsv2), &dsv2, &sum); 
        ds->when_radiant = dsv2.when; 
        memcpy(ds->radiant_thresholds, dsv2.radiant_thresholds, sizeof(ds->radiant_thresholds));
        memcpy(ds->radiant_scalers, dsv2.radiant_scalers, sizeof(ds->radiant_scalers));
        memcpy(ds->radiant_prescalers, dsv2.radiant_prescalers, sizeof(ds->radiant_prescalers));
        ds->radiant_scaler_period = dsv2.radiant_scaler_period; 
        memcpy(ds->lt_trigger_thresholds, dsv2.lt_trigger_thresholds, sizeof(ds->lt_trigger_thresholds));
        memcpy(ds->lt_servo_thresholds, dsv2.lt_servo_thresholds, sizeof(ds->lt_servo_thresholds));
        memcpy(&ds->lt_scalers, &dsv2.lt_scalers, sizeof(dsv2.lt_scalers)); 
        ds->station = dsv2.station; 
        break; 
      }
    case 3: 
      {
        memset(ds,0,sizeof(*ds)); 
        rno_g_daqstatus_v3_t dsv3; 
        rd = do_read(h, sizeof(dsv3), &dsv3, &sum); 
        ds->when_radiant = dsv3.when_radiant; 
        ds->when_lt = dsv3.when_lt; 
        memcpy(ds->radiant_thresholds, dsv3.radiant_thresholds, sizeof(ds->radiant_thresholds));
        memcpy(ds->radiant_scalers, dsv3.radiant_scalers, sizeof(ds->radiant_scalers));
        memcpy(ds->radiant_prescalers, dsv3.radiant_prescalers, sizeof(ds->radiant_prescalers));
        ds->radiant_scaler_period = dsv3.radiant_scaler_period; 
        memcpy(ds->lt_trigger_thresholds, dsv3.lt_trigger_thresholds, sizeof(ds->lt_trigger_thresholds));
        memcpy(ds->lt_servo_thresholds, dsv3.lt_servo_thresholds, sizeof(ds->lt_servo_thresholds));
        memcpy(&ds->lt_scalers, &dsv3.lt_scalers, sizeof(dsv3.lt_scalers)); 
        ds->station = dsv3.station; 
        break; 
      }
    case 4: 
      {
        memset(ds,0,sizeof(*ds)); 
        rno_g_daqstatus_v4_t dsv4; 
        rd = do_read(h, sizeof(dsv4), &dsv4, &sum); 
        ds->when_radiant = dsv4.when_radiant; 
        ds->when_lt = dsv4.when_lt; 
        memcpy(ds->radiant_thresholds, dsv4.radiant_thresholds, sizeof(ds->radiant_thresholds));
        memcpy(ds->radiant_scalers, dsv4.radiant_scalers, sizeof(ds->radiant_scalers));
        memcpy(ds->radiant_prescalers, dsv4.radiant_prescalers, sizeof(ds->radiant_prescalers));
        ds->radiant_scaler_period = dsv4.radiant_scaler_period; 
        memcpy(ds->lt_trigger_thresholds, dsv4.lt_trigger_thresholds, sizeof(ds->lt_trigger_thresholds));
        memcpy(ds->lt_servo_thresholds, dsv4.lt_servo_thresholds, sizeof(ds->lt_servo_thresholds));
        memcpy(&ds->lt_scalers, &dsv4.lt_scalers, sizeof(dsv4.lt_scalers)); 
        ds->radiant_voltages = dsv4.radiant_voltages; 
        ds->cal = dsv4.cal; 
        ds->station = dsv4.station; 
        break; 
      }
    case DAQSTATUS_VER:
      {
        rd = do_read(h,sizeof(*ds),ds, &sum); 
        break;
      }
    default: 
      fprintf(stderr,"Unknown daqstatus version %d\n", version); 
      return 0; 
  }

  rdsum = do_read(h,sizeof(wanted_sum), &wanted_sum,0); 
  if (!rdsum || sum!=wanted_sum) 
  {
    fprintf(stderr,"Checksum error. Got %x, wanted %x\n", sum,wanted_sum); 
    return -1; 
  }

  return rd; 
}

int rno_g_daqstatus_read(rno_g_file_handle_t h, rno_g_daqstatus_t *ds)
{
  io_header_t hd; 

  int rd = read_io_header(&hd, h);
  if (!rd) return 0; 

  if (hd.magic != DAQSTATUS_MAGIC)
  {
    //this is not a header! 
    fprintf(stderr,"Wrong magic! Got %hx, expected %hx\n", hd.magic, DAQSTATUS_MAGIC); 
    return -1; 
  }

  return real_rno_g_daqstatus_read(h, ds, hd.version);
}

const char * rno_g_trigger_type_to_string(rno_g_trigger_type_t t)
{
  switch((int) t)
  {
    case RNO_G_TRIGGER_SOFT:
      return "SOFT";
    case RNO_G_TRIGGER_EXT:
      return "EXT";
    case RNO_G_TRIGGER_PPS:
      return "PPS";
    case RNO_G_TRIGGER_RF_RADIANTX:
      return "RADIANTX";
    case RNO_G_TRIGGER_RF_RADIANT0 | RNO_G_TRIGGER_RF_RADIANTX:
      return "RADIANT0";
    case RNO_G_TRIGGER_RF_RADIANT1 | RNO_G_TRIGGER_RF_RADIANTX:
      return "RADIANT1";
    default:
      return "UNKNOWN";
  }
}








int rno_g_any_read(rno_g_file_handle_t h, rno_g_any_t * any)
{

  if (!any) return -1; 
  any->ok = 0; 
  io_header_t hd;
  int rd = read_io_header(&hd, h); 
  if (!rd) return 0; 

  switch (hd.magic) 
  {
    case HEADER_MAGIC: 
      rd  = real_rno_g_header_read(h, &any->data.hd, hd.version); 
      any->what = RNO_G_HEADER_T; 
      break;
    case WAVEFORM_MAGIC: 
      rd  = real_rno_g_waveform_read(h, &any->data.wf, hd.version); 
      any->what = RNO_G_WAVEFORM_T; 
      break;
    case DAQSTATUS_MAGIC: 
      rd  = real_rno_g_daqstatus_read(h, &any->data.ds, hd.version); 
      any->what = RNO_G_DAQSTATUS_T; 
      break;
    case PEDESTAL_MAGIC: 
      rd  = real_rno_g_pedestal_read(h, &any->data.ped, hd.version); 
      any->what = RNO_G_PEDESTAL_T; 
      break;
    default: 
      printf("Unrecognized magic: %x\n", hd.magic); 
  }

  // TODO, this should be a better check... 
  any->ok = rd > 0; 

  return rd; 
}

int rno_g_any_write(rno_g_file_handle_t h, const rno_g_any_t * any) 
{
  if (!any || !any->ok) return -1; 

  switch (any->what) 
  {
    case RNO_G_WAVEFORM_T: 
      return rno_g_waveform_write(h, &any->data.wf);
    case RNO_G_HEADER_T: 
      return rno_g_header_write(h, &any->data.hd);
    case RNO_G_DAQSTATUS_T: 
      return rno_g_daqstatus_write(h, &any->data.ds);
    case RNO_G_PEDESTAL_T: 
      return rno_g_pedestal_write(h, &any->data.ped);
    default: 
      return -1; 
  }
  return -1; 
}

int rno_g_any_dump(FILE * f, const rno_g_any_t * any) 
{
  if (!any || !any->ok) return -1; 

  switch (any->what) 
  {
    case RNO_G_WAVEFORM_T: 
      return rno_g_waveform_dump(f, &any->data.wf);
    case RNO_G_HEADER_T: 
      return rno_g_header_dump(f, &any->data.hd);
    case RNO_G_DAQSTATUS_T: 
      return rno_g_daqstatus_dump(f, &any->data.ds);
    case RNO_G_PEDESTAL_T: 
      return rno_g_pedestal_dump(f, &any->data.ped);
    default: 
      return -1; 
  }
  return -1; 
}

const char * rno_g_data_type_name(rno_g_data_type_t t)
{
  switch(t)
  {
    case RNO_G_WAVEFORM_T:
      return "waveform";
    case RNO_G_HEADER_T:
      return "header";
    case RNO_G_DAQSTATUS_T:
      return "daqstatus";
    case RNO_G_PEDESTAL_T:
      return "pedestal";
    default:
      return "invalid";
  }

  return NULL;
}
int rno_g_any_dump_json(FILE * f, const rno_g_any_t * any) 
{
  if (!any || !any->ok) return -1; 

  int ret = fprintf(f, " { \"type\": \"%s\", \"payload\": ", rno_g_data_type_name(any->what));
  switch (any->what) 
  {
    case RNO_G_WAVEFORM_T: 
      return ret + rno_g_waveform_dump_json(f, &any->data.wf) + fprintf(f," }");
    case RNO_G_HEADER_T: 
      return ret + rno_g_header_dump_json(f, &any->data.hd) + fprintf(f," }");
    case RNO_G_DAQSTATUS_T: 
      return ret + rno_g_daqstatus_dump_json(f, &any->data.ds) + fprintf(f," }");
    case RNO_G_PEDESTAL_T: 
      return ret + rno_g_pedestal_dump_json(f, &any->data.ped) + fprintf(f," }");
    default: 
      return -1; 
  }
  return -1; 
}



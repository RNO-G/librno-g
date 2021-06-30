#include "rno-g.h" 
#include <stdio.h> 
#include <stddef.h> 
#include <zlib.h> 
#include <string.h> 
#include <inttypes.h>
#include <time.h> 

/* IO functions. 
 *
 * All assumes little-endian and similar ABI to Linux/gcc. 
 *
 */

#define HEADER_VER 1
#define WF_VER 3 
#define PED_VER 2 
#define DAQSTATUS_VER 2 

#define HEADER_MAGIC 0xead1 
#define WAVEFORM_MAGIC 0xafd1 
#define PEDESTAL_MAGIC 0x57a1 
#define DAQSTATUS_MAGIC 0xdacc 


typedef struct io_header
{
  uint16_t magic; 
  uint16_t version;
} io_header_t; 



static uint32_t init_cksum =1; 

//This is a bit silly, since we usually will save as gzip and have a checksum anyway
static uint32_t cksum(uint32_t start_cksum, int N, const uint8_t *data) 
{
  return adler32(start_cksum, data,N); 
}

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



int rno_g_header_read(rno_g_file_handle_t h, rno_g_header_t *header)
{
  io_header_t hd; 
  int rd = do_read(h, sizeof(hd), &hd,0); 
  if (!rd) return 0; 

  if (hd.magic != HEADER_MAGIC)
  {
    //this is not a header! 
    fprintf(stderr,"Wrong magic! Got %hx, expected %hx\n", hd.magic, HEADER_MAGIC); 
    return -1; 
  }

  uint32_t sum = init_cksum; 
  uint32_t wanted_sum = 0; 
  rd = 0; 
  int rdsum = 0;
  // here we handle converting to the newest kind of header
  switch (hd.version) 
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
      fprintf(stderr,"Uknown header version %d\n", hd.version); 
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

  wr += do_write(h, 1, &wf->station, &sum); 

  do_write(h, sizeof(sum),&sum,0); 

  return wr;
}

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


int rno_g_waveform_read(rno_g_file_handle_t h, rno_g_waveform_t *wf)
{
  io_header_t hd; 
  int rd = do_read(h, sizeof(hd), &hd,0); 
  if (!rd) return 0; 

  if (hd.magic != WAVEFORM_MAGIC)
  {
    //this is not a waveform! 
    fprintf(stderr,"Wrong magic! Got %hx, expected %hx\n", hd.magic, WAVEFORM_MAGIC); 
    return -1; 
  }

  uint32_t sum = init_cksum; 
  uint32_t wanted_sum = 0; 
  int rdsum;
  int ichan = 0; 
  rd = 0; 
  // here we handle converting to the newest kind of waveform
  switch (hd.version) 
  {
    case 1: 
    case 2: 
    case WF_VER:
      {
        rd = do_read(h,N_BEFORE_DATA,wf,&sum); 
        for (ichan = 0; ichan < RNO_G_NUM_RADIANT_CHANNELS; ichan++)
        {
          rd+= do_read(h,2*wf->radiant_nsamples, wf->radiant_waveforms[ichan], &sum);
          if ( hd.version < 3) 
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

        if (hd.version > 1) 
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
      fprintf(stderr,"Unknown waveform version %d\n", hd.version); 
  }
  return 0; 
}


int rno_g_init_handle(rno_g_file_handle_t * h, const char * name, const char * mode) 
{

  //check for gz ending
  char * dot = strrchr(name,'.'); 

  if (!strcasecmp(dot, ".gz"))
  {
    h->type = RNO_G_GZIP; 
    h->handle.gz = gzopen(name,mode); 
    return h->handle.gz==0;
  }

  h->type = RNO_G_RAW; 
  h->handle.raw = fopen(name,mode); 
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

  return 1; 
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


int rno_g_pedestal_read(rno_g_file_handle_t h, rno_g_pedestal_t * pd) 
{
  io_header_t hd; 
  int rd = do_read(h, sizeof(hd), &hd,0); 

  if (!rd) return 0; 

  if (hd.magic != PEDESTAL_MAGIC) 
  {
    fprintf(stderr,"Wrong magic! Got %hx, expected %hx\n", hd.magic, PEDESTAL_MAGIC); 
    return -1; 
  }

  uint32_t sum = init_cksum; 
  uint32_t wanted_sum = 0; 
  int rdsum=0;

  switch (hd.version) 
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
      memcpy(pd->pedestals, pdv1.pedestals, sizeof(pd->pedestals)); 
      break;
    }
    case PED_VER: 
      rd += do_read(h, sizeof(rno_g_pedestal_t), pd, &sum); 
      break; 
    default: 
      fprintf(stderr,"Unknown pedestal version %d\n", hd.version); 
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


int rno_g_daqstatus_dump(FILE *f, const rno_g_daqstatus_t* ds) 
{
  int when = ds->when; 
  int ns = (ds->when-when)*1e9; 
  struct tm when_tm; 
  gmtime_r((time_t*)&when, &when_tm); 
  int ret = fprintf(f,"DAQSTATUS, radiant_period="); 
  if (!ds->radiant_scaler_period) 
  {
    ret+=fprintf(f,"PPS"); 
  }
  else
  {
    ret+=fprintf(f,"%f s", ds->radiant_scaler_period);  
  }
  ret += fprintf(f,", recorded at %04d-%02d-%02d %02d:%02d:%02d.%09dZ\n", 
                 when_tm.tm_year + 1900, 1+when_tm.tm_mon, when_tm.tm_mday, when_tm.tm_hour, 
                 when_tm.tm_min, when_tm.tm_sec,  ns); 
  ret+=fprintf(f,  "========== RADIANT================\n"); 
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

  ret+=fprintf(f,  "============FLOWER=============\n"); 
  uint64_t ncycles = ds->lt_scalers.ncycles; 
  ret+=fprintf(f,  "  ncycles: %" PRIu64 " , counter: %hu\n", ncycles, ds->lt_scalers.scaler_counter_1Hz); 
  ret+=fprintf(f,  "==CH==SERVO_THR==TRIG_THR=SERVO_SCAL_1HZ==SERVO_SCAL_0.1HZ==SERVO_SCAL_GATE==TRIG_SCAL_1HZ==TRIG_SCAL_0.1Hz==TRIG_SCAL_GATED\n"); 
  for (int i = 0; i < RNO_G_NUM_LT_CHANNELS; i++)
  {
    ret+=fprintf(f," %d  | %03d     | %03d   |      %04d   |      %04d         |       %04d     |      %04d    |      %04d      |   %04d        \n",
                     i, ds->lt_servo_thresholds[i], ds->lt_trigger_thresholds[i],
                     ds->lt_scalers.s_1Hz.servo_per_chan[i], ds->lt_scalers.s_100mHz.servo_per_chan[i],ds->lt_scalers.s_1Hz_gated.servo_per_chan[i],
                     ds->lt_scalers.s_1Hz.trig_per_chan[i], ds->lt_scalers.s_100mHz.trig_per_chan[i],ds->lt_scalers.s_1Hz_gated.trig_per_chan[i]);
  }

  ret += fprintf(f,"coinc|         |          |      %04d   |      %04d         |       %04d     |      %04d    |      %04d      |   %04d        \n",
                     ds->lt_scalers.s_1Hz.servo_coinc, ds->lt_scalers.s_100mHz.servo_coinc,ds->lt_scalers.s_1Hz_gated.servo_coinc,
                     ds->lt_scalers.s_1Hz.trig_coinc, ds->lt_scalers.s_100mHz.trig_coinc,ds->lt_scalers.s_1Hz_gated.trig_coinc); 

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

typedef struct rno_g_daqstatus_v1
{
  double when; 
  uint32_t thresholds[RNO_G_NUM_RADIANT_CHANNELS]; 
  uint16_t scalers[RNO_G_NUM_RADIANT_CHANNELS]; 
  uint8_t prescalers[RNO_G_NUM_RADIANT_CHANNELS]; 
  float scaler_period; 
} rno_g_daqstatus_v1_t; 


int rno_g_daqstatus_read(rno_g_file_handle_t h, rno_g_daqstatus_t *ds)
{
  io_header_t hd; 
  int rd = do_read(h, sizeof(hd), &hd,0); 
  if (!rd) return 0; 

  if (hd.magic != DAQSTATUS_MAGIC)
  {
    //this is not a header! 
    fprintf(stderr,"Wrong magic! Got %hx, expected %hx\n", hd.magic, DAQSTATUS_MAGIC); 
    return -1; 
  }

  uint32_t sum = init_cksum; 
  uint32_t wanted_sum = 0; 
  rd = 0; 
  int rdsum = 0;
  // here we handle converting to the newest kind of daqstatus
  switch (hd.version) 
  {

    case 0: 
    case 1: 
      {
        //zero out struct before copying the parts we know
        memset(ds,0,sizeof(*ds)); 
        rno_g_daqstatus_v1_t dsv1; 
        rd = do_read(h,sizeof(dsv1),&dsv1, &sum); 
        if (hd.version == 0) dsv1.scaler_period /=2.5; 
        ds->when = dsv1.when; 
        ds->radiant_scaler_period = dsv1.scaler_period; 
        memcpy(ds->radiant_thresholds, dsv1.thresholds, sizeof(ds->radiant_thresholds));
        memcpy(ds->radiant_scalers, dsv1.scalers, sizeof(ds->radiant_scalers));
        memcpy(ds->radiant_prescalers, dsv1.prescalers, sizeof(ds->radiant_prescalers));
        break;

      }
    case DAQSTATUS_VER:
      {
        rd = do_read(h,sizeof(*ds),ds, &sum); 
        break;
      }
    default: 
      fprintf(stderr,"Unknown daqstatus version %d\n", hd.version); 
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





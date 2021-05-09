#include "rno-g.h" 
#include <stdio.h> 
#include <stddef.h> 
#include <zlib.h> 
#include <string.h> 
#include <time.h> 

/* IO functions. 
 *
 * All assumes little-endian and similar ABI to Linux/gcc. 
 *
 */

#define HEADER_VER 0
#define WF_VER 0 
#define PED_VER 0 

#define HEADER_MAGIC 0xead1 
#define WAVEFORM_MAGIC 0xafd1 
#define PEDESTAL_MAGIC 0x57a1 


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
      rd = fread(data,N,1,h.handle.raw); 
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
  ret+=fprintf(f,"RUN %d, EVENT %d\n", header->event_number, header->run_number); 
  ret+=fprintf(f, "   PPS_COUNT: %d, STATION: %d\n", header->pps_count, header->station_number);
  return ret; 
}

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
  // here we handle converting to the newest kind of header
  switch (hd.version) 
  {

    case HEADER_VER:
      {
        int rd = do_read(h,sizeof(*header),header, &sum); 
        int rdsum = do_read(h,sizeof(wanted_sum), &wanted_sum,0); 
        if (!rdsum || sum!=wanted_sum) 
        {
          fprintf(stderr,"Checksum error. Got %x, wnated %x\n", sum,wanted_sum); 
          return -1; 
        }
        return rd; 
      }
    default: 
      fprintf(stderr,"Uknown header version %d\n", hd.version); 
  }
  return 0; 
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


  do_write(h, sizeof(sum),&sum,0); 

  return wr;
}


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
    case WF_VER:
      {
        rd = do_read(h,N_BEFORE_DATA,wf,&sum); 
        for (ichan = 0; ichan < RNO_G_NUM_RADIANT_CHANNELS; ichan++)
        {
          rd+= do_read(h,2*wf->radiant_nsamples, wf->radiant_waveforms[ichan], &sum);
        }
         for (ichan = 0; ichan < RNO_G_NUM_LT_CHANNELS; ichan++)
        {
          rd+= do_read(h,wf->lt_nsamples, wf->lt_waveforms[ichan], &sum);
        }
        
        rdsum = do_read(h, sizeof(wanted_sum),&wanted_sum,0); 


        if (!rdsum || sum!=wanted_sum) 
        {
          fprintf(stderr,"Checksum error. Got %x, wanted %x\n", sum,wanted_sum); 
          return -1; 
        }
        return rd; 
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
  char ctime_buf[32]; 
  const char * timestr = ctime_r((time_t*) &pedestal->when, ctime_buf);
  int ret = fprintf(stdout,"PEDESTAL of %d events recorded at %s\n", pedestal->nevents, timestr); 

  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
  {
    ret += fprintf(stdout,"CH%d,", i); 
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


int rno_g_pedestal_read(rno_g_file_handle_t h, rno_g_pedestal_t * pd) 
{
  io_header_t hd; 
  int rd = do_read(h, sizeof(hd), &hd,0); 
  if (hd.magic != PEDESTAL_MAGIC) 
  {
    fprintf(stderr,"Wrong magic! Got %hx, expected %hx\n", hd.magic, PEDESTAL_MAGIC); 
    return -1; 

  }

  uint32_t sum = init_cksum; 
  uint32_t wanted_sum = 0; 
  int rdsum;

  switch (hd.version) 
  {
    case PED_VER: 
      rd = do_read(h, sizeof(rno_g_pedestal_t), pd, &sum); 
      rdsum = do_read(h, sizeof(wanted_sum),&wanted_sum,0); 
      if (!rdsum || sum != wanted_sum) 
      {
          fprintf(stderr,"Checksum error. Got %x, wanted %x\n", sum,wanted_sum); 
          return -1; 
      }
      return rd; 

    default: 
      fprintf(stderr,"Unknown pedestal version %d\n", hd.version); 
  }
  return 0; 

}


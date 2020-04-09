#include "rno-g.h" 
#include <stdio.h> 
#include <stddef.h> 

/* IO functions. 
 *
 * All assumes little-endian and similar ABI to Linux/gcc. 
 *
 */

#define HEADER_VER 0
#define WF_VER 0 

#define HEADER_MAGIC 0xead1 
#define WAVEFORM_MAGIC 0xafd1 


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

  if (cksum) *sum = cksum(*sum,N,data); 

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
    case RNO_G_GZIP: 
      rd = gzread(h.handle.gz, data,N); 
    default: 
      return 0; 
  }

  if (cksum) *sum = cksum(*sum,rd,data); 
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
}


 //WARNING: watch out for this struct changing! 
#define N_BEFORE_DATA  ( offsetof(rno_g_waveform_t,reserved) + sizeof(((rno_g_waveform_t*) 0)->reserved)) 

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
    wr += do_write(h, 2*wf->nsamples, wf->radiant_waveforms[ichan],&sum); 
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
  // here we handle converting to the newest kind of header
  switch (hd.version) 
  {
    case WF_VER:
      {
        rd = do_read(h,N_BEFORE_DATA,wf,&sum); 
        for (ichan = 0; ichan < RNO_G_NUM_RADIANT_CHANNELS; ichan++)
        {
          rd+= do_read(h,2*wf->nsamples, wf->radiant_waveforms[ichan], &sum);
        }
        
        rdsum = do_read(h, sizeof(wanted_sum),&wanted_sum,0); 


        if (!rdsum || sum!=wanted_sum) 
        {
          fprintf(stderr,"Checksum error. Got %x, wnated %x\n", sum,wanted_sum); 
          return -1; 
        }
        return rd; 
      }
    default: 
      fprintf(stderr,"Uknown waveform version %d\n", hd.version); 
  }
}


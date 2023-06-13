#include "rno-g-block-offsets.h"

#include <math.h> 
#include <string.h>
#include <fftw3.h> 
#include <pthread.h>
#include <gsl/gsl_statistics_short.h> 


static void compute_offsets_mean(int nsamp, const int16_t * restrict wf, int16_t * restrict offsets)
{
  int nwindows = nsamp >> 7 ; 

  for (int iwindow = 0; iwindow < nwindows; iwindow++) 
  {
    uint32_t accum = 0; 
    int base = iwindow << 7; 
    for (int i =0; i < 128; i++) 
    {
      accum+= wf[base +i]; 
    }
    offsets[iwindow] = (accum+64) >>7; 
  }
}


//this uses a histogram approach to the minimum. Maybe an nth_element like approach is faster? Probably should profile... 
static void compute_offsets_median_hist(int nsamp, const int16_t * restrict wf, int16_t * restrict offsets)
{
  static _Thread_local uint8_t hist[8192]; 

  int nwindows = nsamp >> 7 ; 

  //histogram all values in window, storing the minimum
  for (int iwindow = 0; iwindow < nwindows; iwindow++) 
  {
    int16_t vmin = 4095; 
    int base = iwindow << 7; 
    for (int i = 0; i < 128; i++) 
    {
      int16_t val = wf[base+i]; 
      vmin = val < vmin ? val : vmin; 
      hist[val+4096]++; 
    }

    int16_t accum = 0; 
    uint16_t j = vmin + 4096; //start at minimum
    while (accum < 64)
    {
      accum += hist[j++]; 
    }
    uint16_t lower = j; 
    while (accum < 65) 
    {
      accum += hist[j++]; 
    }

    offsets[iwindow] = (j+lower+1)/2; 

    memset(hist,0,sizeof(hist)); 
  }
}

static void compute_offsets_median_select(int nsamp, const int16_t * restrict wf, int16_t * restrict offsets)
{

  int nwindows = nsamp >> 7 ; 

  for (int iwindow = 0; iwindow < nwindows; iwindow++) 
  {
    int16_t scratch[128]; 
    int base = iwindow << 7; 
    memcpy(scratch, wf + base, sizeof(scratch)); 
    offsets[iwindow] = 0.5+gsl_stats_short_median(scratch, 1, 128); 
  }
}






int rno_g_block_offset( int nsamp, const int16_t * restrict wf, int16_t * restrict offsets,  rno_g_block_offset_method_t method) 
{
  return rno_g_block_offset_advanced(nsamp, (int16_t * restrict) wf, 0, offsets, method, 0); 
}



//lookup tables for all the complex exponentials 
const float ns = 128; // samples per block 
const float twopif = 2 * M_PI;                       
                      
#define NFFT(NBLOCKS) (NBLOCKS >> 6) +1 
#define LOOKUP_TABLE_MAX 32

#define LOOKUP_TABLE_CONSTANT_TERM_INDEX(nblocks, i) ((1+nblocks)*i)
#define LOOKUP_TABLE_JTERM_INDEX(nblocks, i,j) ((1+nblocks)*i +1 + j)

static rno_g_complex * lookup_table[LOOKUP_TABLE_MAX]; 

#define LOOKUP_TABLE_CONSTANT_TERM(nblocks, i) lookup_table[nblocks-1][LOOKUP_TABLE_CONSTANT_TERM_INDEX(nblocks,i)]
#define LOOKUP_TABLE_JTERM(nblocks, i,j) lookup_table[nblocks-1][LOOKUP_TABLE_JTERM_INDEX(nblocks,i,j)]
static pthread_mutex_t lookup_table_lock = PTHREAD_MUTEX_INITIALIZER; 

void maybe_init_lookup_table(int nblocks) 
{
  if (!lookup_table[nblocks-1]) 
  {
    //do a double checked lock for thread safety 
    pthread_mutex_lock(&lookup_table_lock); 
    if (!lookup_table[nblocks-1]) 
    {
      int nfft =NFFT(nblocks); 
      rno_g_complex * ptr = malloc((1+nblocks)*sizeof(rno_g_complex) * nfft); 
      //allocate terms contiguously
      float dtdf = 1/(2.*nfft); 
      for (int i = 0; i <  nfft; i++) 
      {
        float w= twopif*i;//don't include df, since cancels out with dt
        ptr[LOOKUP_TABLE_CONSTANT_TERM_INDEX(nblocks,i)] = cexpf(-I*w*(ns/2) * dtdf) *  sinf( ns*w*dtdf/2.) / sinf(w*dtdf/2.); 
        for (int j = 0; j < nblocks; j++) 
        {
          ptr[LOOKUP_TABLE_JTERM_INDEX(nblocks,i,j)] = cexpf(-I*w*j*ns*dtdf);
        }
      }
      lookup_table[nblocks-1] = ptr; 
    }
    pthread_mutex_unlock(&lookup_table_lock); 
  }
}



static void apply_fft_offset(int nblocks, int16_t * restrict A, rno_g_complex * restrict fft_offsets, float factor) 
{
  if (nblocks < 1 || nblocks > LOOKUP_TABLE_MAX) return ;
  maybe_init_lookup_table(nblocks); 
  int nfft = NFFT(nblocks) ; 

  for (int i = 0; i < nfft; i++) 
  {
    rno_g_complex C = LOOKUP_TABLE_CONSTANT_TERM(nblocks, i); 
    rno_g_complex ans = 0; 

    //unrolled loop 
    for (int jj = 0; jj < nblocks/4; jj++) 
    {
      ans += A[4*jj] * LOOKUP_TABLE_JTERM(nblocks,i,4*jj)     +  
             A[4*jj+1] * LOOKUP_TABLE_JTERM(nblocks,i,4*jj+1) +  
             A[4*jj+2] * LOOKUP_TABLE_JTERM(nblocks,i,4*jj+2) +  
             A[4*jj+3] * LOOKUP_TABLE_JTERM(nblocks,i,4*jj+3); 
    }

    //leftovers 
    for (int j = 4 * (nblocks/4); j < nblocks; j++) 
    {
      ans += A[j] * LOOKUP_TABLE_JTERM(nblocks,i,j); 
    }
    fft_offsets[i] +=factor*C*ans; 
  }
}



int rno_g_block_offset_advanced( int nsamp, int16_t * restrict wf, rno_g_complex * restrict fft, int16_t * restrict offsets, rno_g_block_offset_method_t method, rno_g_block_offset_flags_t flags) 
{
  switch(method) 
  {

    case RNO_G_BLOCK_OFFSET_MEAN:
      compute_offsets_mean(nsamp, wf, offsets); 
      break;
    case RNO_G_BLOCK_OFFSET_MEDIAN_HIST: 
      compute_offsets_median_hist(nsamp, wf, offsets); 
      break; 
    case RNO_G_BLOCK_OFFSET_MEDIAN_SELECT: 
      compute_offsets_median_select(nsamp, wf, offsets); 
      break; 
    case RNO_G_BLOCK_OFFSET_OOB_FIT_SIMPLEX:
    case RNO_G_BLOCK_OFFSET_OOB_FIT_GRADIENT:
//      compute_offsets_oob (nsamp, wf, fft, offsets)
      break; 
    default:  
        return -1; 
  }

  if (flags & RNO_G_BLOCK_OFFSET_APPLY) 
  {
    //TODO: replace with NEON intrinsics 
    for (int i = 0; i < nsamp; i++) 
    {
      wf[i] -= offsets[i>>7]; 
    }

    if(fft) 
    {
      apply_fft_offset(nsamp>>7, offsets, fft, -1); 
    }
  }
  return 0; 
}



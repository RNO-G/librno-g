#ifndef _RNO_G_NSAMPLE_DIFF_HIST_H
#define _RNO_G_NSAMPLE_DIFF_HIST_H
#include "rno-g.h" 
#include <string.h>

/** Difference histogram between nsamples. What is histogram is either wf[i] - wf[i-delta_nsamples] or the absolute
 * value of this.   */ 
typedef struct rno_g_nsample_diff_hist
{
  //set these up to initialize histogram
  struct rno_g_nsample_diff_hist_setup
  {
    uint8_t station; 
    uint8_t channel;
    uint16_t delta_nsamples; //the number of samples to take differnences between
    uint16_t nbins;  // Number of bins in the histogram
    uint8_t binsize;  // The size of each bin
    uint8_t use_abs : 1;  // use abs if true, i.e. the histogram range is [0,nbins*binsize), otherwise the range is  [-nbins/2*binsize, nbins/2*binsize) if nbins is even, [nbins/2*binsize,nbins/2*binsize] if nbins is odd. 
  } setup; 
                      
  //zero these to start 
  uint32_t nfilled;  // number of entries filled in the histogram
  float entries_sum; //the sum of entries filled in the histogram (used for mean) 
  float entries_sum2; // the sum of entries squared filled in the histogram (used for stdev) 
  uint32_t data[]; //nbins+2 long due to underflow, overflow, even though underflow makes no sense for abs
} rno_g_nsample_diff_hist_t; 

rno_g_nsample_diff_hist_t * rno_g_nsample_diff_hist_create( struct rno_g_nsample_diff_hist_setup setup); 


#define rno_g_sample_diff_hist_size(nbins) (sizeof(rno_g_nsample_diff_hist_t) + (nbins+2) * sizeof(uint32_t))

// returns number of bins filled on success, -1 on problem (e.g. channel bad, or station number doesn't match) 
int rno_g_nsample_diff_hist_fill(rno_g_nsample_diff_hist_t * hist, const rno_g_waveform_t * wf); 

// writes histogram as a json 
int rno_g_nsample_diff_hist_write_json(FILE *f, const rno_g_nsample_diff_hist_t * h, int indent_level); 

// writes histograms as an html page using jsroot  
int rno_g_nsample_diff_hist_write_jsroot_webpage(FILE *f, int nhist, const rno_g_nsample_diff_hist_t ** h); 


#endif


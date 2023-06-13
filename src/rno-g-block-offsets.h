
/** 
 * RNO-G Block Offset Handling
 *
 * (C) 2023 RNO-G Collaboration
 * Cosmin Deaconu <cozzyd@kicp.uchicago.edu> 
 * Alexander Novikov  <novikov@udel.edu> 
 *
 */ 



#ifndef _RNO_G_BLOCK_OFFSETS_H
#define _RNO_G_BLOCK_OFFSETS_H

#ifdef __cplusplus
extern "C" { 
#endif
#include "rno-g.h" 
#include <complex.h> 
#include <stdint.h> 

//hopefully float precision is enough? 
typedef _Complex float rno_g_complex; 

typedef enum 
{
  RNO_G_BLOCK_OFFSET_MEAN,                //Calculate block offsets using mean
  RNO_G_BLOCK_OFFSET_MEDIAN_HIST,         //Calculate block offsets using median, calculated using histogram
  RNO_G_BLOCK_OFFSET_MEDIAN_SELECT,       //Calculate block offsets using median, calculated using quickselect,
  RNO_G_BLOCK_OFFSET_OOB_FIT_SIMPLEX,     //Calculate block offsets using OOB fit with simplex method
  RNO_G_BLOCK_OFFSET_OOB_FIT_GRADIENT     //Calculate block offsets using OOB fit with gradient method
} rno_g_block_offset_method_t ; 



/* 
 * This computes the block offset using the specified method to the input waveform, computing the offset (rounded to an int) 
 * using the simpler API. 
 *
 * @param nsamp The number of samples in wf (should be a multiple of the window size... otherwise what are you doing? 
 * @param wf [in]   The input samples 
 * @param offsets [out] The computed offsets. assumed to be long enough to hold nsamp/128. 
 * @param method The method to use
 * @returns 0 on success
 */ 
int rno_g_block_offset( int nsamp, const int16_t * restrict wf, int16_t *restrict offsets, rno_g_block_offset_method_t method); 


typedef enum 
{
  RNO_G_BLOCK_OFFSET_APPLY   = 1 << 0 , //apply the blockoffset to the input arrays 
  RNO_G_BLOCK_OFFSET_VERBOSE = 1 << 1, // verbose output
} rno_g_block_offset_flags_t; 

/*
 *
 * This computes the block offset using the specified method to the input waveform,r eturning the offset (rounding to an int). Unlike the simplier API, this may also be 
 * used to apply the offset to the input waveform. Also, it optionally takes a pointer to the FFT of the waveform, which is assumed to be correct, and will also modify the fft if applying an offset accordingly. 
 * 
 * @param nsamp The number of samples in wf
 * @param wf [inout] The input waveform, will only be modified if RNO_G_BLOCKOFFSET_APPLY is in flags 
 * @param fft [inout] If present, will be assumed to be correct and used in OOB methods. Regardless of method, if apply is true, will be applied to the FFT as well .
 * @param offsets [out] The computed offsets. assumed to be long enough to hold nsamp/128. 
 * @param method The method to use
 * @param flags see above
 * @returns 0 on success
 */

int rno_g_block_offset_advanced( int nsamp, int16_t * restrict wf, rno_g_complex * restrict fft, int16_t * restrict offsets, rno_g_block_offset_method_t method, rno_g_block_offset_flags_t flags); 



#ifdef __cplusplus
}
#endif 
#endif

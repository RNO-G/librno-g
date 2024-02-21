#ifndef _RNO_G_REAL_TIME_H
#define _RNO_G_REAL_TIME_H

/** Real-Time Analysis Scripts Embedded into Radiant API
 *  
 *  Ryan Krebs
 *  rjk5416@psu.edu
 *
 * */


#ifdef __cplusplus
extern "C" {
#endif

#include "rno-g.h" 



int real_time_calc_rms(real_time_t * rt); 
int real_time_calc_snr(real_time_t * rt); 
int real_time_load_template(real_time_t * rt);
int real_time_load_fft(real_time_t * rt);
int real_time_free_fft(real_time_t * rt);
int real_time_calc_template_fft(real_time_t * rt);
int real_time_calc_event_fft(real_time_t * rt);
int real_time_calc_impulsivity(real_time_t * rt);
int real_time_calc_template_match(real_time_t * rt);
int real_time_calc_phase_delays(real_time_t * rt);
int real_time_calc_phased_integral(real_time_t * rt);
int real_time_set_thresh(real_time_t * rt);
int real_time_calc_priority(real_time_t * rt);
int real_time_set_transform(real_time_t * rt);
int real_time_calc_desc(real_time_t * rt);
int pick_buffered_events(float * lin_events,uint8_t * sorted_places, int n_events,float threshold);
int sort_buffered_events(float * lin_events,uint8_t * sorted_places, int n_events,int index_of_best);


int test_phased_power_sum();
int test_template();
int see_perf_and_bad_snr();
int benchmark_vars();
int old_rms(int16_t * waveform, float * rms);
int float_rms(int16_t * waveform, float * rms);
int vector_rms(int16_t * waveform, float * rms);

int calc_snr(int16_t * waveform,float * rms,float * snr);

int float_float_rms(float * waveform, float * rms);
int calc_float_snr(float * waveform,float * rms,float * snr);


int time_delay(int16_t * wave1, int16_t * wave2, float * best_corr);
int interp_time_delay(int16_t * wave1, int16_t * wave2, float * best_corr);
int fft_time_delay(int16_t * wave1,int16_t * wave2,float * best_corr);

//gets hilbert envelope from filled comp1, and places the analytic signal in comp2. using the plans referenced. it outputs the hilbert envelope and impulsivity
int get_hilbert_env(fftwf_complex * comp1, fftwf_complex * comp2, fftwf_plan forward,fftwf_plan backward,float * hilbert_env,float * impulsivity);

//takes the ffts of two waveforms in comp1 and comp2 to calculate the shift between them
int get_time_delay_from_fft(fftwf_complex * comp1, fftwf_complex * comp2,fftwf_complex * comp3, fftwf_plan backward,int16_t *shift,float *best_corr);

//takes in squared waveforms and the delays between channels and return the intregrated power sum around the peak
int get_peak_phased_power_sum(float * wave1,float * wave2, float * wave3, float * wave4,  int * delays, float * int_pow_sum);

int template_match(fftwf_complex * comp1, fftwf_complex * comp2,fftwf_complex * comp3, fftwf_plan backward,float normalization,float *corr);

#endif

/* 
Block Offsets  
2023 RNO-G Collaboration
A. Novikov <novikov@udel.edu> 
*/ 



#include "../src/rno-g.h"

#define RNO_G_RADIANT_BLOCK_SIZE 128
#define RNO_G_NUM_RADIANT_BLOCKS 16 
#define RNO_G_NUM_RADIANT_FFT_FREQUENCIES 32

using namespace std;

#ifndef BLOCK_OFFSETS_H
#define BLOCK_OFFSETS_H


extern float freq_oob[RNO_G_NUM_RADIANT_FFT_FREQUENCIES]; 
extern const int nbIterations ; // number of iterations
extern const float simplexSize; // stopping criterium: simplex size
extern const float initialStep; // initial step
extern const float pi ;
extern const float dt; // nanoseconds
extern const float pidt; 
extern const float sqrt2dt; // = sqrt(2)*float(dt); 
extern int channel_to_process;



extern gsl_vector_complex *fit_sum;
extern fftw_complex fftw_out_oob[RNO_G_MAX_RADIANT_NSAMPLES/2 +1];
extern double dataLPF[RNO_G_MAX_RADIANT_NSAMPLES];

 



extern fftw_plan p, p_inv;
extern gsl_matrix_complex *fit;                                    //fit function (matrix), it is constant; needs to be done only once
extern double fftw_in[RNO_G_MAX_RADIANT_NSAMPLES];                    // input for FFTW
extern fftw_complex fftw_out[RNO_G_MAX_RADIANT_NSAMPLES/2 + 1];    // FFTW output
extern gsl_vector_complex *fftoob_tochi2;
extern vector<int> trace;





vector<int>         getTrace (int fromWhere, int channel_id);
gsl_vector_complex  doFFTW_OOB (vector<int> &dataIn);
void                fit_sum_f (const gsl_vector *v, float freq_oob[], gsl_matrix_complex *fit, gsl_vector_complex* fit_sum);
double              chi2_f(const gsl_vector *v, void *params); // chi2
void                chi2_df(const gsl_vector *v, void *params, gsl_vector *df); // gradient of chi2 
void                chi2_fdf(const gsl_vector *x, void *params, double *f, gsl_vector *df); // Compute both chi2_f and chi2_df together. 
vector<int>       computeOffestsGradient();
vector<float>       computeOffestsSimplex();
void                do_fftw_plan();
void                compute_fit_function();
void                prepare_fft_to_compute_offsets();
void                clean_up();
void                mallocForFFT();


#endif

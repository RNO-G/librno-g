// compile:  g++ block_offsets.C -lgsl -lgslcblas -lfftw3

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <complex.h>    
#include <vector>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h> 
#include <gsl/gsl_vector_double.h> 
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>
#include <gsl/gsl_fft_complex.h>
#include <string>
#include <fftw3.h>
#include "block_offsets.h"
//#include "../src/rno-g.h"
#include "/home/rno-g/librno-g/src/rno-g.h"

#include <chrono>


using namespace std;

gsl_complex gsl_i;



#define REAL(z,i) ((z)[2*(i)])
#define IMAG(z,i) ((z)[2*(i)+1])

float freq_oob[RNO_G_NUM_RADIANT_FFT_FREQUENCIES] = {
                                1.5625e-03,
                                3.125e-03,
                                4.6875e-03,
                                6.2500e-03,
                                7.8125e-03,
                                9.3750e-03,
                                1.09375e-02,
                                1.25e-02,
                                1.40625e-02,
                                1.5625e-02,
                                1.71875e-02,
                                1.875e-02,
                                2.03125e-02,
                                2.1875e-02,
                                2.34375e-02,
                                2.50000e-02,
                                2.65625e-02,
                                2.81250e-02,
                                2.96875e-02,
                                3.12500e-02,
                                3.28125e-02,
                                3.43750e-02,
                                3.59375e-02,
                                3.75000e-02,
                                3.90625e-02,
                                4.06250e-02,
                                4.21875e-02,
                                4.37500e-02,
                                4.531250e-02,
                                4.687500e-02,
                                4.843750e-02,
                                5.000000e-02
                                                }; 


const int nbIterations = 1000; // number of iterations
const float simplexSize = 0.8; // stopping criterium: simplex size
const float initialStep = 1.0; // initial step
const float pi = M_PI;
const float dt = 0.3125; // nanoseconds
const float pidt = M_PI*0.3125; 

const float sqrt2dt = sqrt(2)*float(dt); 
int channel_to_process;


gsl_vector_complex *fit_sum = NULL;
fftw_complex fftw_out_oob[RNO_G_MAX_RADIANT_NSAMPLES/2 +1];
double dataLPF[RNO_G_MAX_RADIANT_NSAMPLES];

fftw_plan p, p_inv;
gsl_matrix_complex *fit = NULL;                             //fit function (matrix), it is constant; needs to be done only once
double fftw_in[RNO_G_MAX_RADIANT_NSAMPLES];                 // input for FFTW
fftw_complex fftw_out[RNO_G_MAX_RADIANT_NSAMPLES/2 + 1];    // FFTW output
gsl_vector_complex *fftoob_tochi2 = NULL;

vector<int> trace;

//################################################################################################################################

void mallocForFFT()
{
  fftoob_tochi2 = gsl_vector_complex_alloc(RNO_G_NUM_RADIANT_FFT_FREQUENCIES);
}


vector<int> getTrace (int traceSource, int channel_id)
{

  vector<int> trace;
  int temp;
  int channel = channel_id;
  string filename;
  switch (traceSource)
  {
    case 0:  
    {
      filename = "/home/novikov/RNO-G/data/station11/run700/raw_txt/event_1/ch_" + std::to_string(channel)+".dat";
      break;
    }
    case 1:  
    {
      filename = "/home/novikov/RNO-G/block_offsets/comparison/ch20_art_offsets/ch_with_offsets/" + std::to_string(channel)+".dat";
      break;
    }  
    case 2:
    {
      filename = "/home/rno-g/block_offsets/data/station11/run700/raw_txt/event_1/ch_" + std::to_string(channel)+".dat"; // check
      break;
    }  
    case 3:{
      filename = "/home/rno-g/block_offsets/data/comparison/ch20_art_offsets/ch_with_offsets/" + std::to_string(channel)+".dat"; //check
    } 
    //case 4: - radiant 
    
  }

  ifstream fileWF(filename);

  while ((fileWF>>temp)) {
      trace.push_back(temp);  
      //cout<<temp<<" ";
    }
  //cout<<endl;;  
  return trace;  
}


void fit_sum_f (const gsl_vector *v, float freq_oob[], gsl_matrix_complex *fit, gsl_vector_complex* fit_sum){

  float offsets[RNO_G_NUM_RADIANT_BLOCKS];
  gsl_complex temp_sum;
  gsl_vector_complex_set_zero(fit_sum);
  for (int freq_id = 0; freq_id < RNO_G_NUM_RADIANT_FFT_FREQUENCIES; freq_id ++){
    GSL_SET_COMPLEX(&temp_sum, 0.0, 0.0);
    for (int block = 0; block < RNO_G_NUM_RADIANT_BLOCKS; block++ )
    {
        offsets[block] = gsl_vector_get(v, block); // 16 minimization offsets
        temp_sum=  gsl_complex_add( temp_sum, gsl_complex_mul_real(gsl_matrix_complex_get(fit, block, freq_id), offsets[block]));  
    }
    gsl_vector_complex_set(fit_sum, freq_id, temp_sum);  
    //cout<<GSL_REAL(temp_sum)<<endl;
    }
} 

double chi2_f(const gsl_vector *v, void *params){ // returns double value of chi2 which is then minimized 
   
  float *freq_oob = (float *)params;
  fit_sum = gsl_vector_complex_alloc(RNO_G_NUM_RADIANT_FFT_FREQUENCIES); 
  const gsl_vector_complex fft_oob_complex =*fftoob_tochi2;
  float chi2 = 0.0; 
  fit_sum_f(v , freq_oob, fit, fit_sum); // compute fit and fit_sum for both chi2_f and chi2_df 
  for (int freq_id = 0; freq_id < RNO_G_NUM_RADIANT_FFT_FREQUENCIES; freq_id ++){
      chi2 += (gsl_complex_abs2(gsl_complex_sub(gsl_vector_complex_get(fit_sum, freq_id), gsl_vector_complex_get(&fft_oob_complex, freq_id)))); 
      //cout<<chi2<<endl;
  }   
  gsl_vector_complex_free(fit_sum);
  // cout<<chi2<<endl;
  return (chi2);
}


void chi2_df(const gsl_vector *v, void *params, gsl_vector *df) // gradient of chi2
{
   
  float offsets[RNO_G_NUM_RADIANT_BLOCKS], sum_sq_fit; // offsets - vector to minimize 
  gsl_complex sum_fit;
  const gsl_vector_complex fft_oob_complex =*fftoob_tochi2;
  for (int block = 0; block < RNO_G_NUM_RADIANT_BLOCKS; block++ )
  {
      sum_sq_fit = 0.0;
      GSL_SET_COMPLEX(&sum_fit, 0, 0);
      offsets[block] = gsl_vector_get(v, block); // 16 minimization offsets
      for (int freq_id = 0; freq_id < RNO_G_NUM_RADIANT_FFT_FREQUENCIES; freq_id ++)
      {    
           sum_fit = gsl_complex_add(sum_fit, gsl_complex_mul(gsl_matrix_complex_get(fit, block, freq_id), gsl_vector_complex_get(&fft_oob_complex, freq_id)));
           sum_sq_fit += gsl_complex_abs2(gsl_matrix_complex_get(fit, block, freq_id)); 
      }
      gsl_vector_set(df, size_t(block),  2.0*(sum_sq_fit*offsets[block] -  GSL_REAL(sum_fit)));
  } 
}


/* Compute both chi2_f and chi2_df together. */
void chi2_fdf (const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
  *f = chi2_f(x, params);
  chi2_df(x, params, df);
}

vector<float> computeOffestsSimplex(){

    int i, status;
    float size;
    size_t iter = 0;
    int sum[RNO_G_NUM_RADIANT_BLOCKS] ={0};
    gsl_vector *a_guess;
    vector<float> offsets;
    a_guess = gsl_vector_alloc(RNO_G_NUM_RADIANT_BLOCKS);
    vector<int> traceForGuess = trace;
    for (i =0; i < RNO_G_NUM_RADIANT_BLOCKS; i++){
        for (int j = 0; j< RNO_G_RADIANT_BLOCK_SIZE; j++){
            sum[i]+=traceForGuess[i*RNO_G_RADIANT_BLOCK_SIZE + j];
        }
        gsl_vector_set(a_guess, i, float(sum[i]) / RNO_G_RADIANT_BLOCK_SIZE);
    }
    gsl_vector *ss;
    ss = gsl_vector_alloc (RNO_G_NUM_RADIANT_BLOCKS);
    gsl_vector_set_all (ss, initialStep);
    gsl_multimin_function chi2_to_min;
    chi2_to_min.f = &chi2_f; // function to be minimized
    chi2_to_min.n = RNO_G_NUM_RADIANT_BLOCKS;  /* number of function components */
    chi2_to_min.params =  freq_oob; // TODO: make it work with (gsl) vector, now only takes params as simple array
    const gsl_multimin_fminimizer_type *T;
    T = gsl_multimin_fminimizer_nmsimplex; //gsl_multimin_fminimizer_nmsimplex
    gsl_multimin_fminimizer *s;
    s = gsl_multimin_fminimizer_alloc (T, RNO_G_NUM_RADIANT_BLOCKS);
    gsl_multimin_fminimizer_set (s, &chi2_to_min, a_guess, ss); 

    //cout<<"minimizer set done!"<<endl;

    do
    {
      iter++;
      status = gsl_multimin_fminimizer_iterate(s);
      if (status)
        break;
      size = gsl_multimin_fminimizer_size (s);
      status = gsl_multimin_test_size (size, simplexSize);
      if (status == GSL_SUCCESS)

        {
          cout<<"converged to minimum at "<<iter<<endl;
            for (int i =0; i<RNO_G_NUM_RADIANT_BLOCKS; i++){
                cout<<gsl_vector_get (s->x, i)<<" ";
                offsets.push_back(gsl_vector_get (s->x, i));
            }
            cout<<endl;
            
        }
    }
    while (status == GSL_CONTINUE && iter < nbIterations);
   
    gsl_vector_free(a_guess);
    gsl_vector_free(ss);
    gsl_multimin_fminimizer_free (s);
    return offsets;
}

vector<int> computeOffestsGradient(){

  //auto start = chrono::high_resolution_clock::now();
  int i, j, status;
  int sum[RNO_G_NUM_RADIANT_BLOCKS] ={0};

  const double step_size = 0.2;   // works 'well' with these parameters for 1 iteration
  const double tolerance =  0.1;  // works 'well' with these parameters for 1 iteration
  //float chi2_initial;

  gsl_vector *x, *a_guess;
  vector<int> offsets;
  
  a_guess = gsl_vector_alloc(RNO_G_NUM_RADIANT_BLOCKS);
  
  const gsl_multimin_fdfminimizer_type *T;
  gsl_multimin_fdfminimizer *s;
  gsl_multimin_function_fdf chi2_to_min;

  chi2_to_min.n = RNO_G_NUM_RADIANT_BLOCKS;  /* number of function components */
  chi2_to_min.f = chi2_f; // function to be minimized
  chi2_to_min.df = chi2_df; // gradient 
  chi2_to_min.fdf = chi2_fdf;  // function + gradient 
  chi2_to_min.params =  freq_oob; // TODO(?): make it work with (gsl) vector, now only takes params as simple array

  

  //----starting point (1st guess): average of each block for low pass filtered waveform------
  //sum[RNO_G_NUM_RADIANT_BLOCKS] ={0};
  x = gsl_vector_alloc(RNO_G_NUM_RADIANT_BLOCKS);
  for (i =0; i < RNO_G_NUM_RADIANT_BLOCKS; i++){
      for (j = 1; j< RNO_G_RADIANT_BLOCK_SIZE; j++){ // ignore the 1st lpf sample
          sum[i]+=dataLPF[i*RNO_G_RADIANT_BLOCK_SIZE + j]/100/20; //TODO: understand why 20
      }
      gsl_vector_set(x, i, float(sum[i]) / RNO_G_RADIANT_BLOCK_SIZE);
      //cout<<gsl_vector_get(x, i)<<" ";
  }
  
  T = gsl_multimin_fdfminimizer_conjugate_fr; // other options here: gsl_multimin_fdfminimizer_vector_bfgs , gsl_multimin_fdfminimizer_steepest_descent
  s = gsl_multimin_fdfminimizer_alloc (T, RNO_G_NUM_RADIANT_BLOCKS);
  
  //chi2_initial = s->f;
  gsl_multimin_fdfminimizer_set (s, &chi2_to_min, x, step_size, tolerance); // double step_size, double tol !!!WORKS!!
 
  //---- on BBB we can afford only 1 iteration due to its speed----------
  status = gsl_multimin_fdfminimizer_iterate(s);
  for (i =0; i<RNO_G_NUM_RADIANT_BLOCKS; i++){
    offsets.push_back(round(gsl_vector_get(s->x, i)));
  }

  // auto elapsed = std::chrono::high_resolution_clock::now() - start;
  // long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
  // cout<<"microseconds for minimization: "<<microseconds<<" miliseconds:"<<microseconds/1000<< endl;
  
  //--- since there is no time for more than 1 iteration there is a check if this 1st iteration actually makes chi2 smaller, and if not the initial guesses are used as offsets. 
  //cout<<(s->f)<<" "<<gsl_vector_get(s->gradient, 0)<<endl;
  // if (s->f < chi2_initial) // iteration improved chi2
  // {
  //   for (i =0; i<RNO_G_NUM_RADIANT_BLOCKS; i++){
  //     offsets.push_back(round(gsl_vector_get(s->x, i)));
  //   }
  //   cout<<"iteration"<<endl;
  // }
  // else // iteration made chi2 worse
  // {
  //   for (i =0; i<RNO_G_NUM_RADIANT_BLOCKS; i++){
  //   offsets.push_back(round(float(sum[i]) / float(RNO_G_RADIANT_BLOCK_SIZE)));
  //   }
  //   cout<<"average"<<endl;
  // }

  gsl_vector_free(a_guess);
  gsl_vector_free(x);
  gsl_multimin_fdfminimizer_free (s);
  //gsl_vector_complex_free(fftoob_tochi2);
  

  return offsets;
}



void do_fftw_plan() // needs to be done once
{   
  p = fftw_plan_dft_r2c_1d(RNO_G_MAX_RADIANT_NSAMPLES, fftw_in, fftw_out, FFTW_ESTIMATE);
  p_inv = fftw_plan_dft_c2r_1d(RNO_G_MAX_RADIANT_NSAMPLES, fftw_out_oob, dataLPF, FFTW_ESTIMATE);
}


void compute_fit_function() // needs to be done only once, this fucntion is constant, may potentially be stored in a file
{
  GSL_SET_COMPLEX(&gsl_i, 0, 1); // only used here
  gsl_complex fit_temp, complex_fit_exponent;
  fit = gsl_matrix_complex_alloc(RNO_G_NUM_RADIANT_BLOCKS, RNO_G_NUM_RADIANT_FFT_FREQUENCIES);
  for (int freq_id = 0; freq_id < RNO_G_NUM_RADIANT_FFT_FREQUENCIES; freq_id ++){
    for (int block = 0; block < RNO_G_NUM_RADIANT_BLOCKS; block++ ){
      // real_fit_exponent = -2*pi * float(freq_oob[freq_id])* float(dt)  * ((block + 0.5)*float(RNO_G_RADIANT_BLOCK_SIZE) - 0.5);
      complex_fit_exponent = gsl_complex_mul_real(gsl_i, -2*pidt * (freq_oob[freq_id]) * ((block + 0.5)*(RNO_G_RADIANT_BLOCK_SIZE) - 0.5));
      //cout<<freq_oob[freq_id]<<" "<<GSL_IMAG(complex_fit_exponent)<<" ";
      fit_temp = gsl_complex_mul_real((gsl_complex_exp(complex_fit_exponent)),                 
                                                      (sqrt2dt*
                                                      sin(pidt*freq_oob[freq_id]*RNO_G_RADIANT_BLOCK_SIZE)/
                                                      sin(pidt*freq_oob[freq_id]))
                                      ); 
      gsl_matrix_complex_set(fit, block, freq_id, fit_temp);
      //cout<<freq_oob[freq_id]<<" "<<GSL_REAL(fit_temp)<<endl;
    }
  }
}  

void prepare_fft_to_compute_offsets() // prepare FFT vectors for offsets computation 
{
  int i; 
  gsl_complex ztemp;
  // fftoob_tochi2 = gsl_vector_complex_alloc(RNO_G_NUM_RADIANT_FFT_FREQUENCIES);
  for (i = 1; i < RNO_G_NUM_RADIANT_FFT_FREQUENCIES+1; i++)
        {
          GSL_SET_COMPLEX(&ztemp, 20*fftw_out[i][0]/sqrt(RNO_G_MAX_RADIANT_NSAMPLES), fftw_out[i][1]/sqrt(RNO_G_MAX_RADIANT_NSAMPLES)); // TODO: understand why normalization factor is 20
          gsl_vector_complex_set(fftoob_tochi2, i-1, ztemp);
             
        }     
  for (i = 0; i < RNO_G_NUM_RADIANT_FFT_FREQUENCIES; i++)
        {
          fftw_out_oob[i][0] = fftw_out[i][0];
          fftw_out_oob[i][1] = fftw_out[i][1];
        }
        
  for (i = RNO_G_NUM_RADIANT_FFT_FREQUENCIES+1; i < RNO_G_MAX_RADIANT_NSAMPLES/2 +1; i++) // filter out 'high' frequencies
        {
          fftw_out_oob[i][0] = 0;
          fftw_out_oob[i][1] = 0; 
        }
  //gsl_vector_complex_free(fftoob_tochi2);

}

void clean_up()
{
  fftw_destroy_plan(p);
  fftw_destroy_plan(p_inv);
  gsl_matrix_complex_free(fit);
  gsl_vector_complex_free(fftoob_tochi2);
  fftw_cleanup();
 
}

/*
//----FFTW------
gsl_vector_complex doFFTW_OOB (vector<int> &dataIn)
{
    int i;
    fftw_complex fftw_out[RNO_G_MAX_RADIANT_NSAMPLES/2 + 1];
    gsl_complex ztemp;
    gsl_vector_complex *fft = NULL;
    
    // FFTW needs doubles
    for (i = 0; i < RNO_G_MAX_RADIANT_NSAMPLES; i++)
    {
        fftw_in[i] = double(dataIn[i]);
    }
   
    fft = gsl_vector_complex_alloc(RNO_G_NUM_RADIANT_FFT_FREQUENCIES);
    fftw_execute(p);
    for (i = 0; i < RNO_G_NUM_RADIANT_FFT_FREQUENCIES; i++)
    {
        //printf(" %+9.5f %+9.5f\n", 20*fftw_out[i][0]/sqrt(RNO_G_MAX_RADIANT_NSAMPLES), 20*fftw_out[i][1]/20/sqrt(RNO_G_MAX_RADIANT_NSAMPLES));
        GSL_SET_COMPLEX(&ztemp, 20*fftw_out[i][0]/sqrt(RNO_G_MAX_RADIANT_NSAMPLES), 20*fftw_out[i][1]/20/sqrt(RNO_G_MAX_RADIANT_NSAMPLES)); // TODO: understand why normalization factor is 20
        //cout<<gsl_complex_abs(ztemp)<<endl;
        fftw_out_oob[i][0] = fftw_out[i][0];
        fftw_out_oob[i][1] = fftw_out[i][1];
        gsl_vector_complex_set(fft, i-1, ztemp);
        
    }


  //  fftw_execute(p_inv);
     for (i = 1; i < RNO_G_NUM_RADIANT_FFT_FREQUENCIES + 1; i++)
    {
        cout<<i<<": "<<dataLPF[i];
        cout<<endl;    
    }
    

    return *fft;
}
*/



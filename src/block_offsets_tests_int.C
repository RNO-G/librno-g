// compile:  g++ block_offsets_tests_int.C -lgsl -lgslcblas -lfftw3

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
#include <chrono>
#include <string>
#include <fftw3.h>
// #include <Eigen/Core>
// #include <LBFGS.h> -- for https://lbfgspp.statr.me/

#define REAL(z,i) ((z)[2*(i)])
#define IMAG(z,i) ((z)[2*(i)+1])
using namespace std;
ofstream fileOut, fileOutTime;

// sampling rate=  3.2
// dt=  0.3125

const int block_size=128, trace_len = 2048, nb_freq = 32, nb_blocks = 16;
const int nbIterations = 100; // number of iterations
const float simplexSize = 0.8; // stopping criterium: simplex size
const float initialStep = 1.0; // initial step
const float pi = M_PI;
const float dt = 0.3125; // nanoseconds
int channel_to_process;
vector<int> trace;
gsl_vector_complex fftoob_tochi2;
const int voltageFraction = 10000; // 1 - signals in volts; 1000 - signals in milivolts

float freq_oob[nb_freq] = {
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

vector<int> getTrace (int channel_id)
{
    vector<int> trace;
    int temp;
    int channel = channel_id;
    int sample = 0;
    //ifstream fileWF("comparison/all_traces_24chs_11-700-2.txt");
    string filename = "/home/novikov/RNO-G/data/station11/run700/raw_txt/ch_" + std::to_string(channel)+".dat";
   // string filename = "/home/rno-g/block_offsets/data/station11/run700/raw_txt/ch_" + std::to_string(channel)+".dat";
    ifstream fileWF(filename);

    while ((fileWF>>temp)) {
            trace.push_back(temp);  
        }
  cout<<endl;;  
  return trace;  
}


//Radix-2 FFT routines for complex data -- do FFT
gsl_vector_complex doFFTOOB (vector<int> dataIn)

{
    const int freqLim = 32; //  32 corresponds to 50MHz
    int i; double data[trace_len*2] = {0};
    gsl_vector_complex *fft = NULL;
    fft = gsl_vector_complex_alloc(freqLim); 
    gsl_complex ztemp;
    auto start = chrono::high_resolution_clock::now();
    for (i = 0; i < trace_len; i++)
    {
        REAL(data,i) = dataIn[i];
    }
    gsl_fft_complex_radix2_forward (data, 1, trace_len);
    for (i = 1; i < freqLim+1; i++) // 32 corresponds to 50MHz
    {
        printf(" %+9.5f %+9.5f\n", 20*REAL(data,i)/sqrt(trace_len), 20*IMAG(data,i)/sqrt(trace_len));
        GSL_SET_COMPLEX(&ztemp, 20*REAL(data,i)/sqrt(trace_len), 20*IMAG(data,i)/sqrt(trace_len)); // TODO: understand why normalization factor is 20
        gsl_vector_complex_set(fft, i-1, ztemp);
    }
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    cout<<"microseconds for FFT: "<<microseconds<<endl; 
    //fileOutTime<<microseconds<<endl;
  return *fft;
}

//----FFTW------
gsl_vector_complex doFFTW_OOB (vector<int> dataIn)
{
    fftw_complex data[trace_len], fftw_out[trace_len];
    fftw_plan p;
    gsl_complex ztemp;
    gsl_vector_complex *fft = NULL;
     fft = gsl_vector_complex_alloc(nb_freq);
    int i;
    auto start = chrono::high_resolution_clock::now();
    for (i = 0; i < trace_len; i++)
    {
        data[i][0] = dataIn[i];
        data[i][1] = 0;
    }
    p = fftw_plan_dft_1d(trace_len, data, fftw_out, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(p);
    for (i = 1; i < nb_freq + 1; i++)
    {
        //printf(" %+9.5f %+9.5f\n", 20*fftw_out[i][0]/sqrt(trace_len), 20*fftw_out[i][1]/20/sqrt(trace_len));
        GSL_SET_COMPLEX(&ztemp, 20*fftw_out[i][0]/sqrt(trace_len), 20*fftw_out[i][1]/20/sqrt(trace_len)); // TODO: understand why normalization factor is 20
        //cout<<gsl_complex_abs(ztemp)<<endl;
        gsl_vector_complex_set(fft, i-1, ztemp);
    }
        
    fftw_destroy_plan(p);
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    cout<<"microseconds for FFT: "<<microseconds<<endl; 
    return *fft;
}

 void fit_sum_f (const gsl_vector *v, float freq_oob[], gsl_matrix_complex *fit, gsl_vector_complex* fit_sum){

  float A[trace_len/block_size];
  gsl_complex gsl_i, complex_fit_exponent, temp_sum, fit_temp, firstTerm, secondTerm, thirdTerm, sum_sq_fit, sum_fit;
  GSL_SET_COMPLEX(&gsl_i, 0, 1);
  float real_fit_exponent, chi2;
  chi2 = 0.0;
   
  for (int block = 0; block < nb_blocks; block++ ){
      A[block] = gsl_vector_get(v, block); // 16 minimization offsets
      //cout<<"A: "<<A[block]<<" ";
      for (int freq_id = 0; freq_id < nb_freq; freq_id ++){
          real_fit_exponent = -2*pi * float(freq_oob[freq_id])* float(dt)  * ((block + 0.5)*float(block_size) - 0.5);
          complex_fit_exponent = gsl_complex_mul_real(gsl_i, real_fit_exponent);
          fit_temp = gsl_complex_mul_real((gsl_complex_exp(complex_fit_exponent)), 
                                                          (
                                                          sqrt(2)*float(dt)*
                                                          sin(pi*freq_oob[freq_id]*float(dt)*float(block_size))/
                                                          sin(pi*freq_oob[freq_id]*float(dt)))
                                                      ); 
          gsl_matrix_complex_set(fit, block, freq_id, fit_temp);
          
        }
    }
      
    // fit_sum --------------------------------------------------
  
    gsl_vector_complex_set_zero(fit_sum);
    for (int freq_id = 0; freq_id < nb_freq; freq_id ++){
        GSL_SET_COMPLEX(&temp_sum, 0.0, 0.0);
        for (int block = 0; block < nb_blocks; block++ ){
             temp_sum=  gsl_complex_add( temp_sum, gsl_complex_mul_real(gsl_matrix_complex_get(fit, block, freq_id), A[block]));
        }
        gsl_vector_complex_set(fit_sum, freq_id, temp_sum);    
    }
} 

double chi2_f(const gsl_vector *v, void *params){ // returns double value of chi2 which is then minimized

    float A[nb_blocks]; // offsets - vector to minimize
    //float dA[trace_len/block_size]; // gradient
    float *freq_oob = (float *)params;
    gsl_vector_complex *fit_sum ;
    fit_sum = gsl_vector_complex_alloc(nb_freq); 
    gsl_matrix_complex *fit;
    fit = gsl_matrix_complex_alloc(nb_blocks, nb_freq);
    gsl_complex gsl_i, complex_fit_exponent, temp_sum, fit_temp, firstTerm, secondTerm, thirdTerm, sum_sq_fit, sum_fit;
    GSL_SET_COMPLEX(&gsl_i, 0, 1);
   // const gsl_vector_complex fft_oob_complex = getFFTOOB(channel_to_process);
    const gsl_vector_complex fft_oob_complex =fftoob_tochi2;
    float real_fit_exponent, chi2; 
    chi2= 0.0;
    fit_sum_f(v , freq_oob, fit, fit_sum);
    for (int freq_id = 0; freq_id < nb_freq; freq_id ++){
        chi2 += gsl_complex_abs2(gsl_complex_sub(gsl_vector_complex_get(fit_sum, freq_id), gsl_vector_complex_get(&fft_oob_complex, freq_id)));
    }   
    //cout<<"chi2: "<<chi2<<endl;
    // gsl_vector_complex_free(&fit_sum);
    //gsl_matrix_complex_free(fit);
    return (chi2);

}

double chi2_f_BACKUP(const gsl_vector *v, void *params){ // returns double value of chi2 which is then minimized

    float A[nb_blocks]; // offsets - vector to minimize
    //float dA[trace_len/block_size]; // gradient
    float *freq_oob = (float *)params;
    gsl_vector_complex *fit_sum = NULL;
    fit_sum = gsl_vector_complex_alloc(nb_freq); 
    gsl_matrix_complex *fit = NULL;
    fit = gsl_matrix_complex_alloc(nb_blocks, nb_freq);
    gsl_complex gsl_i, complex_fit_exponent, temp_sum, fit_temp, firstTerm, secondTerm, thirdTerm, sum_sq_fit, sum_fit;
    GSL_SET_COMPLEX(&gsl_i, 0, 1);
   // const gsl_vector_complex fft_oob_complex = getFFTOOB(channel_to_process);
    const gsl_vector_complex fft_oob_complex =fftoob_tochi2;
    float real_fit_exponent, chi2;

    //----chi2-------------------------
  
    chi2 = 0.0;
    cout<<"000"<<endl;
    for (int block = 0; block < nb_blocks; block++ ){
        A[block] = gsl_vector_get(v, block); // 16 minimization offsets
        for (int freq_id = 0; freq_id < nb_freq; freq_id ++)
        {
            real_fit_exponent = -2*pi * float(freq_oob[freq_id])* float(dt)  * ((block + 0.5)*float(block_size) - 0.5);
            complex_fit_exponent = gsl_complex_mul_real(gsl_i, real_fit_exponent);
            fit_temp = gsl_complex_mul_real((gsl_complex_exp(complex_fit_exponent)), 
                                                            (
                                                            sqrt(2)*float(dt)*
                                                            sin(pi*freq_oob[freq_id]*float(dt)*float(block_size))/
                                                            sin(pi*freq_oob[freq_id]*float(dt)))
                                                        ); 
            gsl_matrix_complex_set(fit, block, freq_id, fit_temp);
            cout<<   "fit  "<< gsl_complex_abs(fit_temp)<<endl; 
          
        }
    }
    
    // fit_sum --------------------------------------------------
    gsl_vector_complex_set_zero(fit_sum);
     cout<<"001"<<endl;
    for (int freq_id = 0; freq_id < nb_freq; freq_id ++){
        GSL_SET_COMPLEX(&temp_sum, 0.0, 0.0);
        for (int block = 0; block < nb_blocks; block++ ){
             temp_sum=  gsl_complex_add( temp_sum, gsl_complex_mul_real(gsl_matrix_complex_get(fit, block, freq_id), A[block]));
                //cout<<" temp_sum:  "<< gsl_complex_abs(temp_sum) <<" A[block]: "<<A[block]<< endl;
        }
        gsl_vector_complex_set(fit_sum, freq_id, temp_sum); 
        
    }

      chi2= 0.0;
     cout<<"002"<<endl;  
    // chi2 ------------------------------------------
    for (int freq_id = 0; freq_id < nb_freq; freq_id ++){
      /*  chi2 += gsl_complex_abs(gsl_complex_sub(gsl_vector_complex_get(fit_sum, freq_id), gsl_vector_complex_get(&fft_oob_complex, freq_id)))*
                gsl_complex_abs(gsl_complex_sub(gsl_vector_complex_get(fit_sum, freq_id), gsl_vector_complex_get(&fft_oob_complex, freq_id))); */
       //cout<<freq_id<<" chi2: "<<chi2<<" dif= "<<(gsl_complex_abs(gsl_vector_complex_get(fit_sum, freq_id)) - gsl_complex_abs(gsl_vector_complex_get(&fft_oob_complex, freq_id)))<<endl;

       // --- prepare for gradient:
 
       firstTerm = gsl_complex_mul(gsl_vector_complex_get(fit_sum, freq_id), gsl_vector_complex_get(fit_sum, freq_id));
       secondTerm = gsl_complex_mul_real(gsl_complex_mul(gsl_vector_complex_get(fit_sum, freq_id), gsl_vector_complex_get(&fft_oob_complex, freq_id)), 2.0);
       thirdTerm = gsl_complex_mul(gsl_vector_complex_get(&fft_oob_complex, freq_id), gsl_vector_complex_get(&fft_oob_complex, freq_id));

       chi2 += gsl_complex_abs(gsl_complex_add(gsl_complex_sub(firstTerm, secondTerm), thirdTerm));
       
       //--------gradient, dchi2/dAvect 

    }   
    
gsl_vector_complex_free(fit_sum);
gsl_matrix_complex_free(fit);

  cout<<"chi2: "<<chi2<<endl;
    return (chi2);

}

void chi2_df(const gsl_vector *v, void *params, gsl_vector *df)
{
    float A[nb_blocks]; // offsets - vector to minimize
    //float dA[trace_len/block_size]; // gradient
    float *freq_oob = (float *)params;
    gsl_vector_complex *fit_sum ;
    fit_sum = gsl_vector_complex_alloc(nb_freq); 
    gsl_matrix_complex *fit;
    fit = gsl_matrix_complex_alloc(nb_blocks, nb_freq);
    gsl_complex gsl_i, complex_fit_exponent, temp_sum, fit_temp, firstTerm, secondTerm, thirdTerm, sum_sq_fit, sum_fit;
    GSL_SET_COMPLEX(&gsl_i, 0, 1);
    const gsl_vector_complex fft_oob_complex =fftoob_tochi2;
    float real_fit_exponent, chi2;

    //------------------------------------------------------------

    GSL_SET_COMPLEX(&sum_sq_fit, 0, 0);
    GSL_SET_COMPLEX(&sum_fit, 0, 0);

    fit_sum_f(v , freq_oob, fit, fit_sum);

    for (int block = 0; block < nb_blocks; block++ ){
      A[block] = gsl_vector_get(v, block); // 16 minimization offsets
        for (int freq_id = 0; freq_id < nb_freq; freq_id ++){
           sum_sq_fit  = gsl_complex_add(sum_sq_fit, gsl_complex_mul(gsl_matrix_complex_get(fit, block, freq_id), gsl_matrix_complex_get(fit, block, freq_id)));
           sum_fit = gsl_complex_add(sum_fit, gsl_matrix_complex_get(fit, block, freq_id));
            // cout<<freq_id<<" sum_sq_fit= "<<gsl_complex_abs(sum_sq_fit)<<" sum_fit= "<<gsl_complex_abs(sum_fit)<<endl;
           // cout<<freq_id<<" fit_sum: "<<gsl_complex_abs(gsl_vector_complex_get(fit_sum, freq_id))<<
           // " fit: "<<gsl_complex_abs( gsl_matrix_complex_get(fit, block, freq_id))<<endl;
        }
        gsl_vector_set(df, size_t(block),  GSL_REAL(gsl_complex_sub(gsl_complex_mul_real(sum_sq_fit, 2*A[block]), gsl_complex_mul_real(sum_fit, 2.0))));
       
        //cout<<"dA["<<block<<"]: "<<gsl_vector_get(df, block)<<" real(sum_sq_fit): "<<GSL_REAL(sum_sq_fit)<<" real(sum_fit): "<<GSL_REAL(sum_fit)<<" A: "<<A[block]<<endl;
        //cout<<size_t(block)<<endl;
    } 
    cout<<"222"<<endl;
}


/* Compute both chi2_f and chi2_df together. */
void chi2_fdf (const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
  cout<<"start chi2_fdf"<<endl;
  *f = chi2_f(x, params);
  cout<<"f done"<<endl;
  chi2_df(x, params, df);
  cout<<"df done"<<endl;
}

vector<float> computeOffestsSimplex(){

    int sample, i, status;
    float temp, temp2, chi2, size;
    size_t iter = 0;
    int sum[nb_blocks] ={0};
    gsl_vector *a_guess;
    vector<float> offsets;
    a_guess = gsl_vector_alloc(nb_blocks);
    vector<int> traceForGuess = trace;
    cout<<"Initialization..."<<endl;
    for (i =0; i < nb_blocks; i++){
        for (int j = 0; j< block_size; j++){
            sum[i]+=traceForGuess[i*block_size + j];
        }
        gsl_vector_set(a_guess, i, float(sum[i]) / block_size);
        //cout<<i<<" a_gues: "<<double(double(sum[i]) / double(block_size))<<" sum[i]: "<<sum[i]<<endl;
    }
    cout<<"guess done"<<endl;
    gsl_vector *ss;
    ss = gsl_vector_alloc (nb_blocks);
    gsl_vector_set_all (ss, initialStep);
    gsl_multimin_function chi2_to_min;
    chi2_to_min.f = &chi2_f; // function to be minimized
    chi2_to_min.n = nb_blocks;  /* number of function components */
    chi2_to_min.params =  freq_oob; // TODO: make it work with (gsl) vector, now only takes params as simple array
    const gsl_multimin_fminimizer_type *T;
    T = gsl_multimin_fminimizer_nmsimplex2; //gsl_multimin_fminimizer_nmsimplex
    gsl_multimin_fminimizer *s;
    s = gsl_multimin_fminimizer_alloc (T, nb_blocks);
    gsl_multimin_fminimizer_set (s, &chi2_to_min, a_guess, ss); 

    cout<<"minimizer set done!"<<endl;

    auto start = chrono::high_resolution_clock::now();
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
            for (int i =0; i<nb_blocks; i++){
                cout<<gsl_vector_get (s->x, i)<<" ";
                fileOut<<gsl_vector_get (s->x, i)<<" ";
                offsets.push_back(gsl_vector_get (s->x, i));
            }
            cout<<endl;
            fileOut<<endl;
        }
    }
    while (status == GSL_CONTINUE && iter < nbIterations);
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    cout<<"microseconds: "<<microseconds<<endl;
    //fileOutTime<<iter<<" "<<microseconds<<endl;
    gsl_vector_free(a_guess);
    gsl_vector_free(ss);
    gsl_multimin_fminimizer_free (s);
    return offsets;
}

vector<float> computeOffestsGradient(){

    int sample, i;
    float temp, temp2;   
    float  chi2;
    size_t iter = 0;
    int status;
    float size;
    int sum[nb_blocks] ={0};
    gsl_vector *x;
    gsl_vector *a_guess;
    vector<float> offsets;
    a_guess = gsl_vector_alloc(nb_blocks);
   
    vector<int> traceForGuess = trace;
    cout<<"Initialization Gradient..."<<endl;
  

    const gsl_multimin_fdfminimizer_type *T;
    gsl_multimin_fdfminimizer *s;
    
    
   
    gsl_multimin_function_fdf chi2_to_min;

    cout<<"minimizer is being set"<<endl;
 
    chi2_to_min.n = nb_blocks;  /* number of function components */
    chi2_to_min.f = chi2_f; // function to be minimized
    chi2_to_min.df = chi2_df;
    chi2_to_min.fdf = chi2_fdf;  
    chi2_to_min.params =  freq_oob; // TODO: make it work with (gsl) vector, now only takes params as simple array

    //----starting point------
    x = gsl_vector_alloc(nb_blocks);
    for (i =0; i < nb_blocks; i++){
        for (int j = 0; j< block_size; j++){
            sum[i]+=traceForGuess[i*block_size + j];
        }
        gsl_vector_set(x, i, double(sum[i]) / block_size);
        //gsl_vector_set(x, i, 3.0);
        //cout<<i<<" a_gues: "<<double(double(sum[i]) / double(block_size))<<" sum[i]: "<<sum[i]<<endl;
    }
    cout<<"guess done"<<endl;
    T = gsl_multimin_fdfminimizer_conjugate_fr;
    //T = gsl_multimin_fdfminimizer_vector_bfgs; 
    s = gsl_multimin_fdfminimizer_alloc (T, nb_blocks);
    gsl_multimin_fdfminimizer_set (s, &chi2_to_min, x, 0.1, 1e-3); 
    cout<<"initial gradient: "<<gsl_vector_get(s->gradient, 0)<<endl;
    cout<<"minimizer set done!"<<endl;

    auto start = chrono::high_resolution_clock::now();
    do
    {
      iter++;
     cout<<"gradient: "<<gsl_vector_get(s->gradient, 0)<<endl;  
     status = gsl_multimin_fdfminimizer_iterate(s);
      if (status){
        cout<<"break!, status: "<<status<<endl;
        //break;
      }
        
    
    status = gsl_multimin_test_gradient (s->gradient, 1e-2);

    cout<<iter<<" "<<gsl_vector_get (s->x, 0)<<" "<<gsl_vector_get (s->x, 1)<<" "<<
                     gsl_vector_get (s->x, 2)<<" "<<gsl_vector_get (s->x, 3)<<" "<<
                     gsl_vector_get (s->x, 4)<<" "<<gsl_vector_get (s->x, 5)<<" "<<
              (s->f)<<endl;

      if (status == GSL_SUCCESS)
        {
          cout<<"converged to minimum at "<<iter<<endl;
            for (int i =0; i<nb_blocks; i++){
                cout<<gsl_vector_get (s->x, i)<<" ";
                fileOut<<gsl_vector_get (s->x, i)<<" ";
                offsets.push_back(gsl_vector_get (s->x, i));
            }
            cout<<endl;
            fileOut<<endl;
        }
    }
    while (status == GSL_CONTINUE && iter < nbIterations);
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    cout<<iter<<" microseconds: "<<microseconds<<endl;
    fileOutTime<<iter<<" "<<microseconds<<endl;
    gsl_vector_free(a_guess);
    gsl_vector_free(x);
    gsl_multimin_fdfminimizer_free (s);
    return offsets;
}

void test_gradient()
{
  gsl_vector  *x;
  x = gsl_vector_alloc (nb_blocks);
  gsl_vector *df;
  df = gsl_vector_alloc(nb_blocks);
  double *f ;
  int sum[nb_blocks] ={0};
  vector<int> traceForGuess = trace;
  for (int i =0; i < nb_blocks; i++){
        for (int j = 0; j< block_size; j++){
            sum[i]+=traceForGuess[i*block_size + j];
        }
        
        gsl_vector_set(x, i, float(sum[i]) / block_size );
      // gsl_vector_set(x, i, -0.01 );
        //cout<<i<<" a_gues: "<<double(double(sum[i]) / double(block_size))<<" sum[i]: "<<sum[i]<<endl;
    }

  chi2_fdf(x, freq_oob, f, df);

  for (int i =0; i< nb_blocks; i++){
    cout<<i<<": "<<gsl_vector_get(df, i)<<endl;
  }

  cout<<"f: "<<*f<<endl;

}

int main()
{
    fileOut.open("offsets_out_int_11-700-2_config.dat");
    fileOutTime.open("outMicroSeconds_config.dat");
    int channel;
    gsl_vector  *x;
    x = gsl_vector_alloc (nb_blocks);
    auto start = chrono::high_resolution_clock::now();
    for (channel = 0; channel< 24; channel++){
        channel_to_process = channel;  
        cout<<"channel: "<<channel<<endl;
        trace = getTrace(channel_to_process);
        fileOutTime<<channel<<" ";
        //fftoob_tochi2 = doFFTOOB(trace);
        fftoob_tochi2 = doFFTW_OOB(trace);
        fileOut<<channel<<" ";

        //  cout<<"x done"<<endl;
        gsl_vector *df;
        df  = gsl_vector_alloc(nb_freq);
        int sum[nb_blocks] ={0};
        vector<int> traceForGuess = trace;
        for (int i =0; i < nb_blocks; i++){
            for (int j = 0; j< block_size; j++){
                sum[i]+=traceForGuess[i*block_size + j];
            }
        gsl_vector_set(x, i, float(sum[i]) / block_size);
        //cout<<i<<" a_gues: "<<double(double(sum[i]) / double(block_size))<<" sum[i]: "<<sum[i]<<endl;
        }


        computeOffestsSimplex();
        //computeOffestsGradient();
        cout<<"###########################################"<<endl;
    } 

    //test_gradient();
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    cout<<"grand total microseconds: "<<microseconds<<endl;
    fileOut.close();
    fileOutTime.close();
    return 0;
 }  
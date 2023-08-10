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
#include <chrono>
#include <string>
#include <fftw3.h>
#include "block_offsets.h"

using namespace std;

int main()
{
  enum traceSource{
    testLaptopAllChannels,
    testLaptopArtificialOffsets,
    testBBBAllChannels,
    testBBBArtificialOffsets,
    radiant
  }
  
  traceSource = testBBBAllChannels;

  bool printout = false;
  bool artificial_offsets = false;
  vector<int> offsets;

  int nbTraces;
  if (artificial_offsets) nbTraces = 10000;
  else nbTraces = RNO_G_NUM_RADIANT_CHANNELS;
  
  
  int channel, i;
  auto start = chrono::high_resolution_clock::now();

  
  compute_fit_function(); // needs to be done only once, this fucntion is constant, may potentially be stored in a file
  do_fftw_plan();
  
  mallocForFFT();
  //########## looping over waveforms starts here ############################
  
  for (channel = 0; channel< nbTraces; channel++)
  { 
    if (printout) cout<<"channel: "<<channel<<endl;
    trace = getTrace(traceSource, channel);
    for (i = 0; i < RNO_G_MAX_RADIANT_NSAMPLES; i++)
    {
      fftw_in[i] = double(trace[i]);
    }

    fftw_execute(p);
    prepare_fft_to_compute_offsets();    
    fftw_execute(p_inv);   
    offsets = computeOffestsGradient();
    //computeOffestsSimplex();

    if (printout)
    {
      for (i =0; i<RNO_G_NUM_RADIANT_BLOCKS; i++) cout<<offsets[i]<<" ";
      cout<<endl;
    }    
  } 

  //##############################################################################
  clean_up();

//----total time---------------------
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
  cout<<"total microseconds: "<<microseconds<<" miliseconds:"<<microseconds/1000<< endl;
//--------------------------------------------
  return 0; 
 }  


    //     auto elapsed = std::chrono::high_resolution_clock::now() - start;
    // long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    // cout<<"microseconds for get trace: "<<microseconds<<" miliseconds:"<<microseconds/1000<< endl;
    // totalMicrosendsTrace += microseconds; 

    //   auto start = chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::high_resolution_clock::now() - start;
    // long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    // cout<<"microseconds for FFT: "<<microseconds<<endl; 
        // if (printout) fileOut.open("offsets_out_int_11-700-1-gradient-ver1-0.dat");
    // if (artificial_offsets) fileOut.open("ch20_art_computed.dat");
    // fileOutGuess.open("offsets_out_int_11-700-1-gradient-guess.dat");
    //fileOutTime.open("outMicroSeconds_config.dat");
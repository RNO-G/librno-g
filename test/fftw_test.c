//fftw performance test 
// LDFLAGS+=-lfftw3f make fftw_test 

#include <fftw3.h> 
#include <stdio.h> 
#include <stdint.h> 
#include <time.h> 
#include <stdlib.h> 

#define NCHAN 24 
#define NSAMP 1024

static uint16_t fake_data[NCHAN][NSAMP]; 


int main(int nargs, char ** args) 
{

  //fill in fake_data 
  FILE * frandom = fopen("/dev/urandom","r"); 
  fread(fake_data, NSAMP*2, NCHAN, frandom); 
  fclose(frandom);

  int ntimes = 20; 
  if (nargs > 1) ntimes = atoi(args[1]); 

  float * x = fftwf_alloc_real(NSAMP); 
  fftwf_complex * X = fftwf_alloc_complex(NSAMP); 

  fftwf_import_wisdom_from_filename("wisdom.dat"); 

  fftwf_plan fw = fftwf_plan_dft_r2c_1d(NSAMP, x, X, FFTW_MEASURE); 
  fftwf_plan bw = fftwf_plan_dft_c2r_1d(NSAMP, X, x, FFTW_MEASURE); 

  fftwf_export_wisdom_to_filename("wisdom.dat");
  struct timespec start;
  struct timespec end;

  clock_gettime(CLOCK_MONOTONIC,&start);

  for (int i = 0; i < ntimes; i++) 
  {
    for (int ichan = 0; ichan < NCHAN; ichan++) 
    {
      for (int j = 0; j < NSAMP; j++) 
      {
        x[j] = fake_data[ichan][j]; 
      }

      fftwf_execute(fw); 
      fftwf_execute(bw); 
    }
  }

  clock_gettime(CLOCK_MONOTONIC,&end);
  double dt = (end.tv_sec - start.tv_sec) + 1e-9 * (end.tv_nsec - start.tv_nsec); 

  printf("Processed %d \"events\" in %g (%g Hz, %g channels/second)\n", ntimes, dt, ntimes/dt, ntimes*NCHAN/dt);



}

#include "radiant.h" 
#include <stdio.h>
#include <stdlib.h> 



int main(int nargs, char ** args) 
{
  float freq = 150;
  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant");
  if (!rad) return 1; 
  if (nargs < 2) 
  {
    printf("Using default frequency: %f MHz\n", freq); 
  }
  else
  {
    float try = atof(args[1]); 
    if (try) freq = try; 
    printf("Using frequency: %f MHz\n", freq); 
  }


  float actual = 0;
  radiant_set_frequency(rad, freq,&actual) ; 
  printf("Actual freq: %f MHz\n", actual); 

  radiant_close(rad); 
}



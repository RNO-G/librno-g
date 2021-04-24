#include "radiant.h" 
#include <stdio.h>




int main(void) 
{
  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant");
  if (!rad) return 1; 

  radiant_dump(rad, 0, RADIANT_DUMP_UPDATE_GPIOS) ; 

  radiant_close(rad); 


}



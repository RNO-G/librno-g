#include "radiant.h" 
#include <stdio.h>
#include <string.h> 
#include <unistd.h> 




int main(void) 
{
  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant", 46,-61); //not sure if right gpio yet 
  if (!rad) return 1; 
  rno_g_daqstatus_t ds; 
  radiant_read_daqstatus(rad,&ds);
  rno_g_daqstatus_dump(stdout,&ds); 
  radiant_close(rad); 
}



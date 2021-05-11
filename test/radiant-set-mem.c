#include "radiant.h" 
#include <stdio.h>
#include <stdlib.h> 




int main(int nargs, char ** args) 
{
  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant",0,-61);
 
  if (nargs < 3) 
  {
    printf("usage: radiant_set_mem addr0 addr val\n"); 
  }
  int addr = strtol(args[1],0,0); 
  int val = strtol(args[2],0,0); 
  radiant_set_mem(rad, addr > 0x400000 ? DEST_MANAGER : DEST_FPGA, addr & 0xffffff, 4,(uint8_t*) &val); 
  printf("setting 0x%x to %x\n", addr, val); 
  radiant_close(rad); 
  return 0; 

}

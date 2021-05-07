#include "radiant.h" 
#include <stdio.h>
#include <stdlib.h> 




int main(int nargs, char ** args) 
{
  radiant_dev_t * rad = radiant_open("/dev/spi/0.0", "/dev/ttyRadiant",0);
 
  if (nargs < 2) 
  {
    printf("usage: radiant_get_mem addr0 [addr1 ... ]\n"); 
  }
  for (int i = 1; i < nargs; i++) 
  {
    int addr = strtol(args[i],0,0); 
    uint8_t mem[4]; 
    radiant_get_mem(rad, addr > 0x400000 ? DEST_MANAGER : DEST_FPGA, addr & 0xffffff, 4, mem); 
    printf("0x%x: %x %x %x %x\n", addr, mem[3],mem[2],mem[1],mem[0]); 
  }
  radiant_close(rad); 
  return 0; 

}

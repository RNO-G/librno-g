// don't try this at home
// get gpio state from /dev/mem
// since this seems to be the easiest way to check the SD card detect

#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/stat.h>


static int gpio_bank = 0;
static int gpio_num = 6; 

#define GPIO0_BASE  0x44E07000
#define GPIO1_BASE  0x4804C000
#define GPIO2_BASE  0x481AC000
#define GPIO3_BASE  0x481AE000

#define GPIO_SIZE 4096

#define GPIO_OE 0x134
#define GPIO_DIN 0x138
#define GPIO_DOUT 0x13c




static off_t gpio_base[4] = { GPIO0_BASE, GPIO1_BASE, GPIO2_BASE, GPIO3_BASE }; 

int main(int nargs, char ** args) 
{
  if (nargs > 1) 
  {
    int maybe_gpio_bank = -1;
    int maybe_gpio_num = -1; 
    sscanf(args[1], "%d.%d", &maybe_gpio_bank, &maybe_gpio_num); 
    if (maybe_gpio_bank < 0 || maybe_gpio_bank > 3 || maybe_gpio_num < 0 || maybe_gpio_num > 31)
    {
      fprintf(stderr,"usage: raw_gpio BANK.NUM\n"); 
      return 1;
    }
    gpio_bank = maybe_gpio_bank;
    gpio_num = maybe_gpio_num;


  }

  int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);;

  if (mem_fd < 0) 
  {
    fprintf(stderr,"Could not open /dev/mem\n"); 
    return 1; 
  }

  char * gpio_map = mmap(0, GPIO_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, gpio_base[gpio_bank]); 
  if (MAP_FAILED == gpio_map)
  {
    fprintf(stderr,"Couldn't mmap\n"); 
    return 1; 
  }

  volatile unsigned * oe = (volatile unsigned*) (gpio_map + GPIO_OE); 
  volatile unsigned * din = (volatile unsigned*) (gpio_map + GPIO_DIN); 
 
  printf("GPIO %d.%d:  OE: %d, DIN:%d \n", !!(*oe & (1 << gpio_num)), !!(*din && (1 << gpio_num))); 

}


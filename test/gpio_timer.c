
#include <time.h> 
#include <sys/types.h>
#include <fcntl.h>
#include <sys/stat.h> 
#include <unistd.h> 
#include <stdlib.h>
#include <stdio.h> 
#include <signal.h> 
#include <string.h>
#include <poll.h> 

int gpio = 66;



static char edge_path[512]; 
static char val_path[512]; 


static volatile int do_quit = 0;
void handler(int sig) 
{
  (void) sig; 
  do_quit=1; 
}


static char buf[128];
int main(int nargs, char ** args) 
{

  if (nargs > 1) gpio = atoi(args[1]); 

  
  

  sprintf(edge_path,"/sys/class/gpio/gpio%d/edge",gpio); 

  sprintf(val_path,"/sys/class/gpio/gpio%d/value", gpio); 

  if (access(edge_path,F_OK))
  {
    fprintf(stderr,"No %s, will try exporting\n", edge_path);
    FILE * fexport = fopen("/sys/class/gpio/export","w"); 
    fprintf(fexport,"%d\n", gpio);
    fclose(fexport); 
    sleep(1); 
    if (access(edge_path,F_OK))
    {
      fprintf(stderr,"exporting failed!!\n");
      return 1; 
    }
  }


  int edge_fd = open(edge_path,O_RDWR); 
  int val_fd = open(val_path,O_RDONLY); 

  write(edge_fd,"both",4); 

  struct pollfd fdset; 


  printf("Starting loop, ctrl-c to stop\n"); 
  signal(SIGINT, handler); 

  struct timespec when; 

  int nint = 0;
  while(!do_quit) 
  {
    memset(buf,0,sizeof(buf)); 
    memset(&fdset,0,sizeof(fdset)); 

    fdset.fd = val_fd; 
    fdset.events = POLLPRI; 

    int rc = poll(&fdset,1,-1); 
    clock_gettime(CLOCK_REALTIME,&when);
    if (fdset.revents & POLLPRI) 
    {
      lseek(fdset.fd,0,SEEK_SET); 
      int len = read(fdset.fd,buf,sizeof(buf)-1); 
      if (nint) 
        printf("gpio%d [%d] TIMER: %c at %d.%09d\n",gpio, nint, buf[0], when.tv_sec, when.tv_nsec); 
    }
    nint++; 
  }

  
  write(edge_fd,"none",4); 
}

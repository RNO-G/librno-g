
#include <time.h> 
#include <sys/types.h>
#include <fcntl.h>
#include <sys/stat.h> 
#include <unistd.h> 
#include <stdio.h> 
#include <signal.h> 
#include <string.h>
#include <poll.h> 

const int TIMER_GPIO = 23;



static char edge_path[512]; 
static char val_path[512]; 


static volatile int do_quit = 0;
void handler(int sig) 
{
  (void) sig; 
  do_quit=1; 
}


static char buf[128];
int main() 
{

  sprintf(edge_path,"/sys/class/gpio/gpio%d/edge",TIMER_GPIO); 

  sprintf(val_path,"/sys/class/gpio/gpio%d/value", TIMER_GPIO); 

  if (access(edge_path,F_OK))
  {
    fprintf(stderr,"No %s\n", edge_path);
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
        printf("[%d] TIMER: %c at %d.%09d\n", nint, buf[0], when.tv_sec, when.tv_nsec); 
    }
    nint++; 
  }

  
  write(edge_fd,"none",4); 
}

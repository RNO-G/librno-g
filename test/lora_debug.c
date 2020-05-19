
#include <time.h> 
#include <sys/types.h>
#include <fcntl.h>
#include <sys/stat.h> 
#include <unistd.h> 
#include <stdio.h> 
#include <signal.h> 
#include <string.h>
#include <poll.h> 

const int RX_GPIO = 20;
const int TX_GPIO = 51;



static char rx_edge_path[512]; 
static char rx_val_path[512]; 
static char tx_edge_path[512]; 
static char tx_val_path[512]; 


static volatile int do_quit = 0;
void handler(int sig) 
{
  do_quit=1; 
}

const char * gpio_names[] = {"RX","TX"}; 

static char buf[128];
int main(int nargs, char ** args) 
{

  sprintf(rx_edge_path,"/sys/class/gpio/gpio%d/edge", RX_GPIO); 
  sprintf(tx_edge_path,"/sys/class/gpio/gpio%d/edge", TX_GPIO); 

  sprintf(rx_val_path,"/sys/class/gpio/gpio%d/value", RX_GPIO); 
  sprintf(tx_val_path,"/sys/class/gpio/gpio%d/value", TX_GPIO); 

  if (access(rx_edge_path,F_OK))
  {
    fprintf(stderr,"No %s\n", rx_edge_path);
  }

  if (access(tx_edge_path,F_OK))
  {
    fprintf(stderr,"No %s\n", tx_edge_path);
  }


  int rx_edge_fd = open(rx_edge_path,O_RDWR); 
  int tx_edge_fd = open(tx_edge_path,O_RDWR); 

  int rx_val_fd = open(rx_val_path,O_RDONLY); 
  int tx_val_fd = open(tx_val_path,O_RDONLY); 



  write(tx_edge_fd,"both",4); 
  write(rx_edge_fd,"both",4); 

  struct pollfd fdset[2]; 
  int nfds =2;


  printf("Starting loop, ctrl-c to stop\n"); 
  signal(SIGINT, handler); 

  struct timespec when; 

  int nint = 0;
  while(!do_quit) 
  {
    memset(buf,0,sizeof(buf)); 
    memset(fdset,0,sizeof(fdset)); 

    fdset[0].fd = rx_val_fd; 
    fdset[1].fd = tx_val_fd; 
    fdset[0].events = POLLPRI; 
    fdset[1].events = POLLPRI; 

    int rc = poll(fdset,nfds,-1); 
    clock_gettime(CLOCK_REALTIME,&when);
    for (int i = 0; i < nfds; i++) 
    {

      if (fdset[i].revents & POLLPRI) 
      {
        lseek(fdset[i].fd,0,SEEK_SET); 
        int len = read(fdset[i].fd,buf,sizeof(buf)-1); 
        if (nint) 
          printf("[%d] %s: %c at %d.%09d\n", nint, gpio_names[i], buf[0], when.tv_sec, when.tv_nsec); 
      }
    }
    nint++; 

  }

  
  write(rx_edge_fd,"none",4); 
  write(tx_edge_fd,"none",4); 


}

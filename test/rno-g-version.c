#include "rno-g.h" 
#include <stdio.h>


int main(int nargs, char ** args) 
{
  printf("%s\n", rno_g_get_git_hash()); 
  return 0;
}

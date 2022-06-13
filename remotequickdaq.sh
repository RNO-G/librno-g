#! /bin/bash

if [ $# -lt 1 ] 
then 
 echo usage: ./remotequickdaq.sh host [ARGS] 
 exit 1
fi 

host=$1
shift
args="$@" 
if [ "${args}x" = "x" ] 
then 
  args="-f" 
fi 

ssh rno-g@${host} "LD_LIBRARY_PATH+=/home/rno-g/librno-g/build /home/rno-g/librno-g/build/radiant-try-event ${args}" 







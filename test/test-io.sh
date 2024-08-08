#! /bin/bash 
#
test_files=`echo test/data/s21_dh_pulsetest/*.dat.gz`


for f in $test_files ;
do
  echo "Testing $f against $f.expected"
  build/test/rno-g-dump $f | diff $f.expected - || exit 1
done








#! /bin/bash 
#
test_files=`echo test/data/*.dat.gz`

tar xf test/data/expected.tar.xz -C test/data "*.expected"

for f in $test_files ;
do
  echo "Testing $f against $f.expected"
  build/test/rno-g-dump $f | diff $f.expected - || exit 1
done








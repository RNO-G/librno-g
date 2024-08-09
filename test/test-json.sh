#! /bin/bash
#
test_files=`echo test/data/*.dat.gz`


for f in $test_files ;
do
  echo "Testing $f against $f.expected.json"
  build/test/rno-g-dump -j -n 10 $f | jq -c | diff $f.expected.json - || exit 1
done








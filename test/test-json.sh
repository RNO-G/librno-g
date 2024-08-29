#! /bin/bash
#
test_files=`echo test/data/*.dat.gz`

tar xf test/data/expected.tar.xz -C test/data "*.expected.json"

for f in $test_files ;
do
  echo "Testing $f against $f.expected.json"
  build/test/rno-g-dump -j $f | jq -c | diff $f.expected.json - || exit 1
done








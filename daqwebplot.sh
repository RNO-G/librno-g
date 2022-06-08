#! /bin/sh

if [ $# -lt 1 ] 
then 
 echo usage: ./daqwebplot.sh host id=`date -Is` dest=~/public_html/daqwebplot maxev=100
 exit 1
fi 




HOST=$1 
ID=${2-`date -Is`} 
DEST=${3-${HOME}/public_html/daqwebplot} 
MAXEV=${4-100} 


if [ "$(basename $(pwd))" != "librno-g" ] 
then 
  echo "Run this script from librno-g (i.e. as ./daqwebplot.sh" 
  echo "Or, even better, fix it so it can run from other places :" 
  exit 1
fi 


echo "Copying data from $HOST" 

OUT="${DEST}/${HOST}/${ID}" 
mkdir -p "${OUT}"
mkdir -p "${OUT}/wfs"
mkdir -p "${OUT}/peds"

rsync --compress -a -P $HOST:/data/test/\{wfs.dat,header.dat,daqstatus.dat,peds.dat\} "$OUT"

make -s build/test/rno-g-dump-hdr build/test/rno-g-dump-ped build/test/rno-g-dump-wf build/test/rno-g-dump-ds build/test/rno-g-wf-stats

echo "<html><head><title>$HOST -- $ID</title></head><body>" > ${OUT}/index.html
echo "<h1>$HOST -- $ID</h1><hr>" >> ${OUT}/index.html
echo "<li>header: <a href='header.dat'>.dat</a> <a href='headers.txt'>.txt</a>" >> ${OUT}/index.html 
echo "<li>wfs: <a href='wfstats.txt'>stats</a> <a href='wfs'>plots</a>  <a href='wfs.dat'>.dat</a> <a href='wfs.csv'>.csv</a> <a href='wfs.csv.gz'>compressed .csv</a>" >> ${OUT}/index.html 
echo "<li>daqstatus: <a href='daqstatus.dat'>.dat</a> <a href='ds.txt'>.txt</a>" >> ${OUT}/index.html 
echo "<li>peds: <a href='peds'>plots</a> <a href='peds.dat'>.dat</a> <a href='peds.csv'>.csv</a> <a href="peds.csv.gz">compressed .csv</a>" >> ${OUT}/index.html 
echo "</ul><hr>First event:<br>" >> ${OUT}/index.html 
echo "<img src='wfs/0.png'><br>" >> ${OUT}/index.html
echo "<hr>Spectra:<br><img src='spectra.png'><br><hr><ul>" >> ${OUT}/index.html
echo "</ul></body></html>" >> ${OUT}/index.html

LD_LIBRARY_PATH+=`pwd`/build build/test/rno-g-wf-stats "${OUT}/wfs.dat" > "${OUT}/wfstats.txt"
LD_LIBRARY_PATH+=`pwd`/build:/usr/local/lib root -b -q test/quick_spectra.C\(\"${OUT}/wfs.dat\",16777215,\"${OUT}/spectra.png\"\) 
LD_LIBRARY_PATH+=`pwd`/build:/usr/local/lib root -b -q test/quick_plot.C\(\"${OUT}/wfs.dat\",0,1,${MAXEV},\"${OUT}/wfs\"\) 
LD_LIBRARY_PATH+=`pwd`/build root -b -q test/quick_peds.C\(\"${OUT}/peds.dat\",\"${OUT}/peds\"\) 
LD_LIBRARY_PATH+=`pwd`/build build/test/rno-g-dump-hdr "${OUT}/header.dat" > "${OUT}/headers.txt"
LD_LIBRARY_PATH+=`pwd`/build build/test/rno-g-dump-ped "${OUT}/peds.dat" > "${OUT}/peds.csv"
LD_LIBRARY_PATH+=`pwd`/build build/test/rno-g-dump-wf "${OUT}/wfs.dat" > "${OUT}/wfs.csv"
LD_LIBRARY_PATH+=`pwd`/build build/test/rno-g-dump-ds "${OUT}/daqstatus.dat" > "${OUT}/ds.txt"
gzip -f -k ${OUT}/wfs.csv
gzip -f -k ${OUT}/peds.csv




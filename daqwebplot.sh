#! /bin/sh

if [ $# -lt 1 ] 
then 
 echo usage: ./daqwebplot.sh host id=`date -Is` label=\"\" dest=~/public_html/daqwebplot maxev=100 user=rno-g
 exit 1
fi 




HOST=$1 
ID=${2-`date -Is`} 
LABEL=${3-""} 
DEST=${4-${HOME}/public_html/daqwebplot} 
MAXEV=${5-100} 
USER=${6-rno-g} 


if [ "$(basename $(pwd))" != "librno-g" ] 
then 
  echo "Run this script from librno-g (i.e. as ./daqwebplot.sh" 
  echo "Or, even better, fix it so it can run from other places :" 
  exit 1
fi 


echo "Copying data from $HOST" 

OUT="${DEST}/${HOST}/${ID}" 

if [ -d "${OUT}" ] ; then 
  read -p "'$OUT' exists already, proceed? [y/N] " resp && [ "${resp}" = 'y' ] || exit 1
fi 

mkdir -p "${OUT}"
mkdir -p "${OUT}/wfs"
mkdir -p "${OUT}/peds"

rsync --compress -a -P ${USER}@${HOST}:/data/test/${LABEL}/wfs.dat "$OUT"
rsync --compress -a -P ${USER}@${HOST}:/data/test/${LABEL}/header.dat "$OUT"
rsync --compress -a -P ${USER}@${HOST}:/data/test/${LABEL}/daqstatus.dat "$OUT"
rsync --compress -a -P ${USER}@${HOST}:/data/test/${LABEL}/peds.dat "$OUT"

make -s build/test/rno-g-dump-hdr build/test/rno-g-dump-ped build/test/rno-g-dump-wf build/test/rno-g-dump-ds build/test/rno-g-wf-stats

cat <<EOF > ${OUT}/index.html 
<html><head><title>$HOST -- $ID</title>
<script type='text/javascript'>
function get() 
{
  return parseInt(document.getElementById("whichwf").value);
}
function go(i) 
{
  document.getElementById("wfimg").src="wfs/" + i + ".png"; 
  document.getElementById("whichwf").value = i; 
  document.getElementById("whichwf").alt = "event"  + i; 
}
</script>
</head><body>
<h1>$HOST -- $ID</h1><hr>
<li>header: <a href='header.dat'>.dat</a> <a href='headers.txt'>.txt</a>
<li>wfs: <a href='wfstats.txt'>stats</a> <a href='wfs'>plots</a>  <a href='wfs.dat'>.dat</a> <a href='wfs.csv'>.csv</a> <a href='wfs.csv.gz'>compressed .csv</a>
<li>daqstatus: <a href='daqstatus.dat'>.dat</a> <a href='ds.txt'>.txt</a>
<li>peds: <a href='peds'>plots</a> <a href='peds.dat'>.dat</a> <a href='peds.csv'>.csv</a> <a href="peds.csv.gz">compressed .csv</a>
</ul><hr>Event <input id='whichwf' size=4 value='0' onchange='document.getElementById("wfimg").src="wfs/" + document.getElementById("whichwf").value+".png"'> 
<input type='button' value='<--' onclick='go(get()-1)'>
<input type='button' value='-->' onclick='go(get()+1)'>: <br>
<img src='wfs/0.png' id='wfimg'><br>
<hr>Spectra:<br><img src='spectra.png'><br><hr><ul>
</ul></body></html>
EOF

LD_LIBRARY_PATH+=:`pwd`/build build/test/rno-g-wf-stats "${OUT}/wfs.dat" > "${OUT}/wfstats.txt"
LD_LIBRARY_PATH+=:`pwd`/build:/usr/local/lib root -b -q test/quick_spectra.C\(\"${OUT}/wfs.dat\",16777215,\"${OUT}/spectra.png\"\) 
LD_LIBRARY_PATH+=:`pwd`/build:/usr/local/lib root -b -q test/quick_plot.C\(\"${OUT}/wfs.dat\",0,1,${MAXEV},\"${OUT}/wfs\"\) 
LD_LIBRARY_PATH+=:`pwd`/build root -b -q test/quick_peds.C\(\"${OUT}/peds.dat\",\"${OUT}/peds\"\) 
LD_LIBRARY_PATH+=:`pwd`/build build/test/rno-g-dump-hdr "${OUT}/header.dat" > "${OUT}/headers.txt"
LD_LIBRARY_PATH+=:`pwd`/build build/test/rno-g-dump-ped "${OUT}/peds.dat" > "${OUT}/peds.csv"
LD_LIBRARY_PATH+=:`pwd`/build build/test/rno-g-dump-wf "${OUT}/wfs.dat" > "${OUT}/wfs.csv"
LD_LIBRARY_PATH+=:`pwd`/build build/test/rno-g-dump-ds "${OUT}/daqstatus.dat" > "${OUT}/ds.txt"
gzip -f -k ${OUT}/wfs.csv
gzip -f -k ${OUT}/peds.csv




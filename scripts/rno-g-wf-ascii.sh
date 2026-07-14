#!/usr/bin/env bash
# ASCII-plot RADIANT waveforms from .wf.dat.gz files, one channel per row.
# Uses rno-g-dump-wf (build with 'make rno-g-utils') to decode the file.

set -euo pipefail

usage() {
  cat >&2 <<EOF
usage: $(basename "$0") [-e EVENT] [-c CHANNELS] [-w WIDTH] [-H HEIGHT] [-m MARKER] [-s START] [-n NSAMPLES] FILE [FILE...]

  -e EVENT     event index within the file (default: 0)
  -c CHANNELS  channels to plot, comma list with ranges, e.g. 0,1,4-7 (default: 0-23)
  -w WIDTH     plot width in columns (default: terminal width)
  -H HEIGHT    rows per channel plot (default: 20)
  -m MARKER    plot marker character (default: '.')
  -s START     first sample to plot (default: 0)
  -n NSAMPLES  number of samples to plot from START (default: all)

If WIDTH equals NSAMPLES (one column per sample), the plot draws slopes
with '/' and '\' instead of MARKER, connected by '|' across empty rows.

Example:

  $(basename "$0") -c 0 -n w FILE

Draws a single waveform (channel 0) over the full terminal window. Reduce -w and -H if the waveform extends the window size.

Environment: DUMP_WF overrides the path to the rno-g-dump-wf binary.
EOF
  exit 1
}

EVENT=0
CHANNELS=""
WIDTH=""
HEIGHT=20
MARKER="."
START=0
NSAMPLES=0

while getopts "e:c:w:H:m:s:n:h" opt; do
  case $opt in
    e) EVENT=$OPTARG ;;
    c) CHANNELS=$OPTARG ;;
    w) WIDTH=$OPTARG ;;
    H) HEIGHT=$OPTARG ;;
    m) MARKER=${OPTARG:0:1} ;;
    s) START=$OPTARG ;;
    n) NSAMPLES=$OPTARG ;;
    *) usage ;;
  esac
done
shift $((OPTIND-1))
[ $# -ge 1 ] || usage

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)/.."


DUMP_WF=${DUMP_WF:-}
if [ -z "$DUMP_WF" ]; then
  if [ -x "$SCRIPT_DIR/build/test/rno-g-dump-wf" ]; then
    DUMP_WF="$SCRIPT_DIR/build/test/rno-g-dump-wf"
  elif command -v rno-g-dump-wf >/dev/null 2>&1; then
    DUMP_WF=$(command -v rno-g-dump-wf)
  else
    echo "rno-g-dump-wf not found: run 'make rno-g-utils' or set DUMP_WF" >&2
    exit 1
  fi
fi
echo $DUMP_WF

export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$SCRIPT_DIR/build"
export DYLD_LIBRARY_PATH="${DYLD_LIBRARY_PATH:-}:$SCRIPT_DIR/build"
echo $LD_LIBRARY_PATH
echo $DYLD_LIBRARY_PATH

# expand channel spec (e.g. "0,2,4-7") into a comma list
if [ -z "$CHANNELS" ]; then
  CHLIST=$(printf "%s," $(seq 0 23))
else
  CHLIST=""
  IFS=, read -ra parts <<< "$CHANNELS"
  for p in "${parts[@]}"; do
    if [[ $p == *-* ]]; then
      CHLIST+=$(printf "%s," $(seq "${p%-*}" "${p#*-}"))
    else
      CHLIST+="$p,"
    fi
  done
fi
CHLIST=${CHLIST%,}

if [ -z "$WIDTH" ]; then
  WIDTH=$(( $(tput cols 2>/dev/null || echo 120) - 10 ))
fi

if [ "${NSAMPLES}" = "w" ]; then
  NSAMPLES=$WIDTH
fi

for file in "$@"; do
  [ $# -gt 1 ] && echo "=== $file ==="
  # awk stops reading once the requested event is done; tolerate the dump's SIGPIPE (141)
  { "$DUMP_WF" "$file" || [ $? -eq 141 ]; } |
  awk -v event="$EVENT" -v chlist="$CHLIST" -v W="$WIDTH" -v H="$HEIGHT" \
      -v marker="$MARKER" -v start="$START" -v nsamp="$NSAMPLES" -v nch=24 '
  function torow(v) { return H - 1 - int((v - vmin) / span * (H - 1) + 0.5) }
  BEGIN {
    nsel = split(chlist, cl, ",");
    for (i = 1; i <= nsel; i++) sel[cl[i] + 0] = 1;
  }
  {
    ev = int((NR - 1) / nch);
    if (ev > event) exit;
    ch = (NR - 1) % nch;
    if (ev != event || !(ch in sel)) next;
    found = 1;
    n = split($0, s, ",");
    # window into [j0, j0+nwin-1], 1-based; start is a 0-based sample offset
    j0 = start + 1; if (j0 < 1) j0 = 1;
    if (j0 > n) {
      printf("ch %2d: start %d is past the last sample (%d)\n", ch, start, n - 1) > "/dev/stderr";
      next;
    }
    nwin = (nsamp + 0 > 0) ? nsamp + 0 : n - j0 + 1;
    if (j0 + nwin - 1 > n) nwin = n - j0 + 1;
    plotted = 1; axis_start = j0 - 1; axis_n = nwin;
    vmin = s[j0] + 0; vmax = vmin; sum = 0; sum2 = 0;
    slope_mode = (nwin == W && nwin >= 2);
    for (c = 0; c < W; c++) { lo[c] = 1e18; hi[c] = -1e18 }
    for (j = j0; j <= j0 + nwin - 1; j++) {
      v = s[j] + 0;
      c = int((j - j0) * W / nwin);
      if (slope_mode) valc[c] = v;
      if (v < lo[c]) lo[c] = v;
      if (v > hi[c]) hi[c] = v;
      if (v < vmin) vmin = v;
      if (v > vmax) vmax = v;
      sum += v; sum2 += v * v;
    }
    mean = sum / nwin;
    rms = sqrt(sum2 / nwin - mean * mean);
    span = vmax - vmin; if (span == 0) span = 1;
    zrow = (vmin <= 0 && vmax >= 0) ? torow(0) : -1;
    if (slope_mode) {
      for (c = 0; c < W; c++) {
        if (c == 0) {
          dirchar[c] = (valc[1] > valc[0]) ? "/" : (valc[1] < valc[0]) ? "\\" : marker;
          arrowrow[c] = torow(valc[0]);
          pipelo[c] = 1; pipehi[c] = 0;   # nothing to connect before the first sample
        } else {
          rp = torow(valc[c - 1]); rc = torow(valc[c]);
          arrowrow[c] = rc;
          dirchar[c] = (valc[c] > valc[c - 1]) ? "/" : (valc[c] < valc[c - 1]) ? "\\" : "-";
          if (rc < rp)      { pipelo[c] = rc + 1; pipehi[c] = rp - 1; }
          else if (rc > rp) { pipelo[c] = rp + 1; pipehi[c] = rc - 1; }
          else              { pipelo[c] = 1; pipehi[c] = 0; }
        }
      }
    }
    printf("ch %2d  min=%d max=%d mean=%.1f rms=%.1f\n", ch, vmin, vmax, mean, rms);
    for (r = 0; r < H; r++) {
      if (r == 0)          printf("%7d |", vmax);
      else if (r == H - 1) printf("%7d |", vmin);
      else if (r == zrow)  printf("%7d |", 0);
      else                 printf("        |");
      for (c = 0; c < W; c++) {
        if (slope_mode) {
          if (r == arrowrow[c]) printf("%s", dirchar[c]);
          else if (r >= pipelo[c] && r <= pipehi[c]) printf("|");
          else printf(" ");
        } else {
          if (lo[c] <= hi[c] && r >= torow(hi[c]) && r <= torow(lo[c])) printf("%s", marker);
          else printf(" ");
        }
      }
      printf("\n");
    }
    printf("\n");
  }
  END {
    if (!found) {
      printf("event %d not found (file has %d events)\n", event, int(NR / nch)) > "/dev/stderr";
      exit 1;
    }
    if (!plotted) exit 1;   # event found but every selected channel window was empty
    # shared x axis: sample index ticks every quarter
    printf("        +"); for (c = 0; c < W; c++) printf("-"); printf("\n         ");
    for (t = 0; t <= 4; t++) {
      lbl = sprintf("%d", axis_start + int(t * axis_n / 4));
      pos = int(t * (W - 1) / 4);
      printf("%*s", (t == 0 ? 1 : pos - prevpos), lbl);
      prevpos = pos + length(lbl) - 1;
    }
    printf(" [sample]");
    printf("\n[%.1f samples/col]\n", axis_n / W);
  }'
done

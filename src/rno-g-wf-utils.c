#include "rno-g-wf-utils.h"
#include "radiant.h"
#include <stdlib.h>
#include <math.h>
#include <assert.h>


rno_g_nsample_diff_hist_t * rno_g_nsample_diff_hist_create(struct rno_g_nsample_diff_hist_setup setup)
{
  if (!setup.nbins) setup.nbins = setup.use_abs ? 512: 1025;
  rno_g_nsample_diff_hist_t * h = malloc(rno_g_sample_diff_hist_size(setup.nbins));
  if (!h) return NULL;
  memset(h,0,rno_g_sample_diff_hist_size(setup.nbins));
  h->setup = setup;
  return h;
}

int rno_g_nsample_diff_hist_fill(rno_g_nsample_diff_hist_t * hist, const rno_g_waveform_t * wf)
{
 //check for matching station
  if (wf->station != hist->setup.station) return -1;
  if (hist->setup.channel > RNO_G_NUM_RADIANT_CHANNELS) return -1;
  int filled = 0;
  for (unsigned i = hist->setup.delta_nsamples; i < wf->radiant_nsamples; i+=hist->setup.delta_nsamples)
  {
    int16_t diff = wf->radiant_waveforms[hist->setup.channel][i] - wf->radiant_waveforms[hist->setup.channel][i-1];
    if (hist->setup.use_abs && diff < 0) diff = -diff;

    hist->entries_sum += diff;
    hist->entries_sum2 += diff*diff;
    hist->nfilled++;
    filled++;
    //figure out the right bin
    if (hist->setup.use_abs)
    {
      int bin = diff / hist->setup.binsize;
      if (bin > hist->setup.nbins) bin = hist->setup.nbins;
      hist->data[1+bin]++;
    }
    else
    {
      int bin = diff / hist->setup.binsize + hist->setup.nbins/2;
      if (bin  < 0) bin = -1;
      if (bin > hist->setup.nbins) bin = hist->setup.nbins;
      hist->data[1+bin]++;
    }
  }

  return filled;
}


int rno_g_nsample_diff_hist_write_json(FILE *f, const rno_g_nsample_diff_hist_t * h, int indent_level)
{
  int written = 0;
  written+=fprintf(f,"%*s{\n", indent_level," ");
  written+=fprintf(f,"%*s\"station\" : %hhu,\n", indent_level+2," ", h->setup.station);
  written+=fprintf(f,"%*s\"channel\" : %hhu,\n", indent_level+2," ", h->setup.channel);
  written+=fprintf(f,"%*s\"delta_nsamples\" : %hu,\n", indent_level+2," ", h->setup.delta_nsamples);
  written+=fprintf(f,"%*s\"nbins\" : %hu,\n", indent_level+2," ", h->setup.nbins);
  written+=fprintf(f,"%*s\"binsize\" : %hu,\n", indent_level+2," ", h->setup.binsize);
  written+=fprintf(f,"%*s\"use_abs\" : %s,\n", indent_level+2," ", h->setup.use_abs ? "true" : "false");
  written+=fprintf(f,"%*s\"xmin\" : %d,\n", indent_level+2," ", h->setup.use_abs ? 0 : -h->setup.nbins/2*h->setup.binsize);
  written+=fprintf(f,"%*s\"xmax\" : %d,\n", indent_level+2," ", h->setup.use_abs ? h->setup.nbins*h->setup.binsize : -h->setup.nbins/2*h->setup.binsize + h->setup.nbins*h->setup.binsize);
  written+=fprintf(f,"%*s\"nfilled\" : %u,\n", indent_level+2," ", h->nfilled );
  written+=fprintf(f,"%*s\"entries_sum\" : %f,\n", indent_level+2," ", h->entries_sum );
  written+=fprintf(f,"%*s\"entries_sum2\" : %f,\n", indent_level+2," ", h->entries_sum2 );
  float mean = h->entries_sum / h->nfilled;
  written+=fprintf(f,"%*s\"mean\" : %f,\n", indent_level+2," ", mean);
  written+=fprintf(f,"%*s\"rms\" : %f,\n", indent_level+2," ", sqrt(h->entries_sum2/h->nfilled - mean*mean));
  written+=fprintf(f,"%*s\"data\" : [ ", indent_level+2, " ");
  for (unsigned u = 0u; u < h->setup.nbins+2u; u++)
  {
    if (u > 0) written+=fprintf(f,",");
    written+=fprintf(f, "%u", h->data[u]);
  }
  written+=fprintf(f,"]\n");
  written+=fprintf(f,"%*s}\n", indent_level, " ");
  return written;
}

int rno_g_nsample_diff_hist_write_jsroot_webpage(FILE * f, int nhists, const rno_g_nsample_diff_hist_t **h, int individual_hists)
{
  if (!f) return -1;

  int nb = 0;
  nb += fprintf(f,"<html>\n");
  nb += fprintf(f,"<head>\n");
  nb += fprintf(f,"  <title>Nsample Diff Histograms</title>\n");
  nb += fprintf(f,"  <style type='text/css'> .plot { width: 600; height: 400;  float: left; } </style>\n");
  nb += fprintf(f,"  <script type=\"module\">\n");
  nb += fprintf(f,"     import { createHistogram, draw } from 'https://root.cern/js/7.0.0/modules/main.mjs';");
  nb += fprintf(f,"     var hists = [];\n");
  nb += fprintf(f,"     var chans = [];\n");
  nb += fprintf(f,"     var nplots = Array(%d).fill(%d);\n", individual_hists ? nhists : RNO_G_NUM_RADIANT_CHANNELS, individual_hists ? 1 : 0);
  if (individual_hists)
    nb += fprintf(f,"     function drawHists() { for (var i = 0; i < hists.length; i++) { draw('c'+(i+1), hists[i],""); }}\n");
  else
    nb += fprintf(f,"     function drawHists() { for (var i = 0; i < hists.length; i++) { draw('ch_'+chans[i], hists[i],"");}}\n");
  nb += fprintf(f,"     function makeHist(o) {  var h = createHistogram('TH1F',o.nbins); "
                  "       h.fName='s' + o.station+'ch'+o.channel+'delta'+o.delta_nsamples;\n"
                  "       h.fTitle='Station ' + o.station+', Channel, '+o.channel%s;\n"
                  "       h.fXaxis.fTitle = o.use_abs ?'|Sample Difference|' : 'Sample Difference';\n "
                  "       h.fYaxis.fTitle = 'Frequency';\n"
                  "       h.fXaxis.fXmin = o.xmin;\n"
                  "       h.fXaxis.fXmax = o.xmax;\n"
                  "       h.fEntries = o.nfilled;\n"
                  "       h.fTsumw = o.nfilled;\n"
                  "       h.fLineColor = nplots[o.channel];\n"
                  "       h.fTsumw2 = o.nfilled;\n"
                  "       h.fTsumwx = o.entries_sum;\n"
                  "       h.fTsumwx2 = o.entries_sum2;\n"
    , individual_hists ? "+ '  #Delta N =' + o.delta_nsamples":"");

  if (!individual_hists)
     nb += fprintf(f,"       for (var i = 0; i < o.nbins+2; i++) { h.fArray[i] = o.data[i]/o.nfilled;}\n");
  nb += fprintf(f, "       return h; }\n");

  for (int i = 0; i < nhists; i++)
  {
    nb += fprintf (f,"     chans.push(%u);\n", h[i]->setup.channel);
    if (!individual_hists)
      nb += fprintf (f,"    nplots[%d]++;\n", h[i]->setup.channel);
    nb += fprintf (f,"     hists.push(makeHist(\n");
    nb += rno_g_nsample_diff_hist_write_json(f,h[i], 5);
    nb += fprintf (f,"     ));\n");
 }

  nb += fprintf(f,"  drawHists(); console.log(hists); \n");
  nb += fprintf(f,"  </script>\n");
  nb += fprintf(f,"</head>\n");
  nb += fprintf(f,"<body>\n");

  if (individual_hists)
  {
    for (int i = 0; i < nhists; i++)
    {
      nb += fprintf(f," <div class='plot' id='c%d'></div>\n", i+1);
    }

  }
  else
  {
    for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++)
    {
      nb += fprintf(f," <div class='plot' id='ch_%d'></div>\n", i+1);
    }
  }
  nb += fprintf(f,"</body></html>\n");
  fclose(f);
  return nb;
}


void rno_g_zerocross_stats_init(rno_g_zerocross_stats_t *zc, uint8_t st, uint8_t ch)
{
  memset(zc, 0, sizeof(*zc));
  zc->station = st;
  zc->channel = ch;
}


void rno_g_zerocross_stats_process(rno_g_zerocross_stats_t * zc, const rno_g_header_t * hd, const rno_g_waveform_t *wf)
{
  // check that station is correct?
  if (wf->station != zc->station || hd->station_number != zc->station)
    fprintf(stderr,"Warning: station mismatch. zc: %hhd, hd: %hhd, wf: %hhd\n", zc->station, hd->station_number,  wf->station);

  if (wf->event_number != hd->event_number || wf->run_number != hd->run_number)
    fprintf(stderr,"Warning: hd/wf mismatch. hd: r%u/e%u,  wf: r%u/e%u\n", hd->run_number, hd->event_number, wf->run_number, wf->event_number);

  //assume we have the newer radiant firmware...
  unsigned start_sample = hd->radiant_start_windows[zc->channel][0] * RADIANT_WINDOW_SIZE;
  unsigned offset = start_sample > 2048 ? 2048 : 0;
  start_sample %=2048;

  zc->nwf++;
  double last_zero = -1;
  int ch = zc->channel;
  for (int i = 0; i < 2047; i++)
  {
    //a zero crossing
    if ( (wf->radiant_waveforms[ch][i+1] <= 0) != (wf->radiant_waveforms[ch][i] <= 0))
    {
      //associate it with this ssample
      zc->crossing_counter[ offset + (start_sample + i ) % 2048]++;
      zc->nzero++;

      //linearly inteprolate to zero position
      double m = wf->radiant_waveforms[ch][i+1] - wf->radiant_waveforms[ch][i];
      double b = wf->radiant_waveforms[ch][i];
      // 0 = m *x + b -> x = -b/m;
      double zero =  -b/m + i;

      if (last_zero >= 0)
      {
        double dzero = zero-last_zero;
        zc->period_sum += dzero;
        zc->period_sum2 += dzero*dzero;
      }

      last_zero = zero;
    }
  }
}

int rno_g_zerocross_stats_period_mean_rms(const rno_g_zerocross_stats_t *zc, double * mean, double *rms)
{
  if (zc->nzero - zc->nwf <= 0)
    return 1;

  double mean_period = zc->period_sum / (zc->nzero-zc->nwf);
  if (mean) *mean = mean_period;
  if (rms)
  {
    *rms = sqrt(zc->period_sum2 / (zc->nzero-zc->nwf)- mean_period*mean_period);
  }
  return 0;
}


int rno_g_zerocross_stats_dump(FILE *f, const rno_g_zerocross_stats_t *zc, int indent_level)
{

  int written = 0;
  written+=fprintf(f,"%*s{\n", indent_level," ");
  written+=fprintf(f,"%*s\"station\" : %hhu,\n", indent_level+2," ", zc->station);
  written+=fprintf(f,"%*s\"channel\" : %hhu,\n", indent_level+2," ", zc->channel);
  written+=fprintf(f,"%*s\"nwf\" : %u,\n", indent_level+2," ", zc->nwf);
  written+=fprintf(f,"%*s\"nzero\" : %u,\n", indent_level+2," ", zc->nzero);
  written+=fprintf(f,"%*s\"period_sum\" : %f,\n", indent_level+2," ", zc->period_sum);
  written+=fprintf(f,"%*s\"period_sum2\" : %f,\n", indent_level+2," ", zc->period_sum2);

  double mean_period = 0;
  double period_rms = 0;
  rno_g_zerocross_stats_period_mean_rms(zc,&mean_period,&period_rms);
  written+=fprintf(f,"%*s\"mean_period\" : %f,\n", indent_level+2," ", mean_period);
  written+=fprintf(f,"%*s\"stdev_period\" : %f,\n", indent_level+2," ", period_rms);

  written+=fprintf(f,"%*s\"crossing_counter\" : [", indent_level+2," ");

  for (int i = 0; i < 4096; i++)
    written += fprintf(f,"%s%hu", i==0?"": ",", zc->crossing_counter[i]);

  written += fputs("]\n",f);
  written+=fprintf(f,"%*s}\n", indent_level," ");

  return written;

}



void rno_g_unscramble(unsigned N, int16_t *wf)
{
  //  even goes 128 forward
  //  odd goes 128 back
  unsigned nwindows = N / 128;
  assert ( nwindows * 128 == N);
  for (unsigned iwindow = 0; iwindow < nwindows-1; iwindow++)
  {
    //copy odd (starting from front)
    memcpy(&wf[iwindow*128 + 64], &wf[(iwindow+1)*128 + 64], 64 *  sizeof(*wf));
    //copy even (starting from back)
    memcpy(&wf [(nwindows - iwindow -1) * 128],  &wf[ (nwindows -iwindow -2) * 128], 64 * sizeof(*wf));
  }

  //fill front and back with 0
  memset(wf, 0, 64* sizeof(*wf));
  memset(&wf[(nwindows-1)*128+64], 0, 64*sizeof(*wf));
}

int rno_g_scramble_score(unsigned N, const int16_t * wf)
{

  int scrambled_sum_diff2 = 0;
  int unscrambled_sum_diff2 = 0;

  unsigned nwindows = N / 128;
  assert( nwindows * 128 == N);

  for (unsigned iwindow = 0; iwindow < nwindows; iwindow++)
  {
    //first half of of window unscrambled
    unscrambled_sum_diff2 +=  pow(wf[iwindow*128]  - wf[iwindow*128-1],2);

    //second half of window unscrambled, except for last window (since that's inadmissable for scrambled
    unscrambled_sum_diff2 +=  pow(wf[iwindow*128+64]  - wf[iwindow*128+64-1],2);

    //first half unscrambled
    if  (iwindow > 0)
      scrambled_sum_diff2 +=  pow(wf[(iwindow-1)*128]  - wf[(iwindow+1)*128-1],2);

    //odd half unscrambled
    if (iwindow < nwindows -1)
      scrambled_sum_diff2 +=  pow(wf[(iwindow+1)*128+64]  - wf[(iwindow-1)*128+64-1],2);

  }

  //adjust for different number of windows (??)
  scrambled_sum_diff2 *= nwindows / (nwindows-1);


  return unscrambled_sum_diff2 - scrambled_sum_diff2;
}


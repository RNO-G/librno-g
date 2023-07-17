#include "rno-g-nsample-diff-hist.h" 
#include <stdlib.h>
#include <math.h>


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
  written+=fprintf(f,"%*sstation : %hhu,\n", indent_level+2," ", h->setup.station); 
  written+=fprintf(f,"%*schannel : %hhu,\n", indent_level+2," ", h->setup.channel); 
  written+=fprintf(f,"%*sdelta_nsamples : %hu,\n", indent_level+2," ", h->setup.delta_nsamples); 
  written+=fprintf(f,"%*snbins : %hu,\n", indent_level+2," ", h->setup.nbins); 
  written+=fprintf(f,"%*sbinsize : %hu,\n", indent_level+2," ", h->setup.binsize); 
  written+=fprintf(f,"%*suse_abs : %s,\n", indent_level+2," ", h->setup.use_abs ? "true" : "false"); 
  written+=fprintf(f,"%*sxmin : %d,\n", indent_level+2," ", h->setup.use_abs ? 0 : -h->setup.nbins/2*h->setup.binsize);
  written+=fprintf(f,"%*sxmax : %d,\n", indent_level+2," ", h->setup.use_abs ? h->setup.nbins*h->setup.binsize : -h->setup.nbins/2*h->setup.binsize + h->setup.nbins*h->setup.binsize);
  written+=fprintf(f,"%*snfilled : %u,\n", indent_level+2," ", h->nfilled );
  written+=fprintf(f,"%*sentries_sum : %f,\n", indent_level+2," ", h->entries_sum );
  written+=fprintf(f,"%*sentries_sum2 : %f,\n", indent_level+2," ", h->entries_sum2 );
  float mean = h->entries_sum / h->nfilled; 
  written+=fprintf(f,"%*smean : %f,\n", indent_level+2," ", mean); 
  written+=fprintf(f,"%*srms : %f,\n", indent_level+2," ", sqrt(h->entries_sum2/h->nfilled - mean*mean));
  written+=fprintf(f,"%*sdata : [ ", indent_level+2, " "); 
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

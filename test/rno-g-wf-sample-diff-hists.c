#include "rno-g-nsample-diff-hist.h" 
#include <stdio.h> 
#include <math.h> 
#include <stdlib.h> 


static FILE * fout = 0; 
static uint32_t chmask = 0xffffff; 
static const char * sample_diffs = "1,64,128"; 
int nbins = 257; 
int binsize = 1; 
int use_abs = 0; 
int individual = 0; 




static void usage() 
{
  fprintf(stderr,"rno-g-wf-sample-diff-hists [-o,--output OFILE=stdout] [-c,--channels MASK=0x%x] [-d,--diffs DIFFS=%s] [-n,--nbins NBINS=1025] [-b,--binsize  BINWIDTH=1] [-a,--abs USE_ABSOLUTE_VALUE] [-i,--individual INDIVIDUAL_PLOTS] infile [infile2 ...]\n", chmask, sample_diffs); 
}


static rno_g_nsample_diff_hist_t ** hists; 
static rno_g_nsample_diff_hist_t ** hists_by_chan[RNO_G_NUM_RADIANT_CHANNELS]; 
static int nhists = 0; 
static int nhists_alloc = 0; 
static int nhists_by_chan[RNO_G_NUM_RADIANT_CHANNELS]; 
static int nhists_by_chan_alloc[RNO_G_NUM_RADIANT_CHANNELS];

void add_hist(rno_g_nsample_diff_hist_t * h) 
{

  if (!nhists_alloc) 
  {
    nhists_alloc = 72; 
    hists = malloc(nhists_alloc * sizeof(h)); 
  }

  if (nhists >= nhists_alloc) 
  {
    nhists_alloc *=2;
    hists = realloc(hists, nhists_alloc*sizeof(h)); 
  }

  hists[nhists++]= h;

  int chan = h->setup.channel; 

  if (!nhists_by_chan_alloc[chan])
  {
    nhists_by_chan_alloc[chan] = 3; 
    hists_by_chan[chan] = malloc(nhists_by_chan_alloc[chan] * sizeof(h)); 
  }

  if (nhists_by_chan[chan] > nhists_by_chan_alloc[chan])
  {
    nhists_by_chan_alloc[chan] *=2; 
    hists_by_chan[chan] = realloc(hists_by_chan[chan],nhists_by_chan_alloc[chan] * sizeof(h)); 
  }

  hists_by_chan[chan][nhists_by_chan[chan]++] = h; 
}

static int  nfiles = 0; 
static int  nfiles_alloc = 0; 
static const char ** files; 

void add_file(const char * f) 
{
  if (!nfiles_alloc) 
  {
    nfiles_alloc = 10; 
    files = malloc (nfiles_alloc*sizeof(f)); 
  }

  if (nfiles >= nfiles_alloc) 
  {
    nfiles_alloc *=1.5; 
    files = realloc(files, nfiles_alloc*sizeof(f)); 
  }

  files[nfiles++] = f; 
}


int main(int nargs, char ** args) 
{

  for (int i = 1; i < nargs; i++) 
  {
    if (!strcmp(args[i],"-o") || !strcmp(args[i],"--output"))
    {
      if (i == nargs-1)
      {
        usage(); 
        return 1; 
      }

      fout = fopen( args[i+1], "w"); 
      if (!fout) 
      {
        fprintf(stderr,"Could not open %s\n", args[i+1]); 
        return 1; 
      }
      i++; 
    }

    else if (!strcmp(args[i],"-b") || !strcmp(args[i],"--binsize"))
    {
      if (i == nargs-1)
      {
        usage(); 
        return 1; 
      }
      binsize = atoi(args[i+1]); 
      if (binsize < 1) binsize = 1; 
      i++; 
    }

    else if (!strcmp(args[i],"-n") || !strcmp(args[i],"--nbins"))
    {
      if (i == nargs-1)
      {
        usage(); 
        return 1; 
      }
      nbins = atoi(args[i+1]); 
      if (nbins < 1) nbins = 1025; 
      i++; 
    }

    else if (!strcmp(args[i],"-a") || !strcmp(args[i],"--abs"))
    {
      use_abs =1; 
    }
    else if (!strcmp(args[i],"-i") || !strcmp(args[i],"--individual"))
    {
      individual =1; 
    }
 
    else if (!strcmp(args[i],"-c") || !strcmp(args[i],"--channels"))
    {
      if (i == nargs-1)
      {
        usage(); 
        return 1; 
      }

      char * endptr; 
      chmask =  strtol(args[i+1], &endptr,0); 
      if (endptr) 
      {
        fprintf(stderr,"Bad mask: %s\n", args[i+1]); 
        return 1; 
      }
      i++;
    }

    else if (!strcmp(args[i],"-d") || !strcmp(args[i],"--diffs"))
    {
      if (i == nargs-1)
      {
        usage(); 
        return 1; 
      }

      sample_diffs = args[i+1]; 
      i++; 
    }
    else
    {
      add_file(args[i]); 
    }
  }

  
  //load the first file 
  if (! nfiles) 
  {
    usage(); 
    return 0; 
  }
  rno_g_waveform_t wf; 
  rno_g_file_handle_t h; 
  int ifile = 0; 
  rno_g_init_handle(&h, files[0], "r"); 
  if (!rno_g_handle_is_open(h)) 
  {
    fprintf(stderr,"Could not open first file %s\n", files[0]); 
    return 1; 
  }

  if (rno_g_waveform_read(h, &wf) <= 0 )
  {
    fprintf(stderr,"Problem reading wf from %s?\n", files[0]); 
  }
  

  // ok, let's parse the diffs 

  char * copy_diffs = strdup(sample_diffs); 

  char * token ; 
  token = strtok(copy_diffs,","); 

  while (token) 
  {
    int diff = atoi(token); 
    if (diff > 0 && diff < RNO_G_MAX_RADIANT_NSAMPLES )
    {
      for (int ichan = 0; ichan < RNO_G_NUM_RADIANT_CHANNELS; ichan++) 
      {
        if ( chmask & ( 1 << ichan))
        {
          rno_g_nsample_diff_hist_t * h = rno_g_nsample_diff_hist_create( 
              (struct rno_g_nsample_diff_hist_setup) 
              {.station = wf.station, 
              .channel = ichan, 
              .delta_nsamples = diff, 
              .nbins = nbins, 
              .binsize = binsize, 
              .use_abs = use_abs});

          add_hist(h); 
        }
      }
    }
    token = strtok(0,","); 
  }

  free(copy_diffs); 

  while (ifile  < nfiles) 
  {
    do 
    {
      for (int i = 0; i < nhists; i++) 
      {
        rno_g_nsample_diff_hist_fill(hists[i], &wf); 
      }
    } while (rno_g_waveform_read(h, &wf) > 0); 
    rno_g_close_handle(&h); 
    ifile++; 
    if (ifile < nfiles -1) 
    {
      rno_g_init_handle(&h, files[ifile], "r"); 
      while (rno_g_handle_is_open(h))
      {
        fprintf(stderr,"Could not open %s, skippping\n",files[ifile]);
        rno_g_init_handle(&h, files[++ifile], "r"); 
      }
    }
  }

  if (fout) rno_g_nsample_diff_hist_write_jsroot_webpage(fout, nhists, (const rno_g_nsample_diff_hist_t **) hists, individual); 

  for (int ichan = 0; ichan < RNO_G_NUM_RADIANT_CHANNELS; ichan++) 
  {
    if (!(chmask & (1 << ichan))) continue; 

    printf("CHANNEL %d%s:\n", ichan, use_abs ? " (abs)" : "" ); 
    for (int ihist = 0; ihist < nhists_by_chan[ichan]; ihist++) 
    {
      const rno_g_nsample_diff_hist_t * h = hists_by_chan[ichan][ihist]; 
      float mean = h->entries_sum/h->nfilled; 
      printf("  delta_nsamples=%d, entries=%d,  mean=%f, rms=%f\n", h->setup.delta_nsamples, h->nfilled, mean, sqrt(h->entries_sum2/h->nfilled -mean*mean));  
    }

  }

  return 0; 
}






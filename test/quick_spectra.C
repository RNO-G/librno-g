
R__LOAD_LIBRARY(librno-g.so) 
R__LOAD_LIBRARY(libz.so) 
R__LOAD_LIBRARY(libRootFftwWrapper.so) 

#include <zlib.h> 
#include "../src/rno-g.h" 
#include "FFTtools.h" 

void median_subtract_blocks(int len, int16_t * data, int block_size = 128) 
{
  int istart = 0; 
  std::vector<int16_t> block(block_size); 

  while (istart  <len)
  {
    memcpy(&block[0], data+istart, block_size * sizeof(int16_t)); 
    std::nth_element(block.begin(), block.begin() + block_size/2, block.end()); 
    int16_t median = block[block_size/2]; 
//    printf("%d %f\n", median, TMath::Mean(block_size, data+istart)); 
    for (int i = istart; i < istart+block_size; i++) 
    {
      if (i >= len) break; 
      data[i]-=median; 
    }
    istart+=block_size; 
  }
}

 std::vector<TGraph *> pows; 
void quick_spectra(const char * file, int mask = 16777215, int ev0 = 0, int maxEv = -1, int zero_sub = 0, int median_sub = 0,  double mindb = -10, double maxdb = 30, double freqmin = 30, double freqmax = 1000)
{
  int nplot = __builtin_popcount(mask); 
  printf("nplot=%d\n", nplot); 
 
  gStyle->SetTitleFontSize(0.09); 

  gzFile f = gzopen(file,"r"); 

  rno_g_file_handle_t h; 
  h.type=rno_g_file_handle_t::RNO_G_GZIP; 
  h.handle.gz = f;
  
  rno_g_waveform_t wf;

  int nskip = ev0;
  //skip forward 
  while (nskip-->0) 
  {
    rno_g_waveform_read(h, &wf); 
  }

  if (maxEv < 0) maxEv = INT_MAX; 
  TCanvas * c = new TCanvas("cquick","Quick Spectra", 1920+4,1080+28); 
  if (nplot > 12) 
  {
    c->Divide(4,ceil(nplot/4.),0.001,0.001); 
  }
  else  if (nplot == 12) 
  {
    c->Divide(3,4,0.001,0.001); 
  }

  else  if (nplot == 9) 
  {
    c->Divide(3,3,0.001,0.001); 
  }
  else  if (nplot > 1) 
  {
    c->Divide(2,ceil(nplot/2.),0.001,0.001); 
  }





  int Nev = 0;

  for (auto g : pows) delete g; 
  pows.clear(); 
  

  int iplot = nplot == 1 ? 0 : 1; 
  for (int iev = 0; iev < maxEv-ev0 ; iev++) 
  {

    if (rno_g_waveform_read(h, &wf) <=0) break; 

    Nev++; 

    double abs_max= -2048; 
    double abs_min= 2048; 

    for (int i = 0; i < 24; i++) 
    {
      if (! ( mask & (1 << i))) continue; 
      double sub = 0; 


      if (median_sub) 
      {
        median_subtract_blocks(wf.radiant_nsamples, wf.radiant_waveforms[i], 128); 
      }

      if (zero_sub) 
      {
        sub = TMath::Mean(wf.radiant_nsamples, wf.radiant_waveforms[i]); 
      }


      TGraph * g = new TGraph(wf.radiant_nsamples); 
      for (int j = 0; j < wf.radiant_nsamples; j++) 
      {
        g->SetPoint(j,j/3.2, (wf.radiant_waveforms[i][j]-sub)*1250./2048); 
      
      }

      TGraph * pow = FFTtools::makePowerSpectrumMilliVoltsNanoSecondsdB(g); 

      delete g; 

      if (pows.size() <= i) 
      {
        pows.push_back(pow); 
      }
      else
      {
        for (int j = 0; j < pow->GetN(); j++) 
        {
          pows[i]->GetY()[j] += pow->GetY()[j]; 
        }
        delete pow; 
      }
    }

  }


  for (int i = 0; i < 24; i++) 
  {
      if (! ( mask & (1 << i))) continue; 
     for (int j = 0; j < pows[i]->GetN(); j++) 
     {
         pows[i]->GetY()[j]/=Nev; 
     }

     c->cd(iplot++); 
     pows[i]->SetTitle(Form("CH %d Spectra [navg=%d];MHz; dBmish", i, Nev));
     pows[i]->GetYaxis()->SetTitleOffset(0.6);
     pows[i]->GetYaxis()->SetTitleSize(0.06);
     pows[i]->GetYaxis()->SetLabelSize(0.06);
     pows[i]->GetYaxis()->SetRangeUser(mindb,maxdb);
     pows[i]->GetXaxis()->SetTitleOffset(0.7);
     pows[i]->GetXaxis()->SetTitleSize(0.06);
     pows[i]->GetXaxis()->SetLabelSize(0.05);
     pows[i]->GetXaxis()->SetRangeUser(freqmin,freqmax);
     pows[i]->Draw("alp"); 
     gPad->SetGridx();
     gPad->SetGridy();
  }

}

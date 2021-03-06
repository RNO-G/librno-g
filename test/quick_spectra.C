
R__LOAD_LIBRARY(librno-g.so) 
R__LOAD_LIBRARY(libz.so) 
R__LOAD_LIBRARY(libRootFftwWrapper.so) 

#include <zlib.h> 
#include "../src/rno-g.h" 
#include "FFTtools.h" 

 std::vector<TGraph *> pows; 
void quick_spectra(const char * file, int ev0 = 0, int maxEv = -1, int zero_sub = 0, double mindb = -10, double maxdb = 30, double freqmin = 30, double freqmax = 1000)
{

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
  c->Divide(4,6,0.001,0.001); 

  int Nev = 0;

  for (auto g : pows) delete g; 
  pows.clear(); 
  

  for (int iev = 0; iev < maxEv-ev0 ; iev++) 
  {

    if (rno_g_waveform_read(h, &wf) <=0) break; 

    Nev++; 

    double abs_max= -2048; 
    double abs_min= 2048; 

    for (int i = 0; i < 24; i++) 
    {
      double sub = 0; 

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
     for (int j = 0; j < pows[i]->GetN(); j++) 
     {
         pows[i]->GetY()[j]/=Nev; 
     }

     c->cd(i+1); 
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
  }

}

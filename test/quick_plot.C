
R__LOAD_LIBRARY(librno-g.so) 
R__LOAD_LIBRARY(libz.so) 
R__LOAD_LIBRARY(libRootFftwWrapper.so) 
#include <zlib.h> 
#include "../src/rno-g.h" 
#include "FFTtools.h" 
#include "DigitalFilter.h" 
TCanvas * c = 0; 
std::vector<TGraph*> gs; 
std::vector<TGraph*> envs; 



void quick_plot(const char * file, int ev = 0, int symmetric=1, int Nev = 1, int save = false, int resfactor=1,int mask=16777215, int min_rms_sample = 0, int max_rms_sample = 400, int zero_sub=0)
{

  FFTtools::ButterworthFilter but(FFTtools::LOWPASS, 2, 0.6/1.6); 
  gStyle->SetTitleFontSize(0.09); 

  gzFile f = gzopen(file,"r"); 

  rno_g_file_handle_t h; 
  h.type=rno_g_file_handle_t::RNO_G_GZIP; 
  h.handle.gz = f;
  
  rno_g_waveform_t wf;

  int nplot = __builtin_popcount(mask); 
  printf("nplot=%d\n", nplot); 
  int nskip = ev;
  //skip forward 
  while (nskip-->0) 
  {
    rno_g_waveform_read(h, &wf); 
  }

  for (int iev = 0; iev < Nev; iev++) 
  {

    rno_g_waveform_read(h, &wf); 

    if (c) 
    {
      delete c; 
      c = 0;
    }

    if (gs.size()) 
    {
      for (auto g: gs) delete g; 
      for (auto env: envs) delete env; 
      gs.clear(); 
      envs.clear(); 
    }

    c = new TCanvas("quick_plot","Quick Plot", 1920*resfactor+4,1080*resfactor+28); 
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
    else 
    {
      c->Divide(2,ceil(nplot/2.),0.001,0.001); 
    }



    double abs_max= -2048; 
    double abs_min= 2048; 

    for (int i = 0; i < 24; i++) 
    {

      if (! ( mask & (1 << i))) continue; 

      TGraph * g = new TGraph(wf.radiant_nsamples); 
      g->SetTitle(Form("R%d, E%d, CH%d", wf.run_number, wf.event_number, i)); 
      g->GetXaxis()->SetTitle("sample");
      g->GetYaxis()->SetTitle("ADC");

      g->GetYaxis()->SetTitleOffset(0.6);
      g->GetYaxis()->SetTitleOffset(0.65);
      g->GetYaxis()->SetTitleSize(0.06);
      g->GetYaxis()->SetLabelSize(0.055);
      g->GetXaxis()->SetTitleOffset(0.7);
      g->GetXaxis()->SetTitleSize(0.06);
      g->GetXaxis()->SetLabelSize(0.045);

      bool all_zeroes = true; 

      //check to make sure not all zeroes
      for (int j = 0; j < wf.radiant_nsamples; j++) 
      {
        if (wf.radiant_waveforms[i][j]) 
        {
          all_zeroes = false; 
          break; 
        }
      }

      double sub = 0; 
      if (zero_sub) 
      {
        sub = TMath::Mean(wf.radiant_nsamples, wf.radiant_waveforms[i]); 
      }

      for (int j = 0; j < wf.radiant_nsamples; j++) 
      {
        g->SetPoint(j,j, wf.radiant_waveforms[i][j]-sub); 
      
        if (!all_zeroes) 
        {
          if (wf.radiant_waveforms[i][j] >  abs_max) abs_max = wf.radiant_waveforms[i][j]; 
          if (wf.radiant_waveforms[i][j] <  abs_min) abs_min = wf.radiant_waveforms[i][j]; 
        }
      }

//      TGraph * filtered = new TGraph(g->GetN(), g->GetX(), g->GetY()); 
      TGraph * env = FFTtools::getHilbertEnvelope(g); 
      but.filterGraph(env); 
      envs.push_back(env); 
      gs.push_back(g); 
    }


    double umin = symmetric==1 ? - TMath::Max(-abs_min, abs_max) : abs_min; 
    double umax = symmetric==1 ? TMath::Max(-abs_min, abs_max) : abs_max; 
    if (symmetric < 0) 
    {
      umin = symmetric; 
      umax = -symmetric; 
    }

    printf("%g %g %g %g\n", umin,umax,abs_min,abs_max); 
    umin-= 0.1 * (umax-umin); 
    umax+= 0.1 * (umax-umin); 
    for (unsigned icd = 1; icd <= gs.size(); icd++) 
    {
      TGraph * g = gs[icd-1]; 
      c->cd(icd)->SetLeftMargin(0.08); 
      gPad->SetGridx();
      gPad->SetTickx();
      gPad->SetGridy();
      gPad->SetTicky();
      gPad->SetRightMargin(0.02); 
      gPad->SetBottomMargin(0.1); 
      g->GetXaxis()->SetRangeUser( 0, g->GetN()); 
      g->GetYaxis()->SetRangeUser( umin, umax); 
      int min_samp = min_rms_sample < 0 ? 0 : min_rms_sample; 
      int max_samp = max_rms_sample <= 0 || max_rms_sample > g->GetN() ? g->GetN() : max_rms_sample; 
      TText * t = new TText(3.8*g->GetN()/5.,1.01*umax, Form("RMS=%g", TMath::RMS(max_samp-min_samp, g->GetY()+min_samp))); 
      t->SetTextSize(0.07); 
      g->Draw("alp"); 
      TGraph * env = envs[icd-1]; 
      env->SetLineColor(3); 
      env->SetLineStyle(3); 
      env->Draw("lsame"); 
      t->Draw(); 
    }

    if (save) c->SaveAs(Form("out/c%d.png", ev+iev)); 
  }
}

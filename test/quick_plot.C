
R__LOAD_LIBRARY(librno-g.so) 
R__LOAD_LIBRARY(libz.so) 
#include <zlib.h> 
#include "../src/rno-g.h" 
TCanvas * c = 0; 
    std::vector<TGraph*> gs; 

void quick_plot(const char * file, int ev = 0, int symmetric=1, int Nev = 1, int save = false)
{

  gStyle->SetTitleFontSize(0.09); 

  gzFile f = gzopen(file,"r"); 

  rno_g_file_handle_t h; 
  h.type=rno_g_file_handle_t::RNO_G_GZIP; 
  h.handle.gz = f;
  
  rno_g_waveform_t wf;

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
      gs.clear(); 
    }

    c = new TCanvas("quick_plot","Quick Plot", 1920+4,1080+28); 
    c->Divide(4,6,0.001,0.001); 


    double abs_max= 0; 
    double abs_min= 0; 

    for (int i = 0; i < 24; i++) 
    {


      TGraph * g = new TGraph(wf.radiant_nsamples); 
      g->SetTitle(Form("R%d, E%d, CH%d", wf.run_number, wf.event_number, i)); 
      g->GetXaxis()->SetTitle("Sample");
      g->GetYaxis()->SetTitle("ADC");

      g->GetYaxis()->SetTitleOffset(0.6);
      g->GetYaxis()->SetTitleSize(0.06);
      g->GetYaxis()->SetLabelSize(0.06);
      g->GetXaxis()->SetTitleOffset(0.7);
      g->GetXaxis()->SetTitleSize(0.06);
      g->GetXaxis()->SetLabelSize(0.05);



      for (int j = 0; j < wf.radiant_nsamples; j++) 
      {
        g->SetPoint(j,j, wf.radiant_waveforms[i][j]); 
        if (wf.radiant_waveforms[i][j] >  abs_max) abs_max = wf.radiant_waveforms[i][j]; 
        if (wf.radiant_waveforms[i][j] <  abs_min) abs_min = wf.radiant_waveforms[i][j]; 
      }


      gs.push_back(g); 
    }


    double umin = symmetric ? - TMath::Max(abs_min, abs_max) : abs_min; 
    double umax = symmetric ? TMath::Max(abs_min, abs_max) : abs_max; 
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
      gPad->SetBottomMargin(0.09); 
      g->GetXaxis()->SetRangeUser( 0, g->GetN()); 
      g->GetYaxis()->SetRangeUser( umin, umax); 
      TText * t = new TText(4*g->GetN()/5.,1.01*umax, Form("RMS=%g", g->GetRMS(2))); 
      t->SetTextSize(0.09); 
      g->Draw("alp"); 
      t->Draw(); 
    }

    if (save) c->SaveAs(Form("c%d.png", ev+iev)); 
  }
}

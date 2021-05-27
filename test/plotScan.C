R__LOAD_LIBRARY(librno-g.so) 
R__LOAD_LIBRARY(libz.so) 
#include <zlib.h> 
#include "../src/rno-g.h" 

void plotScan(const char * file, int chmask = 0xffffff, int save_mode = 0) 
{

  gStyle->SetNumberContours(255); 
  gStyle->SetPalette(kSunset); 
  rno_g_file_handle_t h; 
  rno_g_init_handle(&h,file,"r"); 


  rno_g_pedestal ped;

  std::vector<TCanvas*> canvi; 
  std::vector<TMultiGraph*> mgs; 
  std::vector<double> vbias_l;
  std::vector<double> vbias_r;

  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
  {
    if (chmask  & (1 << i))
    {
      TCanvas * c = new TCanvas(Form("c%d",i), Form("Ch. %d", i), save_mode ?4096 : 2048, save_mode ? 2048: 1024); 
      canvi.push_back(c); 
      mgs.push_back(new TMultiGraph(Form("mgch%d",i),Form("Channel %d Scan; sample; adu", i))); 
    }
    else
    {
      canvi.push_back(0);
      mgs.push_back(0);
    }
  }

  while (rno_g_pedestal_read(h,&ped) > 0) 
  {

    double bias_l = ped.vbias[0]/4095.*3.3; 
    double bias_r = ped.vbias[1]/4095.*3.3; 
    vbias_l.push_back(bias_l);
    vbias_r.push_back(bias_r);
    for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
    {
      if (!canvi[i]) continue; 

      TGraph * g = new TGraph(RNO_G_PEDESTAL_NSAMPLES); 
      g->SetTitle(Form("VBias=%g",  i < RNO_G_NUM_RADIANT_CHANNELS / 2 ? bias_l : bias_r)); 

      for (int j = 0; j < RNO_G_PEDESTAL_NSAMPLES; j++) 
      {
        g->SetPoint(j,j, ped.pedestals[i][j]); 
      }

      mgs[i]->Add(g); 
    }
  }

  rno_g_close_handle(&h); 


  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
  {

    if (canvi[i]) 
    {
      canvi[i]->cd(); 

      int nbias = vbias_l.size(); 
      double * vbias = i < RNO_G_NUM_RADIANT_CHANNELS / 2 ? &vbias_l[0] : &vbias_r[0]; 
      TH2 * h = new TH2S(Form("hch%d",i),Form("Channel %d Scan; sample; bias [V]; adu",i), 4096, 0, 4096, vbias_l.size(), vbias[0], 2*vbias[nbias-1] - vbias[nbias-2]); 
      for (int ibias = 0; ibias < nbias; ibias++) 
      {
        TGraph * g = (TGraph*) mgs[i]->GetListOfGraphs()->At(ibias); 
        for (int ix = 1; ix < h->GetNbinsX(); ix++) 
        {
          h->SetBinContent(ix, ibias+1, g->GetY()[ix-1]); 
        }
      }
      h->SetStats(0); 
      h->SetEntries(4096*nbias); 

      h->Draw("colz"); 
      if (save_mode) 
        canvi[i]->SaveAs(Form("scan%d.png",i)); 
    }
  }


} 

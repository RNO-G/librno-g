R__LOAD_LIBRARY(librno-g.so) 
R__LOAD_LIBRARY(libz.so) 
#include <zlib.h> 
#include "../src/rno-g.h" 

void plotScan(const char * file, int chmask = 0xffffff, int save_mode = 0, int nproj = 6, int includebin=-1) 
{

  gStyle->SetNumberContours(255); 
  gStyle->SetPalette(kSunset); 
  gStyle->SetOptFit(1); 
  gStyle->SetOptStat(0); 
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
      if (nproj) 
      {

        canvi[i]->Divide(1,2); 
        canvi[i]->cd(1); 
      }
      else
      {
        canvi[i]->cd(); 
      }


      std::vector<int> proj_bins; 

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

      if (nproj) 
      {
        gRandom->SetSeed(0); 
        while(proj_bins.size() < nproj) 
        {
          int trial = gRandom->Integer(1+h->GetNbinsX()); 
          if (proj_bins.size() ==0 && includebin >=0) trial = includebin+1; 
          bool unique = true; 
          for (auto x : proj_bins) 
          {
            if (x == trial)
            {
              unique = false; 
              break; 
            }
          }
          if (unique)
          {
            proj_bins.push_back(trial); 
            TArrow * arr = new TArrow(h->GetXaxis()->GetBinCenter(trial), 2.7, h->GetXaxis()->GetBinCenter(trial),2.5,0.01); 
            arr->Draw(); 
          }
        }

        std::sort(proj_bins.begin(), proj_bins.end()); 
      }



      if (nproj) 
      {
        canvi[i]->cd(2)->Divide(nproj,2,0.001,0.001); 
        for (int j = 0; j < nproj; j++) 
        {
          canvi[i]->cd(2)->cd(j+1); 

          gStyle->SetStatX(0.5);
          gStyle->SetStatY(0.85);
          TH1 * proj = h->ProjectionY(Form("ch%d_bin_%d", i, proj_bins[j]), proj_bins[j], proj_bins[j]); 
          proj->SetTitle(Form("Bin %d Projection; Bias [V]; ADC", proj_bins[j])); 
          TF1 * fit = new TF1(Form("f%d_bin_%d",i,proj_bins[j]), "pol1",0.2,2.2); 
          proj->Fit(fit,"RW"); 
          proj->Draw(); 

          gPad->SetGridx();
          gPad->SetGridy();
          canvi[i]->cd(2)->cd(nproj+j+1); 
          TH1 * resid = new TH1D(Form("ch%d_bin_%d_resid",i,proj_bins[j]), Form("Residuals for Bin %d;Bias [V]; Linear Fit Residual [ADC]",proj_bins[j]), proj->GetNbinsX(), proj->GetXaxis()->GetXmin(), proj->GetXaxis()->GetXmax()); 
          for (int ix = 1; ix < resid->GetNbinsX(); ix++) resid->SetBinContent(ix, proj->GetBinContent(ix)-fit->Eval(proj->GetBinCenter(ix))); 
          resid->GetYaxis()->SetRangeUser(-60,60); 
          resid->Draw(); 
          gPad->SetGridx();
          gPad->SetGridy();
        }
      }

      if (save_mode) 
        canvi[i]->SaveAs(Form("scan%d.png",i)); 
    }
  }


} 

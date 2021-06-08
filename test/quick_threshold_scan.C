R__LOAD_LIBRARY(librno-g.so) 
R__LOAD_LIBRARY(libz.so) 
#include <zlib.h> 
#include "../src/rno-g.h" 


TCanvas * c= 0; 
std::vector<TGraphAsymmErrors * > gs; 

int palette[24] = { 2,4,6,7,8,9,12,16,41,48,44,49,40,32,20,29,11,42,28,33,46,3,30};
void quick_threshold_scan(const char * file, int plotmask = 0xffffff)
{

  if (gs.size()) 
  {
    for (auto g: gs) delete g; 
    gs.clear(); 
  }

  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
  {
    TGraphAsymmErrors * g = new TGraphAsymmErrors; 
    g->SetName(Form("g_ch%d", i)); 
    g->SetTitle(Form("Ch. %d; threshold [V]; adjusted scaler [Hz]",i)); 
    gs.push_back(g); 
  }

  rno_g_file_handle_t h; 
  rno_g_init_handle(&h, file, "r"); 
  rno_g_daqstatus_t ds; 
  while (rno_g_daqstatus_read(h,&ds) > 0)
  {
    for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
    {
      double V = ds.thresholds[i] * 2.5/(16777215.); 

      int railed = ds.scalers[i] == 65535; 
      double adj_scaler = ds.scalers[i] / ds.scaler_period * (1+ds.prescalers[i]); 
      printf("%d %g %g %d\n", i, V, adj_scaler, railed); 
      gs[i]->SetPoint(gs[i]->GetN(), V, adj_scaler == 0 ? gRandom->Uniform(0.4,0.6) : adj_scaler); 
      gs[i]->SetPointError(gs[i]->GetN()-1,0,0,adj_scaler ? 0.0 :  gs[i]->GetY()[gs[i]->GetN()-1]-0.01, railed? 2e8-adj_scaler :  0.0); 
      gs[i]->SetMarkerColor(palette[i]); 
      gs[i]->SetLineColor(palette[i]); 
      gs[i]->SetMarkerStyle(21); 
    }
  }

  if (c) delete c; 
  c = new TCanvas("cscan","Scan"); 
  TH2F bg("hbg",Form("period=%g s; threshold [V]; adjusted scaler[Hz]",ds.scaler_period), 10, gs[0]->GetX()[0], gs[0]->GetX()[gs[0]->GetN()-1], 1000, 0.01, 2e8); 
  bg.SetStats(0); 
  bg.DrawCopy(); 

  TLegend * leg = new TLegend(0.1,0.5,0.5,0.9); 
  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
  {
    if (plotmask & (1 << i))
    {
      gs[i]->Draw("p |>"); 
      leg->AddEntry(gs[i]); 
    }
  }
  leg->Draw(); 
  c->SetLogy(); 

  rno_g_close_handle(&h); 
}



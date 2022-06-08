R__LOAD_LIBRARY(librno-g.so) 
R__LOAD_LIBRARY(libz.so) 
#include "../src/rno-g.h" 

void quick_peds(const char * file, const char * save = NULL, int chmask=16777215) 
{
  rno_g_file_handle_t h; 
  rno_g_init_handle(&h,file,"r"); 

  rno_g_pedestal ped;
  rno_g_pedestal_read(h,&ped); 
  double bias_l = ped.vbias[0]/4095.*3.3; 
  double bias_r = ped.vbias[1]/4095.*3.3; 

  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++) 
  {
    if (( chmask  & (1 << i)) == 0 ) continue;
    
    TCanvas * c = new TCanvas(Form("peds_%d", i), Form("peds_%d", i), 1800,600); 
    TGraph * g = new TGraph(RNO_G_PEDESTAL_NSAMPLES); 
    g->SetTitle(Form("Ch %d, VBias=%g;sample;count", i,  i < RNO_G_NUM_RADIANT_CHANNELS / 2 ? bias_l : bias_r)); 
    for (int j = 0; j < RNO_G_PEDESTAL_NSAMPLES; j++) 
    {
      g->SetPoint(j,j, ped.pedestals[i][j]); 
    }

    g->Draw("al"); 
    g->GetXaxis()->SetRangeUser(0,4096); 
    c->SetGridx();
    c->SetGridy();
    c->Update();
    if (save) c->SaveAs(Form("%s/ped_%d.png",save,i)); 
  }
}

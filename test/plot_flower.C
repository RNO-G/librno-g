#include <nlohmann/json.hpp>
using json = nlohmann::json;

static void do_plot(const char * name, json & js, int i) 
{

  std::string host;
  js["hostname"].get_to(host);
  auto events = js["events"]; 
  int N = events.size();
  if (i < 0) i+=N; 
  if (i < 0) return; 

  if (i >= N) 
  {
    std::cerr << "Requested entry " << i << " but only " << N << "entries" << std::endl;
    return; 
  }

  auto event = events[i]; 
  bool force = false; 
  event["force"].get_to(force); 
  double when = 0;
  event["when"].get_to(when);
 
  time_t when_t = (int) when;
  TTimeStamp ts(when_t, 1e9*(when-int(when))); 

  TCanvas * c = new TCanvas("c","c",800,800); 

  TPaveText * pt = new TPaveText(0,0.9,1,1,"BR NDC"); 
  pt->AddText(Form("%s (%s)",name, host.c_str() ));
  pt->AddText(force ? "FORCE TRIGGER" : "RF TRIGGER"); 
  pt->AddText(ts.AsString()); 
  pt->Draw();

  TPad * p = new TPad("wf","wf",0,0,1,0.9); 
  p->Draw();
  p->cd();

  p->Divide(2,2);

  for (int ch = 0; ch < 4; ch++)
  {
    std::vector<int> data;

    event[Form("ch%d",ch)].get_to(data);

    TGraph * g = new TGraph(data.size());
    g->SetTitle(Form("CH %d; sample; adc", ch));
    int max = -100;
    int min = 100;
    for (int j = 0; j < data.size(); j++)
    {
      g->SetPoint(j,j, data[j]);
      if (data[j] > max) max = data[j];
      if (data[j] < min) min = data[j];
    }
    p->cd(ch+1);
    g->Draw();
    g->GetXaxis()->SetRangeUser(0, data.size());
    g->GetYaxis()->SetRangeUser(-128, 127);
    gPad->SetGridx();
    gPad->SetTopMargin(0.2);
    gPad->SetGridy();

    TPaveText * stats = new TPaveText(0.1,0.8,0.9,0.9,"BR NDC");
    stats->AddText(Form("Mean: %f, Vpp: %d", g->GetMean(2), max-min));
    stats->AddText(Form("RMS (full): %f  (first half): %f (second_half):%f ", g->GetRMS(2), TMath::RMS(g->GetN()/2,g->GetY()), TMath::RMS(g->GetN()/2,g->GetY()+g->GetN()/2))); 
    stats->Draw();
  }
}

void plot_flower(const char * json_file, int i = 0)
{


  if (json_file == strstr(json_file,"https://") || json_file == strstr(json_file,"http://"))
  {
    TString out = gSystem->GetFromPipe(Form("curl %s", json_file)); 
    json data = json::parse(out.Data());
    do_plot(basename(json_file),data, i);
  }
  else
  {
    std::ifstream ifs(json_file);
    json data = json::parse(ifs);
    do_plot(basename(json_file),data,i);
  }
}

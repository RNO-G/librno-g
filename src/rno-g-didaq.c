#include "rno-g-didaq.h"
#include "rno-g.h"
#include <string.h>
#include "didaq.h"

int didaq_read_event(didaq_dev_t * bd, rno_g_header_t * hd, rno_g_waveform_t * wf)
{

  didaq_event_readout_t rdout = {0};
  for (int i = 0; i < DIDAQ_NUM_CHANNELS; i++)
  {
    rdout.wfs[i] = wf->didaq_waveforms[i];
  }

  int ret = didaq_event_readout(bd, &rdout);
  if (ret) return ret;

  memset(hd, 0, sizeof(*hd));
  memset(wf, 0, sizeof(*wf));

  //note right now only trig_counter is populated...
  wf->event_number = rdout.meta.trig_counter;
  wf->nsamples = rdout.in.len;
  wf->bytes_per_sample = 1;
  wf->sampling_rate = 1000;


  hd->event_number = rdout.meta.trig_counter;
  hd->trigger_number = rdout.meta.trig_counter;

  hd->trigger_mask = (rdout.meta.trig_type & DIDAQ_TRIGGER_PHASED)  ?  rdout.meta.last_beam_pattern  :
                     (rdout.meta.trig_type & (DIDAQ_TRIGGER_COINC0 | DIDAQ_TRIGGER_COINC1)) ? rdout.meta.last_coinc_pattern :
                      0;


  hd->pps_count = rdout.meta.pps_counter;
  hd->sys_clk = rdout.meta.clk_cycles;
  hd->sysclk_last_pps = 0; // didaq sysclk is 0 at each pps
  hd->sysclk_last_last_pps =  UINT_MAX - didaq_get_clock_rate_estimate(bd);  // this makes the math work the same as before... awkward as it is!

  hd->readout_time_secs = rdout.meta.readout_time.tv_sec;
  hd->readout_time_nsecs = rdout.meta.readout_time.tv_nsec;
  hd->readout_elapsed_nsecs =  rdout.meta.readout_time.tv_nsec - rdout.meta.ready_time.tv_nsec + 1e9 * (rdout.meta.readout_time.tv_sec - rdout.meta.ready_time.tv_nsec);
  hd->raw_tinfo  = 0;
  hd->raw_evstatus  = 0;

  hd->trigger_type =  0;
  if (rdout.meta.trig_type & DIDAQ_TRIGGER_SOFT) hd->trigger_type |= RNO_G_TRIGGER_SOFT;
  if (rdout.meta.trig_type & DIDAQ_TRIGGER_EXT) hd->trigger_type |= RNO_G_TRIGGER_EXT;
  if (rdout.meta.trig_type & DIDAQ_TRIGGER_PPS) hd->trigger_type |= RNO_G_TRIGGER_PPS;
  if (rdout.meta.trig_type & DIDAQ_TRIGGER_PHASED) hd->trigger_type |= RNO_G_TRIGGER_RF_LT_PHASED;
  if (rdout.meta.trig_type & DIDAQ_TRIGGER_COINC0) hd->trigger_type |= RNO_G_TRIGGER_RF_RADIANT0;
  if (rdout.meta.trig_type & DIDAQ_TRIGGER_COINC1) hd->trigger_type |= RNO_G_TRIGGER_RF_RADIANT1;

  for (int i = 0; i < RNO_G_NUM_RADIANT_CHANNELS; i++)
  {
    hd->didaq_start_offsets[i] = rdout.in.start;
  }


 return 0;
}


int didaq_read_daqstatus(didaq_dev_t * bd, rno_g_daqstatus_t)
{

  return 0;
}
int didaq_poll_trigger_ready(didaq_dev_t * bd, int timeout_ms)
{
  return didaq_event_wait(bd, timeout_ms / 1000.);
}



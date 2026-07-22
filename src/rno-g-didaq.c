
#include "rno-g-didaq.h"
#ifdef ON_DIDAQ

int didaq_read_event(didaq_dev_t * bd, rno_g_header_t * hd, rno_g_waveform_t * wf)
{

  didaq_event_readout_t rdout = {0};
  for (int i = 0; i < DIDAQ_NUM_CHANNELS; i++)
  {
    rdout.wfs[i] = wf->didaq_waveforms[i];
  }

  int ret = didaq_event_readout(bd, &rdout);
  if (ret) return ret;

  wf->event_number = rdout.meta.event_counter;
  wf->nsamples = rdout.in.len;
  wf->bytes_per_sample = 1;
  wf->sampling_rate = 1000;


  hd->event_number = rdout.meta.event_counter;


  /** TODO fill in rest / figure out what else we need to do this */

  return 0;

}


int didaq_read_daqstatus(didaq_dev_t * bd, rno_g_daqstatus_t)
{
}

int didaq_poll_trigger_ready(didaq_dev_t * bd, int timeout_ms)
{
  return didaq_event_wait(bd, timeout_ms / 1000.);
}

uint16_t didaq_get_sample_rate(const didaq_dev_t * bd)
{
  return 1000;
}

int didaq_get_fw_version(const didaq_dev_t * bd,
    uint8_t * major, uint8_t *minor, uint8_t* rev,
    uint8_t * year_minus_2000, uint8_t *month, uint8_t * day)
{
  return 0;
}


#endif

#ifndef _RNO_G_DIDAQ_H
#define _RNO_G_DIDAQ_H

#include "rno-g.h"
typedef struct didaq_dev didaq_dev_t;

/** RNO-G shims for libdidaq ,part of librno-g since needs to be kept in sync with data format.
 */


// like radiant_soft_trigger
#define didaq_soft_trigger didaq_force_trigger

int didaq_read_event(didaq_dev_t * bd, rno_g_header_t * hd, rno_g_waveform_t * wf);
int didaq_read_daqstatus(didaq_dev_t * bd, rno_g_daqstatus_t);
int didaq_poll_trigger_ready(didaq_dev_t * bd, int timeout_ms);

uint16_t didaq_get_sample_rate(const didaq_dev_t * bd); //Sample rate in MHz

int didaq_get_fw_version(const didaq_dev_t * bd,
    uint8_t * major, uint8_t *minor, uint8_t* rev,
    uint8_t * year_minus_2000, uint8_t *month, uint8_t * day);

#endif

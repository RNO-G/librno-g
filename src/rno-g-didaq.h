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



#endif

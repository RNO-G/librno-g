#ifndef _RNO_G_DIDAQ_H
#define _RNO_G_DIDAQ_H

#include "rno-g.h"
typedef struct didaq_dev didaq_dev_t;

/** RNO-G shims for libdidaq, part of librno-g since needs to be kept in sync with data format.
 *
 * The shims that touch per-channel quantities take the station number, because the
 * DiDAQ <-> RNO-G channel permutation they apply is station-dependent (see
 * rno_g_didaq_chanmap() in rno-g.h). They are the only place that permutation happens:
 * everything on the rno_g_* side of these calls is in RNO-G channel numbering.
 */


// like radiant_soft_trigger
#define didaq_soft_trigger didaq_force_trigger

int didaq_read_event(didaq_dev_t * bd, rno_g_header_t * hd, rno_g_waveform_t * wf, uint8_t station);
int didaq_read_daqstatus(didaq_dev_t * bd, rno_g_daqstatus_t * ds, uint8_t station);

/** Push the thresholds held in ds to the hardware; the write counterpart of
 *  didaq_read_daqstatus().
 *
 *  ds->didaq_coin_thresholds is indexed by RNO-G channel and gets permuted into DiDAQ input
 *  numbering here; the per-beam thresholds need no permutation. set_phased/set_coinc select
 *  which of the two threshold groups is actually written, mirroring didaq_set_thresholds(),
 *  which skips a group passed as NULL.
 */
int didaq_write_thresholds(didaq_dev_t * bd, const rno_g_daqstatus_t * ds, uint8_t station,
                           int set_phased, int set_coinc);

int didaq_poll_trigger_ready(didaq_dev_t * bd, int timeout_ms);

uint16_t didaq_get_sample_rate(const didaq_dev_t * bd); //Sample rate in MHz

// int didaq_get_fw_version(const didaq_dev_t * bd,
//     uint8_t * major, uint8_t *minor, uint8_t* rev,
//     uint8_t * year_minus_2000, uint8_t *month, uint8_t * day);

#endif

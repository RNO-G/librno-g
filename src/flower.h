#ifndef __flower_h__
#define __flower_h__


/**
 * FLOWER API (RNO-G Experiment)
 * Cosmin Deaconu
 * cozzyd@kicp.uchicago.edu
 *
 */

#include <stdio.h>
#include "rno-g.h"

typedef struct flower_dev flower_dev_t;

/** Assumes already configured!
 * @param spi_device the name of the SPI device (e.g. /dev/spidev2.0 )
 * @param spi_enable name of the spi enable gpio (or 0 for none). Use negative for active low.
 * */
flower_dev_t *  flower_open(const char * spi_device, int spi_enable);

int flower_close(flower_dev_t * dev);

int flower_dump(FILE* f, flower_dev_t *dev);

int flower_fill_daqstatus(flower_dev_t *dev, rno_g_daqstatus_t * ds);

int flower_configure_trigger(flower_dev_t* dev, rno_g_lt_simple_trigger_config_t  cfg, rno_g_lt_phased_trigger_config_t phased_cfg); 

int flower_set_coinc_thresholds(flower_dev_t *dev, const uint8_t * trigger_thresholds, const uint8_t * servo_thresholds, uint8_t mask); 

int flower_set_phased_thresholds(flower_dev_t *dev, const uint16_t * phased_trigger_thresholds, const uint16_t * phased_servo_thresholds, uint16_t mask); 

int flower_fill_header(flower_dev_t *dev, rno_g_header_t * hd);

// if preclear non-zero (not default), then buffer cleared in same spi transaction as sending trigger, minimzing chance of accidental RF trigger.
void flower_enable_force_trigger_preclear(flower_dev_t *dev, int preclear);

int flower_force_trigger(flower_dev_t *dev);

int flower_buffer_check(flower_dev_t * dev, int * avail);

int flower_buffer_clear(flower_dev_t * dev);

int flower_read_waveforms(flower_dev_t * dev, int len, uint8_t ** dest);


// WARNING experimental, ABI may change without notice
typedef struct flower_waveform_metadata
{
  uint32_t event_counter;
  uint32_t trigger_counter;
  uint32_t pps_counter;// this doesn't seem to work?
  uint8_t trigger_type; //1=soft,2=ext,3=coinc,4=phased,5=pps. see enum below.
  uint8_t pps_flag;
  uint64_t timestamp; //48 bit, will roll over
  uint64_t recent_pps_timestamp; //48 bit, will roll over.
                                 //Note that this is recent at readout time, not trigger time, so could be after the trigger timestamp.
} flower_waveform_metadata_t;

enum
{
  FLOWER_TRIG_NONE,
  FLOWER_TRIG_SOFT,
  FLOWER_TRIG_EXT,
  FLOWER_TRIG_COINC,
  FLOWER_TRIG_PHASED,
  FLOWER_TRIG_PPS,
  FLOWER_TRIG_INVALID
} flower_trigger_type;

const char * flower_trigger_type_as_string(uint8_t type);

// EXPERIMENTAL
int flower_read_waveform_metadata(flower_dev_t * dev, flower_waveform_metadata_t * meta);

enum
{
  FLOWER_GAIN_1X,
  FLOWER_GAIN_1_25X,
  FLOWER_GAIN_2X,
  FLOWER_GAIN_2_5X,
  FLOWER_GAIN_4X,
  FLOWER_GAIN_5X,
  FLOWER_GAIN_8X,
  FLOWER_GAIN_10X,
  FLOWER_GAIN_12_5X,
  FLOWER_GAIN_16X,
  FLOWER_GAIN_20X,
  FLOWER_GAIN_25X,
  FLOWER_GAIN_32X,
  FLOWER_GAIN_50X,
  FLOWER_GAIN_TOO_HIGH
} flower_gain_codes;

int flower_set_gains(flower_dev_t * dev, const uint8_t * gain_codes);
int flower_set_fine_gains(flower_dev_t *dev, uint8_t * sub_numerators);

typedef union flower_word
{
  uint32_t word;
  uint8_t bytes[4];
}flower_word_t;

int flower_read_register(flower_dev_t*dev, uint8_t addr, flower_word_t * result);


enum
{
  FLOWER_EQUALIZE_EXCLUDE_CH0 = 1,
  FLOWER_EQUALIZE_EXCLUDE_CH1 = 2,
  FLOWER_EQUALIZE_EXCLUDE_CH2 = 4,
  FLOWER_EQUALIZE_EXCLUDE_CH3 = 8,
  FLOWER_EQUALIZE_VERBOSE = 0x80000000
}e_flower_equalize_opts;

int flower_equalize(flower_dev_t*dev, float target_rms, uint8_t * gain_codes, int opts, int do_fine_gain_adjust, uint8_t * v_fine_gain_number);

typedef struct  flower_trigger_enables
{
  uint8_t enable_pps : 1; 
  uint8_t enable_ext : 1; 
  uint8_t enable_coinc : 1; 
  uint8_t enable_phased : 1;
} flower_trigger_enables_t; 


// This is used to determine what forms internal triggers on the FLOWER, though the coinc is required to pass it to trigout too since otherwise it's not formed.
int flower_set_trigger_enables(flower_dev_t * dev, flower_trigger_enables_t enables);

/**Set the delayed PPS delay. The delay is in multiples of 100 ns*/
int flower_set_delayed_pps_delay(flower_dev_t * dev, uint32_t delay);
int flower_get_delayed_pps_delay(flower_dev_t * dev, uint32_t  *delay);


typedef struct  flower_trigout_enables
{
  uint8_t enable_rf_sysout : 1;
  uint8_t enable_rf_auxout : 1;
  uint8_t enable_pps_sysout : 1;
  uint8_t enable_pps_auxout : 1;
} flower_trigout_enables_t;

int flower_set_trigout_enables(flower_dev_t * dev, flower_trigout_enables_t enables);

int flower_get_fwversion(flower_dev_t *dev, uint8_t *major, uint8_t *minor, uint8_t *rev, uint16_t *year, uint8_t *month, uint8_t *day);

int flower_get_fwversion_int(flower_dev_t *dev, int *version_int);
#endif

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


/** Assumes already configured */ 
flower_dev_t *  flower_open(const char * spi_device, int spi_enable); 

int flower_close(flower_dev_t * dev); 

int flower_dump(FILE* f, flower_dev_t *dev); 

int flower_soft_trigger(flower_dev_t*dev, int trig); 

int flower_configure_trigger(flower_dev_t* dev, rno_g_lt_simple_trigger_config_t cfg); 

int flower_set_thresholds(flower_dev_t *dev, const uint8_t * trigger_thresholds, const uint8_t * servo_thresholds, uint8_t mask); 

int flower_fill_daqstatus(flower_dev_t *dev, rno_g_daqstatus_t * ds); 

int flower_fill_header(flower_dev_t *dev, rno_g_header_t * hd); 

int flower_force_trigger(flower_dev_t *dev); 

int flower_read_waveforms(flower_dev_t * dev, int len, uint8_t ** dest); 

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


typedef union flower_word
{
  uint32_t word; 
  uint8_t bytes[4]; 
}flower_word_t; 

int flower_read_register(flower_dev_t*dev, uint8_t addr, flower_word_t * result); 
   


#endif


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

typedef union flower_word
{
  uint32_t word; 
  uint8_t bytes[4]; 
}flower_word_t; 

int flower_read_register(flower_dev_t*dev, uint8_t addr, flower_word_t * result); 
   


#endif


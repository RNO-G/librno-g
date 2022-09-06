/** C rno_g_cal driver, for both revd and reve 
 * */ 
#include <stdint.h> 
#include "rno-g.h" 

#ifndef _RNO_G_CAL_H
#define _RNO_G_CAL_H
#ifdef __cplusplus
extern "C"
{ 
#endif


typedef struct rno_g_cal_dev rno_g_cal_dev_t; 

rno_g_cal_dev_t * rno_g_cal_open(uint8_t i2c_bus,uint16_t gpio, char board_rev); 

int rno_g_cal_close(rno_g_cal_dev_t* dev); 


int rno_g_cal_enable(rno_g_cal_dev_t *dev); 
int rno_g_cal_disable(rno_g_cal_dev_t *dev); 

int rno_g_cal_setup(rno_g_cal_dev_t* dev); 

int rno_g_cal_read_temp(rno_g_cal_dev_t * dev, float *T); 

int rno_g_cal_select(rno_g_cal_dev_t * dev, rno_g_calpulser_out_t ch); 

int rno_g_cal_set_atten(rno_g_cal_dev_t * dev, uint8_t atten); //attenuation, in half dB steps. 0 = no attenuation, 63=31.5 dB

int rno_g_cal_set_pulse_mode(rno_g_cal_dev_t * dev, rno_g_calpulser_mode_t type); 


void rno_g_cal_enable_dbg(rno_g_cal_dev_t * dev, int dbg); 

#ifdef __cplusplus
}
#endif

#endif


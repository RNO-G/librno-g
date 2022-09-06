/** C rno_g_cal driver, for both revd and reve 
 * */ 
#include <stdint.h> 

#ifndef _RNO_G_CAL_H
#define _RNO_G_CAL_H
#ifdef __cplusplus
extern "C"
{ 
#endif


typedef struct rno_g_cal_dev rno_g_cal_dev_t; 

typedef enum rno_g_cal_channel
{
  RNOG_CAL_CH_NONE,
  RNOG_CAL_CH_COAX,  
  RNOG_CAL_CH_FIBER0, 
  RNOG_CAL_CH_FIBER1 

} rno_g_cal_channel_t; 

typedef enum rno_g_cal_pulse_type
{
  RNOG_CAL_PULSE,
  RNOG_CAL_VCO,
  RNOG_CAL_VCO2 //revE only 
} rno_g_cal_pulse_type_t; 



rno_g_cal_dev_t * rno_g_cal_open(uint8_t i2c_bus,uint16_t gpio, char board_rev); 

int rno_g_cal_close(rno_g_cal_dev_t* dev); 


int rno_g_cal_enable(rno_g_cal_dev_t *dev); 
int rno_g_cal_disable(rno_g_cal_dev_t *dev); 

int rno_g_cal_setup(rno_g_cal_dev_t* dev); 

int rno_g_cal_read_temp(rno_g_cal_dev_t * dev, float *T); 

int rno_g_cal_select(rno_g_cal_dev_t * dev, rno_g_cal_channel_t ch); 

int rno_g_cal_set_atten(rno_g_cal_dev_t * dev, uint8_t atten); //attenuation, in half dB steps. 0 = no attenuation, 63=31.5 dB

int rno_g_cal_set_pulse_type(rno_g_cal_dev_t * dev, rno_g_cal_pulse_type_t type); 

int rno_g_cal_set_pulse(rno_g_cal_dev_t *dev); 
int rno_g_cal_set_vco(rno_g_cal_dev_t *dev); 
int rno_g_cal_set_vco2(rno_g_cal_dev_t *dev); 

void rno_g_cal_enable_dbg(rno_g_cal_dev_t * dev, int dbg); 

#ifdef __cplusplus
}
#endif

#endif


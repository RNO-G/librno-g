#ifndef _RNO_G_RADIANT_H
#define _RNO_G_RADIANT_H

/** Radiant API 
 *  
 *  Cosmin Deaconu 
 *  cozzyd@kicp.uchicago.edu 
 *
 * */


#ifdef __cplusplus
extern "C" {
#endif

#include "rno-g.h" 

/*---------------------------------------------------------------------
*
* High-level RADIANT functions first 
*
-------------------------------------------------------------------------------*/


/* 
 * Opaque handle to RADIANT device.  
 */ 
typedef struct radiant_dev radiant_dev_t; 

/** 
 * Open a radiant device, returning an opaque handle if successful or NULL if not. 
 *
 * @param spi_device  The device name of the spidev driver used  for communicating 
 *                    with the RADIANT (e.g. /dev/spi/0.0) 
 * @param uart_device The device name of the UART device used for communcating with 
 *                    the RADIANT board manager (e.g. /dev/ttyO5) 
 * @returns an opaque handle, or NULL if something failed (probably accompanied by 
 *                    nasty log messages). 
 *
 */
radiant_dev_t * radiant_open (const char * spi_device, const char * uart_device); 



/** Dump some debug information to stream*/

typedef enum 
{
  RADIANT_DUMP_UPDATE_GPIOS  = 1  /// update the gpios (instead of assuming they're in sync... 

}e_radiant_dump_flags; 

int radiant_dump(radiant_dev_t* dev, FILE * stream, int flags); 
 

/** 
 *  Close a radiant device 
 *  @param dev  Opaque handle to device to close.  
 */ 
void radiant_close(radiant_dev_t * dev) ; 


/** Set the run number of the board 
 *
 *  @param bd The opaque handle for the board
 *  @param run The run number to set to 
 *
 **/ 
void radiant_set_run_number(radiant_dev_t * bd, int run); 


/** 
 * Check if an event is available, returning the number of bytes available to read 
 * @param bd The opaque handle for the board
 * @returns the number of bytes available to read (or negative on error) 
 *
 **/ 
int radiant_check_avail(radiant_dev_t * bd); 

/** 
 * High-level read event interface (if there's anything to read). 
 * Note that this optimistically tries to read the header without first checking 
 * if there's * anything to read so if there is no data available it takes a 
 * little bit longer than radiant_check_avail().  
 *
 * @param bd opaque handle for the board
 * @param hd The RNO-G header to populate (if there's something available) 
 * @param wf The RNO-G waveforms to populate (if there's something available) 
 * @returns 0 on success, 1 if nothing available, or negative on error
 */
int radiant_read_event(radiant_dev_t * bd, rno_g_header_t * hd, rno_g_waveform_t *wf); 



/***********************************************************
 *
 * Now the high-level commands to the board manager 
 *
 ********************************************************/ 


/** These are the status bits in the radiant board manager status */ 
typedef enum 
{
 RADIANT_BM_ST_FPGA_DONE = (1 << 0),  /// 1 if FPGA configuration complete
 RADIANT_BM_ST_MGT_DET   = (1 << 1),  /// 0 if MGT connection detected
 RADIANT_BM_ST_SD_DETECT = (1 << 2),  /// 1 if uSD is inserted into BM 
 RADIANT_BM_ST_SD_PG1V0  = (1 << 3),  /// 1.0 V power is good
 RADIANT_BM_ST_SD_PG1V8  = (1 << 4),  /// 1.8 V power is good
 RADIANT_BM_ST_SD_PG2V5  = (1 << 5),  /// 2.5 V power is good
 RADIANT_BM_ST_SD_PG2V6  = (1 << 6),  /// 2.6 V power is good
 RADIANT_BM_ST_SD_PG3V1  = (1 << 7)   /// 3.1 V power is good
} e_radiant_bm_status_bits; 


/** Retrieves  the board manager status, whose bits can be queried
 *
 *  using the e_radiant_bm_control_bits enum
 *
 * @param bd opaque board handle 
 * @param status the status bits will be placed here if susccessful
 * @returns 0 on success. 
 *
 **/ 
int radiant_bm_get_status(radiant_dev_t* bd, uint8_t * status); 


/** The board manager voltages available to read */ 
typedef enum 
{
  RADIANT_BM_ANALOG_V10, 
  RADIANT_BM_ANALOG_V18, 
  RADIANT_BM_ANALOG_V25, 
  RADIANT_BM_ANALOG_LEFTMON, 
  RADIANT_BM_ANALOG_RIGHTMON
} radiant_bm_analog_rd_t; 

/** Read an analog voltage */ 
int radiant_bm_analog_read( radiant_dev_t * bd, radiant_bm_analog_rd_t what, float * v);  


typedef enum 
{
  RADIANT_BM_CTRL_EN_10MHz = 1 << 2, 
  RADIANT_BM_CTRL_BURST = 1 << 3 
} e_radiant_bm_control_bits;


int radiant_bm_set_ctrl(radiant_dev_t *bd, uint8_t ctrl); 
int radiant_bm_get_ctrl(radiant_dev_t *bd, uint8_t *ctrl); 


int radiant_enable_cal_mode(radiant_dev_t * bd, int quad); 
int radiant_disable_cal_mode(radiant_dev_t * bd, int quad); 



/** WARNING: These calibration methods are untested. 
 * It is probably better to use the radsig-cli until these are tested */ 
typedef enum
{
  RADIANT_CAL_50_100, 
  RADIANT_CAL_100_300, 
  RADIANT_CAL_300_600, 
  RADIANT_CAL_600_PLUS
} radiant_cal_band_t; 

typedef enum
{
  RADIANT_CAL_PULSE,
  RADIANT_CAL_SINE
} radiant_cal_pulse_type_t; 

typedef struct
{
  radiant_cal_band_t band; 
  radiant_cal_pulse_type_t pulse_type; 
} radiant_cal_config_t;  

// Sets the calpulser band / pulse or sinewave
int radiant_configure_cal(radiant_dev_t *bd, const radiant_cal_config_t * cfg ) ;

//enables or disables the signal generator
int radiant_enable_cal(radiant_dev_t* bd, int enable) ; 

// sets the frequency of the sine wave
int radiant_set_frequency(radiant_dev_t *bd, float freq_MHz, float * actual_freq_MHz); 

/**** END WARNING*****/


typedef enum 
{
  RADIANT_ATTEN_TRIG, 
  RADIANT_ATTEN_SIG
} radiant_atten_t; 
int radiant_set_attenuator(radiant_dev_t *bd, int channel, radiant_atten_t which, uint8_t value); 



typedef struct
{
  uint8_t dma_enable :1; 
  uint8_t dma_busy :1; 
  uint8_t ext_dma_req_enable :1; 
  uint8_t dma_direction :1 ; 
  uint8_t reserved :1; 
  uint8_t byte_mode :1; 
  uint8_t byte_target_in_byte_mode :2; 
  uint8_t enable_spi_receive : 1; 
  uint8_t cycle_delay : 7; 
  uint16_t tx_full_flag_threshold : 11; 
  uint8_t reserved2 : 4; 
  uint8_t tx_full_flag_value : 1; 
  uint8_t tx_full_flag_enable : 1; 
} radiant_dma_config_t; 

int radiant_get_dma_config(radiant_dev_t * bd, radiant_dma_config_t * cfg) ; 
int radiant_configure_dma(radiant_dev_t *bd, const radiant_dma_config * cfg);  







typedef enum radiant_dest
{
  DEST_FPGA = 0, 
  DEST_MANAGER =1 
} radiant_dest_t ; 


/** low-level radiant UART methods
 *
 * @param bd radiant board
 * @param dest either the FPGA or the board manager 
 * @param addr start addrress
 * @param len the length to write (max 255...) 
 * @param bytes the data to write 
 **/ 
int radiant_set_mem(radiant_dev_t* bd, radiant_dest_t dest, uint32_t addr, uint8_t len, const uint8_t * bytes); 
int radiant_get_mem(radiant_dev_t* bd, radiant_dest_t dest, uint32_t addr, uint8_t len, uint8_t * bytes); 



/** 
 * low-level radiant read command via SPI bus.
 *
 * @param  bd device handle
 * @param  num_avail_bytes  if non-zero, number of available bytes will be written to this memory location
 * @param  n_read_buffers   the number of buffers to read (this does NOT check navail so could write garbage). 
 * @param  read_n_words  pointer to array of number of words to read into each buffer
 * @param  buffers  array of pointers to buffers 
 * @returns whatever the ioctl returns
 * */ 
int radiant_read(radiant_dev_t * bd, uint16_t * num_avail_bytes, 
                 int n_read_buffers, 
                 int *  read_n_words, 
                 uint16_t ** buffers);


/** change betweeen peek and consume read modes */ 
void radiant_set_read_mode(radiant_dev_t *bd, int peek); 

int radiant_clear(radiant_dev_t *bd);
int radiant_reset(radiant_dev_t *bd); 
int radiant_rewind(radiant_dev_t *bd); 




#ifdef __cplusplus
}
#endif


#endif

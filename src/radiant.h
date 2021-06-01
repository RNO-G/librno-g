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
 * @param trigger_gpio The gpio we wait on to tell if we have a trigger. Use 0 or negative to not support (I hope there is no GPIO 0?). 
 * @param spi_enable_gpio. If zero ignored. If positive, will be pulled up, if negative, will be pulled down. 
 * @returns an opaque handle, or NULL if something failed (probably accompanied by 
 *                    nasty log messages). 
 *
 */
radiant_dev_t * radiant_open (const char * spi_device, const char * uart_device, int trigger_gpio, int spi_enable_gpio); 


/* Sets a callback on the trigger gpio interrupt. Note 
 * that this implies spawning a separate thread, so you want to be careful in your callback here. 
 * Set callback to NULL to unset.  For a blocking interface, use radiant_poll_trigger_ready (which is incompatible with a callback since they will poll on the same fdset) 
 *
 **/ 
int radiant_set_trigger_ready_callback(radiant_dev_t *bd, void (*callback)(radiant_dev_t *, void *), void * aux);  

/** THIS IS MUTUALLY EXCLUSIVE WITH radiant_set_gpio_callback, if a callback has been set this will fail with EBUSY.
 *
 * This sets up a poll on the interrupt gpio and returns when done (or interrupted by a signal handler). 
 *
 *
 * @param bd  radiant handle
 * @parame timeout timeout for poll (negative for infinite) in ms 
 * @return Returns negative on error, 1 if an interrupt did happen, 0 if poll ended for some other reason (e.g. signal or timeout expired). 
 *
 * */ 
int radiant_poll_trigger_ready(radiant_dev_t*bd, int timeout_ms); 


/** Set the number of buffers per readout. Valid choices are 1 (for 1024-sample mode) or 2 (for 2048-sample mode).
 * Other values will do nothing. 
 **/ 
int radiant_set_nbuffers_per_readout(radiant_dev_t *bd, int nbuffers) ; 


/** Dump some debug information to stream*/
typedef enum 
{
  RADIANT_DUMP_UPDATE_GPIOS  = 1  /// update the gpios (instead of assuming they're in sync)... 

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
 * @returns 1 if event available, 0 if not, negative on error
 *
 **/ 
int radiant_check_avail(radiant_dev_t * bd); 

int radiant_labs_start(radiant_dev_t*bd); 
int radiant_labs_clear(radiant_dev_t*bd); 
int radiant_labs_stop(radiant_dev_t*bd); 

/** 
 * High-level read event interface
 * THIS DOESN'T ACTUALLY CHECK IF THERE'S ANYTHING TO READ. DO THAT EITHER WITH check_avail or poll. 
 *
 * @param bd opaque handle for the board
 * @param hd The RNO-G header to populate (if there's something available) 
 * @param wf The RNO-G waveforms to populate (if there's something available) 
 * @returns 0 on success, 1 if nothing available, or negative on error
 */
int radiant_read_event(radiant_dev_t * bd, rno_g_header_t * hd, rno_g_waveform_t *wf); 

int radiant_read_daqstatus(radiant_dev_t * bd, rno_g_daqstatus_t* ds); 

/* Send an INTERNAL (not a SOFT trigger, this is used internally for calram and such. This may be removed from the API in the future. Max on howmany is 256
 * DOES not automatically generate a DMA read; 
 * */ 
int radiant_internal_trigger(radiant_dev_t *bd, int howmany, int block); 


/** Sends a soft trrgger */
int radiant_soft_trigger(radiant_dev_t *bd); 

/** Checks if trigger is busy */ 
int radiant_trigger_busy(radiant_dev_t * bd, int * bsy); 



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
  uint8_t endianness :1; 
  uint8_t byte_mode :1; 
  uint8_t byte_target_in_byte_mode :2; 
  uint8_t enable_spi_receive : 1; 
  uint8_t cycle_delay : 7; 
  uint16_t tx_full_flag_threshold : 11; 
  uint8_t reserved2 : 3; 
  uint8_t tx_full_flag_value : 1; 
  uint8_t tx_full_flag_enable : 1; 
} radiant_dma_config_t; 

typedef enum 
{
  RADIANT_DMA_EVENT_MODE, 
  RADIANT_DMA_CAL_MODE, 
  RADIANT_DMA_CPLD_MODE, 
  RADIANT_DMA_LAB4D_MODE, 
  RADIANT_DMA_CAL_WRITE_MODE
} radiant_dma_mode_preset_t; 
int radiant_fill_dma_config(radiant_dma_config_t *cfg, radiant_dma_mode_preset_t preset); 

int radiant_get_dma_config(radiant_dev_t * bd, radiant_dma_config_t * cfg) ; 
int radiant_configure_dma(radiant_dev_t *bd, const radiant_dma_config_t * cfg);  

typedef struct 
{
  uint8_t tx_reset : 1; 
  uint8_t rx_reset : 1; 
  uint8_t engine_reset : 1; 
  uint8_t dma_req: 1; 
  uint8_t padding: 4;
}radiant_dma_ctrl_t; 


int radiant_dma_control(radiant_dev_t *bd, radiant_dma_ctrl_t ctrl); 

int radiant_dma_engine_reset(radiant_dev_t *bd); 
int radiant_dma_tx_reset(radiant_dev_t *bd); 
int radiant_dma_rx_reset(radiant_dev_t *bd); 
int radiant_dma_request(radiant_dev_t *bd); 
int radiant_dma_txn_count_reset(radiant_dev_t *bd); 
int radiant_dma_txn_count(radiant_dev_t *bd, uint32_t * count); 


typedef struct
{
  uint32_t addr; 
  uint16_t length;
  uint8_t incr ;
  uint8_t last ;
}radiant_dma_desc_t; 
/** Low-level Radiant DMA  descriptor setup */ 
int radiant_dma_set_descriptor(radiant_dev_t *bd, uint8_t idescr,radiant_dma_desc_t desc); 


/** Sets up Radiant DMA for event reading with the appropriate channel_mask */
int radiant_dma_setup_event(radiant_dev_t *bd, uint32_t channel_mask); 



/** Pedestal running. Note that this leaves the DMA set up in pedestal mode so you probably need to set up event mode after */ 
int radiant_compute_pedestals(radiant_dev_t *bd, uint32_t mask, uint16_t ntriggers, rno_g_pedestal_t * ped); 

/** This sets the pedestals, which will be automatically subtracted from each event. Note that this is not copied so must have a long lifetime.  Set to NULL if you want no pedestal subtraction. */ 
void radiant_set_pedestals(radiant_dev_t *bd , const rno_g_pedestal_t * peds); 

/** pointer to pedestals currently in use */
const rno_g_pedestal_t * radiant_get_pedestals(radiant_dev_t *bd); 


/** This resets the event fifo counters (I think?) */ 
int radiant_reset_counters(radiant_dev_t *bd); 

/** Synchronizes counters at next pps */ 
int radiant_sync(radiant_dev_t *bd); 

typedef struct radiant_pending
{
  uint8_t pending : 6; 
  uint8_t fifo_empty : 1; 
  uint8_t pending_empty : 1; 
} radiant_pending_t; 
int radiant_get_pending(radiant_dev_t *bd, radiant_pending_t * pending); 

/** Get current times */ 
int radiant_current_pps(radiant_dev_t * bd, uint32_t *pps, uint32_t *sysclk_last_pps, uint32_t *sysclk_last_last_pps); 

int radiant_reset_readout_fifo(radiant_dev_t *bd, int force, int reset_readout);


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
int radiant_set_mem_unacked(radiant_dev_t* bd, radiant_dest_t dest, uint32_t addr, uint8_t len, const uint8_t * bytes); 

int radiant_get_mem(radiant_dev_t* bd, radiant_dest_t dest, uint32_t addr, uint8_t len, uint8_t * bytes); 


int radiant_set_trigger_thresholds_float(radiant_dev_t * bd, int ichan_start, int chan_end,  const float * threshold_V); 
int radiant_get_trigger_thresholds_float(radiant_dev_t * bd, int ichan_start, int ichan_end, float * threshold_V); 


int radiant_set_trigger_thresholds(radiant_dev_t * bd, int ichan_start, int chan_end,  const uint32_t * threshold_V); 

typedef enum
{
  RADIANT_THRESHOLD_UNKNOWN=0xffffffff 
} e_radiant_thresh_consts; 
int radiant_get_trigger_thresholds(radiant_dev_t * bd, int ichan_start, int ichan_end, uint32_t * threshold_V); 


typedef enum
{
  RADIANT_TRIG_EN = 1, //global enable
  RADIANT_TRIG_EXT = 2, //extin trig enable
  RADIANT_TRIG_PPS = 4, //pps trig enable
  RADIANT_TRIGOUT_EN = 256, //global enable
  RADIANT_TRIGOUT_SOFT = 512, //global enable
  RADIANT_TRIGOUT_PPS = 1024, //global enable
  RADIANT_TRIG_CPUFLOW = 65536, //global enable
  RADIANT_TRIG_QUERY = 16777216, // If set, will ignore rest of bits and just return what is currently set(or RADIANT_TRIG_QUERY if there's an error)
  RADIANT_TRIG_DISMASK_QUERY = 33554432 // If set, will ignore rest of bits and just return the trig disable mask. 
 
} e_radiant_trig_enables;

int radiant_trigger_enable(radiant_dev_t * bd, int enables, uint32_t disable_mask); 

/** Sets the trigout length, will be rounded to nearest 10 ns */ 
int radiant_trigout_set_length(radiant_dev_t *bd, int ns);
int radiant_trigout_get_length(radiant_dev_t *bd, int * ns);

/** Use 0 to use pps for period instead of clock */ 
int radiant_set_scaler_period(radiant_dev_t * bd, float period); 
int radiant_get_scaler_period(radiant_dev_t * bd, float *period); 

int radiant_set_prescaler(radiant_dev_t * bd, int scaler, uint8_t prescale_minus_one); 

int radiant_get_scalers(radiant_dev_t * bd, int start, int end, uint16_t * scalers); 


int radiant_cpu_clear(radiant_dev_t *bd); 
int radiant_cpu_clear_pending(radiant_dev_t *bd, int * pending); 

typedef enum radiant_trig_sel
{
  RADIANT_TRIG_A, 
  RADIANT_TRIG_B 
} radiant_trig_sel_t; 


/** Note that this will temporary disable internal triggering while setup 
 *
 * This configures RF trigger A or B
 * @param bd the radaiant handle
 * @param which RADIANT_TRIG_A or RADIANT_TRIG_B
 * @param contributing_channels_mask mask of channels that contribute to this trigger. Set to 0 to disable this trigger. 
 * @param required_coincident_channels the threshold for number of coincident channels 
 * @param coincidence_window_ns the length of the coincidence window in nanoseconds. This will be rounded to the nearest 2.5 ns. Min is 17.5 ns, max is 327.5
 *
 **/
int radiant_configure_rf_trigger(radiant_dev_t * bd, 
    radiant_trig_sel_t which, 
    uint32_t contributing_channels_mask, 
    uint8_t required_coincident_channels, 
    float coincidence_window_ns); 


/** 
 * low-level radiant read command via SPI bus. this assumes that the descriptors have already been set up correctly.
 * Up to 511 read buffers can be supported in the same transaction , anything greater is truncated
 *
 * @param  bd device handle
 * @param  n_read_buffers   the number of buffers to read (this does NOT check navail so could write garbage). 
 * @param  read_n_bytes  pointer to array of number of BYTES to read into each buffer
 * @param  buffers  array of pointers to buffers 
 * @returns whatever the ioctl returns
 * */ 
int radiant_read(radiant_dev_t * bd, int n_read_buffers, uint16_t *  read_n_bytes, uint8_t ** buffers);


/** Sets the dc bias of left/right labs. 0-4095 representing a 3.3V range */ 
int radiant_set_dc_bias(radiant_dev_t * bd, uint16_t left, uint16_t right); 

/** Sets the trigger diode bias , 0-4095 representing a 2 V range. chan is 0-indexed. */ 
int radiant_set_td_bias(radiant_dev_t * bd, int chan,  uint16_t val); 



#ifdef __cplusplus
}
#endif


#endif


/** RNO-G in-memory API/data structures 
 * In flux for now! 
 *
 * (C) 2020 RNO-G Collaboration
 * Cosmin Deaconu <cozzyd@kicp.uchicago.edu> 
 *
 *
 * */ 


#ifndef _RNO_G_H
#define _RNO_G_H

#ifdef __cplusplus
extern "C"
{ 
#endif



//For int ttypes
#include <stdint.h> 

//for CHAR_BIT
#include <limits.h>   

//for FILE* 
#include <stdio.h> 

#define RNO_G_MAX_RADIANT_NSAMPLES 2048
#define RNO_G_LAB4D_NSAMPLES 4096
#define RNO_G_PEDESTAL_NSAMPLES RNO_G_LAB4D_NSAMPLES
#define RNO_G_NUM_RADIANT_CHANNELS 24
#define RNO_G_RADIANT_NBUFS 4

#define RNO_G_MAX_LT_NSAMPLES 512
#define RNO_G_NUM_LT_CHANNELS 4
#define RNO_G_NUM_LT_BEAMS 16


/** Forward declarations of file backends, because some may be conditionally compiled in the future */ 
typedef struct gzFile_s * gzFile;

/* File handles so that we can handle different compression schemes in the same way. 
 *
 * For example, if you have a compressed file full of header packets, you can do: 
 *
 *   gzFile zf = gzopen("headers.gz"."r"); 
 *   rno_g_file_handle h { .type = RNO_G_GZIP, .handle.gz= zf}; 
 *
 *   rno_g_header_t header;
 *   while( rno_g_header_read(h,&header)) 
 *   { 
 *     //do something 
 *   }
 *
 *
 *
 */

typedef struct rno_g_file_handle
{
  enum
  {
    RNO_G_RAW,
    RNO_G_GZIP 
  } type; 

  union 
  {
    FILE * raw; 
    gzFile gz; 
  } handle; 
} rno_g_file_handle_t; 


//convenience maker for rno_g_file_handle (useful for python wrapper too) 

int rno_g_init_handle(rno_g_file_handle_t * h, const char * name, const char * mode); 
int rno_g_close_handle(rno_g_file_handle_t * h); 
#define rno_g_handle_is_open(h) (h.handle.raw)


typedef enum rno_g_trigger_type 
{
    RNO_G_TRIGGER_SOFT         = 1 << 0,  /**< This was a software trigger */
    RNO_G_TRIGGER_EXT          = 1 << 1,  /**< This was an external trigger (note that the LT is implemented as an external trigger, but this is not it!) */
    RNO_G_TRIGGER_RF_LT_SIMPLE = 1 << 2,  /**< This was an RF trigger from the LT board*/
    RNO_G_TRIGGER_RF_LT_PHASED = 1 << 3,  /**< This was an RF trigger from the LT board*/
    RNO_G_TRIGGER_RF_RADIANT0  = 1 << 4,  /**< This was an RF trigger from the RADIANT's trigger 0*/
    RNO_G_TRIGGER_RF_RADIANT1  = 1 << 5,  /**< This was an RF trigger from the RADIANT's trigger 1*/
    RNO_G_TRIGGER_RF_RADIANTX  = 1 << 6,  /**< This was a RF trigger from the RADIANT. The reason this is exists is that due to unreliable bits, we don't know if it was 0 or 1 */
    RNO_G_TRIGGER_PPS          = 1 << 7   /**< This was a PPS trigger*/
} rno_g_trigger_type_t ; 



typedef enum rno_g_flags
{
    RNO_G_FLAG_GATE           =  1 << 0, /**< This occured during PPS gate */ 
    RNO_G_READOUT_ERROR       =  1 << 1, /**< There was some kind of readout error*/ 
    //this will be stored as a uint8_t, so can fit a few more 
}rno_g_flags_t; 


typedef struct rno_g_lt_simple_trigger_config
{
  uint8_t window; 
  uint8_t num_coinc : 3; // require > number of coincidences  
  uint8_t vpp_mode : 1; 
  uint8_t channel_mask: 7; 
} rno_g_lt_simple_trigger_config_t;

typedef struct rno_g_lt_phased_trigger_config
{
  uint16_t beam_mask: 15;
  //maybe if delays are adjustable?
} rno_g_lt_phased_trigger_config_t;

typedef struct radiant_trigger_config
{
  uint32_t mask : 24; 
  uint8_t ncoinc: 5; // if 0, then disabled
  uint8_t window: 7; //ns = 17.5 + 2.5 * window, max = 124
} rno_g_radiant_trigger_config_t; 

typedef enum radiant_trigger_info 
{ 
    RADIANT_TRIGGER_INT0 = 1 << 0, 
    RADIANT_TRIGGER_INT1 = 1 << 1, 
    RADIANT_TRIGGER_EXT = 1 << 2, 
    RADIANT_TRIGGER_SOFT = 1 << 3, 
    RADIANT_TRIGGER_PPS = 1 << 4 , 
    RADIANT_TRIGGER_INTERNAL_COPY = 1 << 5 
} rno_g_radiant_trigger_info_t; 

 /** 
  * The RNO-G event header. Each event has one of these.
  * This is the in-memory format; the on-disk format is slightly different (and includes version information/checksum/magic numbers. See doc/data-format.txt
  * Use the opaque handles for read/write). 
  *
  */
typedef struct rno_g_header 
{
  uint32_t event_number;  //!< Event number (per run, 0-indexed) 
  uint32_t trigger_number;//!< Trigger  number (per run, 0-indexed), including triggers not read out due to deadtime
  uint32_t run_number;    //!< Run number , assigned at startup 

  uint32_t trigger_mask;  //!< Which channels (or beams?) caused the trigger
  uint32_t trigger_value; //!< Relevant for LT trigger only, probably. Something like the beam power? 
  uint32_t sys_clk;  //!< Trigger time tag  (number of clock cycles since start of run or since PPS? If since start of run, may need to be 64-bit) 
  uint32_t pps_count;     //!< Number of PPS's since start of run
  uint32_t readout_time_secs; // !< readout START time in secondpart
  uint32_t readout_time_nsecs; // !< readout START time, nanosecond part 
  uint32_t readout_elapsed_nsecs; // How long it took to do the readout syscall 
  uint32_t sysclk_last_pps;  //!< sysclk time at last PPS
  uint32_t sysclk_last_last_pps; //!< sysclk time at last last PPS
  uint32_t raw_tinfo;       //!< the raw trigger info. To  be figured out. 
  uint32_t raw_evstatus;    //!< the raw event status flags. To be figured out. 

  uint8_t station_number; //!< The station number. 

  /** Trigger type. See rno_g_trigger_type_t  Or-able */ 
  uint8_t trigger_type; 

  /** Various flags for the event. See rno_g_flags_t orable */ 
  uint8_t flags; 
  uint8_t pretrigger_windows; //!< Number of pretrigger windows? 
  uint8_t radiant_start_windows[RNO_G_NUM_RADIANT_CHANNELS][2]; //!<this encodes buffer number too. There are two of these because of 2048-sample readout works. The second one will be 0xff (255) if we are in 1024-mode. 
  uint16_t radiant_nsamples; //!< Number of samples per channel in RADIANT board (could just keep this in waveform if we wanted)
  uint16_t lt_nsamples; //!< Number of samples per channel in low-threshold board  (could just keep this in waveform if we wanted)

  rno_g_lt_simple_trigger_config_t lt_simple_trigger_cfg; 
  rno_g_lt_phased_trigger_config_t lt_phased_trigger_cfg;
  rno_g_radiant_trigger_config_t radiant_trigger_cfg[2]; 

} rno_g_header_t; 

//write in ascii format 
int rno_g_header_dump(FILE*f, const rno_g_header_t * header);
//write in binary format
int rno_g_header_write(rno_g_file_handle_t handle, const rno_g_header_t * header);
int rno_g_header_read(rno_g_file_handle_t handle, rno_g_header_t * header);

typedef struct radiant_readout_delay_struct
{
  //keep as channels in case the groups change
  //delays are max 7 bit ints 
  uint32_t rf0_delay_mask;
  uint32_t rf1_delay_mask;
  uint8_t rf0_delay;
  uint8_t rf1_delay;

} radiant_readout_delay_t;

typedef struct rno_g_waveform
{
  uint32_t event_number; //!< For matching
  uint32_t run_number;   //!< For matching
  uint16_t radiant_nsamples; //!< Number of samples per channel for RADIANT
  uint16_t lt_nsamples; //!< Number of samples per channel for lowthresh
  int16_t radiant_waveforms[RNO_G_NUM_RADIANT_CHANNELS][RNO_G_MAX_RADIANT_NSAMPLES]; //unrolled. 
  uint8_t lt_waveforms[RNO_G_NUM_LT_CHANNELS][RNO_G_MAX_LT_NSAMPLES]; // 8-bit digitizer 
  uint16_t radiant_sampling_rate;
  uint8_t digitizer_readout_delay[RNO_G_NUM_RADIANT_CHANNELS];
  uint8_t station; 
  //radiant_readout_delay_t radiant_readout_delays;
  
} rno_g_waveform_t; 

//write in ascii format (e.g. for stdout) 
int rno_g_waveform_dump(FILE *f, const rno_g_waveform_t * waveform);
// write in binary format 
int rno_g_waveform_write(rno_g_file_handle_t handle, const rno_g_waveform_t * waveform);
int rno_g_waveform_read(rno_g_file_handle_t handle, rno_g_waveform_t * waveform);


typedef struct rno_g_pedestal 
{
  uint32_t when; 
  uint32_t nevents; 
  uint32_t mask : 24; 
  uint8_t flags; //TBD 
  int16_t vbias[2];  //signed so that we can have -1 as unknown
  uint16_t pedestals[RNO_G_NUM_RADIANT_CHANNELS][RNO_G_PEDESTAL_NSAMPLES]; 
  uint8_t station; 
  int run; 
} rno_g_pedestal_t; 

int rno_g_pedestal_dump(FILE *f, const rno_g_pedestal_t * pedestal); 
int rno_g_pedestal_write(rno_g_file_handle_t handle, const rno_g_pedestal_t * pedestal);
int rno_g_pedestal_read(rno_g_file_handle_t handle, rno_g_pedestal_t * pedestal);




typedef struct rno_g_lt_scaler_group
{
  uint16_t trig_coinc; 
  uint16_t trig_per_chan[RNO_G_NUM_LT_CHANNELS]; 
  uint16_t servo_coinc; 
  uint16_t servo_per_chan[RNO_G_NUM_LT_CHANNELS]; 
  uint16_t trig_phased;
  uint16_t trig_per_beam[RNO_G_NUM_LT_BEAMS];
  uint16_t servo_phased;
  uint16_t servo_per_beam[RNO_G_NUM_LT_BEAMS];

} rno_g_lt_scaler_group_t; 


typedef struct rno_g_lt_scalers
{
  rno_g_lt_scaler_group_t s_1Hz; 
  rno_g_lt_scaler_group_t s_1Hz_gated; 
  rno_g_lt_scaler_group_t s_100Hz; 
  uint64_t ncycles : 48 ;  //this is the 118 MHz clock
  uint16_t scaler_counter_1Hz : 16; 
  uint64_t cycle_counter;  // cycle counter, reset on each PPS 
} rno_g_lt_scalers_t; 


// these are all 16-bit relative to 3.3 V rail
typedef struct rno_g_radiant_voltages
{
  uint16_t V_1_0; 
  uint16_t V_1_8; 
  uint16_t V_2_5; 
  uint16_t V_LeftMon; 
  uint16_t V_RightMon; 
} rno_g_radiant_voltages_t; 


typedef enum 
{
  RNO_G_CAL_NO_OUTPUT = 0, 
  RNO_G_CAL_COAX, 
  RNO_G_CAL_FIB0, 
  RNO_G_CAL_FIB1
} rno_g_calpulser_out_t; 

#define RNO_G_CALPULSER_OUT_STRS {"none","coax","fiber0", "fiber1"}

typedef enum 
{
  RNO_G_CAL_NO_SIGNAL = 0, 
  RNO_G_CAL_PULSER, 
  RNO_G_CAL_VCO,
  RNO_G_CAL_VCO2
} rno_g_calpulser_mode_t; 

#define RNO_G_CALPULSER_MODE_STRS {"none","pulser","vco", "vco2"}


typedef struct rno_g_calpulser_info
{
  int16_t T_times_16 : 12;  // Temperature of calboard, times 16 
  rno_g_calpulser_mode_t mode : 2;  //calpulser mode 
  uint8_t enabled: 1; //calpulser turned on
  uint8_t atten_times_2 : 6;  // attenuation of calpulser, times 2 
  rno_g_calpulser_out_t out: 2; //which output did we use? 

  char rev; //Rev of controller board (if 0, this means that we didn't fill this in) 
} rno_g_calpulser_info_t; 

typedef struct rno_g_daqstatus 
{
  double when_radiant; 
  double when_lt; 
  uint32_t radiant_thresholds[RNO_G_NUM_RADIANT_CHANNELS]; 
  uint16_t radiant_scalers[RNO_G_NUM_RADIANT_CHANNELS]; 
  uint8_t radiant_prescalers[RNO_G_NUM_RADIANT_CHANNELS]; 
  float radiant_scaler_period; 
  uint8_t  lt_trigger_thresholds[RNO_G_NUM_LT_CHANNELS]; 
  uint8_t  lt_servo_thresholds[RNO_G_NUM_LT_CHANNELS]; 
  uint16_t lt_phased_trigger_thresholds[RNO_G_NUM_LT_BEAMS];
  uint16_t lt_phased_servo_thresholds[RNO_G_NUM_LT_BEAMS];
  rno_g_lt_scalers_t lt_scalers; 
  rno_g_radiant_voltages_t radiant_voltages; 
  rno_g_calpulser_info_t cal; 
  uint8_t station;
} rno_g_daqstatus_t; 

int rno_g_daqstatus_dump(FILE *f, const rno_g_daqstatus_t * ds); 
int rno_g_daqstatus_dump_flower(FILE *f, const rno_g_daqstatus_t * ds); 
int rno_g_daqstatus_dump_radiant(FILE *f, const rno_g_daqstatus_t * ds); 
int rno_g_daqstatus_dump_calpulser(FILE *f, const rno_g_daqstatus_t * ds); 
int rno_g_daqstatus_write(rno_g_file_handle_t handle, const rno_g_daqstatus_t * ds);
int rno_g_daqstatus_read(rno_g_file_handle_t handle, rno_g_daqstatus_t * ds);


const char * rno_g_get_git_hash(); 


#ifdef __cplusplus
}
#endif
#endif 

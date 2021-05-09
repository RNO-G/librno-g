
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

#define RNO_G_MAX_RADIANT_NSAMPLES 2048
#define RNO_G_PEDESTAL_NSAMPLES 2048
#define RNO_G_NUM_RADIANT_CHANNELS 24 
#define RNO_G_NUM_RADIANT_WINDOWS 16
#define RNO_G_RADIANT_WINDOW_SIZE 128

#define RNO_G_MAX_LT_NSAMPLES 512
#define RNO_G_NUM_LT_CHANNELS 4 



/** Forward declarations of file backends, because some may be conditionally compiled in the future */ 
typedef struct gzFile_s * gzFile;
typedef struct _IO_FILE FILE;

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


typedef enum rno_g_trigger_type 
{
    RNO_G_TRIGGER_SOFT        = 1 << 0,  /**< This was a software trigger */
    RNO_G_TRIGGER_EXT         = 1 << 1,  /**< This was an external trigger */
    RNO_G_TRIGGER_RF_LT       = 1 << 2,  /**< This was an RF trigger from the LT board*/
    RNO_G_TRIGGER_RF_RADIANT  = 1 << 3,  /**< This was an RF trigger from the RAdiant*/
    RNO_G_TRIGGER_RF_FOLLOWUP = 1 << 4   /**< This is a ``followup" trigger*/

      //this will be stored as a uint8_t, so can fit a few more 
} rno_g_trigger_type_t ; 


typedef enum rno_g_flags
{
    RNO_G_FLAG_GATE           =  1 << 0, /**< This occured during PPS gate */ 
    RNO_G_READOUT_ERROR       =  1 << 1, /**< There was some kind of readout error*/ 
    //this will be stored as a uint8_t, so can fit a few more 
}rno_g_flags_t; 


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
  uint32_t trigger_time;  //!< Trigger time tag  (number of clock cycles since start of run or since PPS? If since start of run, may need to be 64-bit) 
  uint32_t pps_count;     //!< Number of PPS's since start of run

  uint8_t station_number; //!< The station number. 

  /** Trigger type. See rno_g_trigger_type_t  Or-able */ 
  uint8_t trigger_type; 

  /** Various flags for the event. See rno_g_flags_t orable */ 
  uint8_t flags; 

  uint8_t pretrigger_windows; //!< Number of pretrigger windows? 
  uint8_t  radiant_start_windows[RNO_G_NUM_RADIANT_CHANNELS]; //!<this encodes buffer number too 
  uint16_t radiant_nsamples; //!< Number of samples per channel in RADIANT board (could just keep this in waveform if we wanted)
  uint16_t lt_nsamples; //!< Number of samples per channel in low-threshold board  (could just keep this in waveform if we wanted)

} rno_g_header_t; 

//write in ascii format 
int rno_g_header_dump(FILE*f, const rno_g_header_t * header);
//write in binary format
int rno_g_header_write(rno_g_file_handle_t handle, const rno_g_header_t * header);
int rno_g_header_read(rno_g_file_handle_t handle, rno_g_header_t * header);

typedef struct rno_g_waveform
{
  uint32_t event_number; //!< For matching
  uint32_t run_number;   //!< For matching
  uint16_t radiant_nsamples; //!< Number of samples per channel for RADIANT
  uint16_t lt_nsamples; //!< Number of samples per channel for RADIANT
  int16_t radiant_waveforms[RNO_G_NUM_RADIANT_CHANNELS][RNO_G_MAX_RADIANT_NSAMPLES]; //unrolled. 
  uint8_t lt_waveforms[RNO_G_NUM_LT_CHANNELS][RNO_G_MAX_LT_NSAMPLES]; // 8-bit digitizer 
} rno_g_waveform_t; 


typedef struct rno_g_pedestal 
{
  uint32_t when; 
  uint16_t nevents; 
  uint16_t flags; //TBD 
  uint16_t pedestals[RNO_G_NUM_RADIANT_CHANNELS][RNO_G_PEDESTAL_NSAMPLES]; 
} rno_g_pedestal_t; 

int rno_g_pedestal_dump(FILE *f, const rno_g_pedestal_t * pedestal); 
int rno_g_pedestal_write(rno_g_file_handle_t handle, const rno_g_pedestal_t * pedestal);
int rno_g_pedestal_read(rno_g_file_handle_t handle, rno_g_pedestal_t * pedestal);


//write in ascii format (e.g. for stdout) 
int rno_g_waveform_dump(FILE *f, const rno_g_waveform_t * waveform);
// write in binary format 
int rno_g_waveform_write(rno_g_file_handle_t handle, const rno_g_waveform_t * waveform);
int rno_g_waveform_read(rno_g_file_handle_t handle, rno_g_waveform_t * waveform);







#ifdef __cplusplus
}
#endif
#endif 

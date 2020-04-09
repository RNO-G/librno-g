
/** RNO-G in-memory API/data structures 
 * In flux for now! 
 *
 * (C) 2020 RNO-G Collaboration
 * Cosmin Deaconu <cozzyd@kicp.uchicago.edu> 
 *
 * */ 



#ifndef _RNO_G_H
#define _RNO_G_H
#ifdef __cplusplus__
extern "C"
{ 
#endif


#include <stdint.h> 
#include <stdio.h> 
#include <zlib.h> 

#define RNO_G_MAX_RADIANT_NSAMPLES 2048
#define RNO_G_NUM_RADIANT_CHANNELS 24 
#define RNO_G_NUM_WINDOWS 16
#define RNO_G_WINDOW_SIZE 128


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





 /** 
  * The RNO-G event header. Each event has one of these.
  * This is the in-memory format; the on-disk format is slightly different (and includes version information/checksum/magic numbers. 
  * Use the opaque handles for read/write). 
  *
  *
  */
typedef struct rno_g_header 
{
  uint32_t event_number;  //!< Event number (per run, 0-indexed) 
  uint32_t trigger_number;//!< Trigger  number (per run, 0-indexed), including non-readout triggers
  uint32_t run_number;    //!< Run number 

  uint32_t trigger_mask;  //!< Which channels (or beams) caused the trigger
  uint32_t trigger_value; //!< Relevant for LT trigger only, probably.
  uint32_t trigger_time;  //!< Trigger time tag 
  uint8_t gate_flag;      //!< Was the gate on during trigger? 
  uint8_t station_number; //!< The station number 
  uint8_t pretrigger_windows; //!< Number of pretrigger windows 
  enum 
  {
    RNO_G_TRIGGER_SOFT,
    RNO_G_TRIGGER_EXT,
    RNO_G_TRIGGER_PPS,
    RNO_G_TRIGGER_RF_LT, 
    RNO_G_TRIGGER_RF_ENV 
  } trigger_type : CHAR_BIT; 

  uint8_t  radiant_start_windows[RNO_G_NUM_RADIANT_CHANNELS]; 
  uint16_t radiant_nsamples; ///!< have this here too 

} rno_g_header_t; 

int rno_g_header_write(rno_g_file_handle_t handle, const rno_g_header_t * header);
int rno_g_header_read(rno_g_file_handle_t handle, rno_g_header_t * header);


typedef struct rno_g_waveform
{
  uint32_t event_number; 
  uint32_t run_number;   
  uint16_t nsamples; 
  uint16_t reserved; //alignment 
  uint16_t radiant_waveforms[RNO_G_NUM_RADIANT_CHANNELS][RNO_G_MAX_RADIANT_NSAMPLES]; //unrolled. 
 // uint16_t lt_waveforms[??][??]; 

} rno_g_waveform_t; 


int rno_g_waveform_write(rno_g_file_handle_t handle, const rno_g_waveform_t * waveform);
int rno_g_waveform_read(rno_g_file_handle_t handle, rno_g_waveform_t * waveform);







#ifdef __cplusplus__
}
#endif
#endif 

#ifndef _RNO_G_RADIANT_H
#define _RNO_G_RADIANT_H

/** Radiant API */


#include "rno-g.h" 

//opaque handle
typedef struct radiant_dev radiant_dev_t; 

//eventually this will take the UART interface too 
radiant_dev_t * radiant_open (const char * spi_device, const char * uart_device); 

void radiant_close(radiant_dev_t * dev) ; 

void radiant_set_run_number(radiant_dev_t * bd, int run); 

/** low-level radiant read command 
 *  
 *
 * @param  bd device handle
 * @param  num_avail_bytes  if non-zero, number of available bytes will be written to this memory location
 * @param  n_read_buffers   the number of buffers to read (this does NOT check navail so could write garbage). 
 * @param  read_words  pointer to array of words to read into each buffer
 * @param  buffers  array of pointers to buffers 
 * @returns whatever the ioctl returns
 * */ 
int radiant_read(radiant_dev_t * bd, uint16_t * num_avail_bytes, 
                 int n_read_buffers, 
                 int *  read_words, 
                 uint16_t ** buffers);


/** change betweeen peek and consume read modes */ 
void radiant_set_read_mode(radiant_dev_t *bd, int peek); 

int radiant_clear(radiant_dev_t *bd);
int radiant_reset(radiant_dev_t *bd); 
int radiant_rewind(radiant_dev_t *bd); 




/** Check if an event is available, returning the number of bytes available to read */ 
int radiant_check_avail(radiant_dev_t * bd); 

// read event (if there's anything to read). 
// note that this tries to read the header, so if there is no data available it takes a little bit longer than radiant_check_avail
// returns 0 on success, 1 if nothing available, or negative on error
int radiant_read_event(radiant_dev_t * bd, rno_g_header_t * hd, rno_g_waveform_t *wf); 





#endif

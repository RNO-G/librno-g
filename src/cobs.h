#ifndef _RNO_G_COBS_H_
#define _RNO_G_COBS_H_


#include <stdint.h>

//cobs encode to a stream

int cobs_encode(int input_len, const uint8_t*input,
                 int(*put_bytes)(int,const uint8_t*,void*),
                 void* aux);

//cobs encode a buffer
int cobs_encode_buf(int input_len,
                    const uint8_t * input,
                    int output_capacity,
                    uint8_t * output);


//cobs write to file descriptor
int cobs_encode_write(int len, const uint8_t * input, int fd);


int cobs_decode(int output_capacity, uint8_t * output, int(*get_bytes)(int, uint8_t*,void*), void * aux );

int cobs_decode_buf(int input_len,
                  const uint8_t * input,
                  int output_capacity,
                  uint8_t * output);

int cobs_decode_read(int capacity, uint8_t * buf, int fd);



#endif

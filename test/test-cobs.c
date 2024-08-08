#include "cobs.h" 
#include <string.h> 
#include <stdio.h>


void print_buf(int len, const uint8_t * bytes)
{

  for (int i = 0; i < len; i++) 
  {
    printf("%02x ", bytes[i]); 
  }
  printf("\n"); 

}


struct buf
{
  uint8_t * ptr; 
  int pos; 
  int len; 
}; 

int buf_put_bytes(int N, const uint8_t * bytes, void * vbufptr) 
{
  struct buf * bufptr = (struct buf *) vbufptr; 
  int written = 0; 
  int i = 0; 
  while (bufptr->pos < bufptr->len && i < N) 
  {
    bufptr->ptr[bufptr->pos++] = bytes[i++]; 
    written++;
  }
  return written; 
}


int buf_get_bytes(int N, uint8_t * bytes, void * vbufptr) 
{
  struct buf * bufptr = (struct buf *) vbufptr; 
  int rd = 0; 
  int i = 0; 
  while (bufptr->pos < bufptr->len && i < N) 
  {
    bytes[i++] = bufptr->ptr[bufptr->pos++]; 
    rd++; 
  }
  return rd; 
}



int main() 
{

  const uint8_t foo[] = { 0,1, 4,5,9,20,20,0,20,10,0,255}; 



  uint8_t encoded[512] = {0}; 
  uint8_t decoded[512] = {0}; 
  uint8_t cb_encoded[512] = {0}; 
  uint8_t cb_decoded[512] = {0}; 

  struct buf cb_encoded_buf = {.ptr = cb_encoded, .pos = 0, .len = 512 }; 

  int buf_len = cobs_encode_buf(sizeof(foo), foo, 512, encoded); 
  int buf_len_cb = cobs_encode(sizeof(foo), foo, buf_put_bytes, &cb_encoded_buf); 
  cb_encoded_buf.pos = 0; 

  int len = cobs_decode_buf(buf_len, encoded, 512, decoded); 
  int len_cb = cobs_decode(512, cb_decoded, buf_get_bytes, &cb_encoded_buf); 


  printf("%lu : %d %d: %d %d\n",sizeof(foo),buf_len, buf_len_cb, len, len_cb); 
  print_buf(sizeof(foo),foo);
  print_buf(buf_len,encoded);
  print_buf(buf_len_cb,cb_encoded);
  print_buf(len,decoded);
  print_buf(len_cb,cb_decoded);

  if (len != sizeof(foo) || len_cb != sizeof(foo))  return -1; 
  return memcmp(foo,decoded, sizeof(foo)) + memcmp(foo,cb_decoded, sizeof(foo));

} 

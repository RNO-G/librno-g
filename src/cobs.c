/** Constant overhead byte stuffing implementation 
 *
 * Based on wikipedia implementation, with some additional checks. 
 *
 * */ 

#include "cobs.h" 
#include <unistd.h> 




static uint8_t zero; 

int cobs_encode(int len, const uint8_t * ptr, int (*put_bytes)(int,const uint8_t*,void*), void *aux)
{
  int written = 0; 

  uint8_t code = 1; 
  const uint8_t * out = ptr; 
  const uint8_t * end = ptr+len; 

  while (ptr < end) 
  {

    //find next 0 or saturate code
    while (*ptr++ && code++ < 0xff && ptr < end); 

    if(1!=put_bytes(1,&code, aux)) return -written; 
    written++; 


    if (code-1!=put_bytes(code-1, out, aux)) return -written; 

    written+=code-1; 
    out+=code; 
    code =1; 
  }

  if (1!=put_bytes(1,&zero, aux)) return -written; 

  written++; 

  return written; 
}


static int write_put_char(int N, const uint8_t *c, void* vfd) 
{
  int fd = *((int*) vfd);
  return write(fd,c,N); 
}

int cobs_encode_write(int len, const uint8_t * input, int fd) 
{
  return cobs_encode(len, input, write_put_char, &fd); 
}


int cobs_encode_buf(int input_len, 
                const uint8_t * input, 
                int output_capacity,
                uint8_t * out) 
{

  if (!output_capacity) return -1; 

  const uint8_t *start = out; 
  output_capacity--; 
  uint8_t * code_ptr = out++;  
  uint8_t code = 1; 


  while(input_len--) 
  {
    if (!output_capacity--) 
    {
      return -1; 
    }
    if (*input && code < 0xff) 
    {
      code++; 
      *out++=*input++; 
    }
    else 
    {
      *code_ptr = code; 
      code = 1; 
      code_ptr = out++; 
      input++; 
    }

  }

  *code_ptr = code; 

  if (!output_capacity--) 
  {
    return -1; 
  }

  *out++  = 0; 

  return out-start; 
}

int cobs_decode(int cap, uint8_t *out,  int (*get_bytes)(int N,uint8_t*, void*), void * aux)
{

  uint8_t v;
  int start_cap = cap; 
  uint8_t code = 0xff; 
  int copy = 0; 
  int sub = 0; 
  while (1==get_bytes(1,&v, aux))
  {
    if (copy) 
    {
      if (!cap--) return -1; 
      *out++=v; 
    }
    else
    {
      if (code != 0xff)
      {
        if (!cap--) return -1; 
        *out++=0; 
        sub = 1; 
      }
      code = v; 
      copy = code; 
      if (!code) 
      {
        break; 
      }
    }

    copy--; 
  }
  return start_cap-cap-sub; 
}


static int read_get_char(int N, uint8_t * v, void *vfd) 
{
  int fd = *((int*) vfd); 
  return read(fd, v,N); 
}

int cobs_decode_read(int cap, uint8_t * buf, int fd) 
{
  return cobs_decode(cap, buf, read_get_char, &fd); 
}



int cobs_decode_buf(int input_len, const uint8_t * in, 
                int output_capacity, uint8_t * out) 
{
  const uint8_t * start = out; 
  const uint8_t * end = in + input_len; 
  uint8_t code = 0xff; 
  int copy = 0; 
  int sub = 0; 

  while (in < end) 
  {
    if (copy!= 0) 
    {
      if (!output_capacity--) return -1; 
      *out++ = *in++; 
    }
    else 
    {
      if (code !=0xff) //put in a zero
      {
        if (!output_capacity--) return -1; 
        *out++= 0; 
        sub=1; 
      }
      code = *in++; 
      copy = code; 

      if (!code) 
      {
        break; 
      }
    }

    copy--; 
  }

  return out-start-sub; 

}


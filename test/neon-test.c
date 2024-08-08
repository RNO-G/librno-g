#include <stdio.h> 
#include <string.h> 
#include <assert.h> 
#include <stdint.h> 

#if defined(__arm__) 
#include <arm_neon.h> 
#endif

#define NOVECTORIZE

static void andall16(uint16_t *v, uint16_t andme, int N) 
{

  int i = 0;
#if defined(__arm__) && !defined(NOVECTORIZE) 
//we'll use ARM NEON intrinsics. Since usually this will be a multiple of 32, we'll use vandq_u16 unrolled by 4 
  int Niter = N/32; 

  if (Niter) 
  {
    //probably a smarter way to do this 
    uint16x8_t vandme = vdupq_n_u16 (andme); 

    for (i = 0; i < Niter; i++) 
    {
      uint16x8_t v1 = vld1q_u16(v+i*32);
      uint16x8_t v2 = vld1q_u16(v+i*32+8);
      uint16x8_t v3 = vld1q_u16(v+i*32+16);
      uint16x8_t v4 = vld1q_u16(v+i*32+24);
      v1 = vandq_u16(v1,vandme);
      v2 = vandq_u16(v2,vandme);
      v3 = vandq_u16(v3,vandme);
      v4 = vandq_u16(v4,vandme);
      vst1q_u16( v+i*32,v1);
      vst1q_u16(v+i*32+8,v2);
      vst1q_u16(v+i*32+16,v3);
      vst1q_u16(v+i*32+24,v4);
    }

    i*=32; // catch any remaining elements with a normal loop
  }
#endif 


  // finish the last (or do all of them, if not using intrinsics) 
  for (; i < N; i++) 
  {
    v[i] &= andme; 
  }
}


static void bswap16(uint16_t *vv, int N) 
{
  int i = 0;

#if defined(__arm__) && !defined(NOVECTORIZE) 
  uint8_t*v = (uint8_t*) vv; 
//we'll use ARM NEON intrinsics. Since usually this will be a multiple of 32, we'll use vandq_u16 unrolled by 4 
  int Niter = N/32; 

  if (Niter) 
  {
    for (i = 0; i < Niter; i++) 
    {
      uint8x16_t v1 = vld1q_u8(v+i*64);
      uint8x16_t v2 = vld1q_u8(v+i*64+16);
      uint8x16_t v3 = vld1q_u8(v+i*64+32);
      uint8x16_t v4 = vld1q_u8(v+i*64+48);
      v1 = vrev16q_u8(v1);
      v2 = vrev16q_u8(v2);
      v3 = vrev16q_u8(v3);
      v4 = vrev16q_u8(v4);
      vst1q_u8(v+i*64,v1);
      vst1q_u8(v+i*64+16,v2);
      vst1q_u8(v+i*64+32,v3);
      vst1q_u8(v+i*64+48,v4);
    }

    i*=32; // catch any remaining elements with a normal loop
  }
#endif 


  // finish the last (or do all of them, if not using intrinsics) 
  for (; i < N; i++) 
  {
    vv[i] = __builtin_bswap16(vv[i]); 
  }
}


static uint16_t test_in[65536]; 
static uint16_t test_and[65536]; 
static uint16_t test_swap[65536]; 

int main () 
{

  //randomize test_buffer 

  FILE * frandom = fopen("/dev/urandom","rw"); 
  fread(test_in, 2, 4096, frandom); 
  fclose(frandom); 


  memcpy(test_and, test_in, sizeof(test_in));
  memcpy(test_swap, test_in, sizeof(test_in));

  for (int i = 0; i < 9001; i++) 
  {
    andall16(test_and, 0x0fff, 4096); 
    bswap16(test_swap, 4096); 
  }

  for (int i = 0; i < 4096; i++) 
  {
    assert (test_and[i] == (test_in[i] & 0x0fff)); 
//    printf("%04x,%04x\n", test_swap[i], test_in[i]); 
    assert (test_swap[i] == __builtin_bswap16(test_in[i])); 
  }

  return 0; 

}

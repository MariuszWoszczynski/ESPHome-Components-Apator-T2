#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../sx1276.chip.c"

static uint8_t decode_symbol(uint8_t symbol) {
  for (uint8_t i=0;i<16;i++) if (CODE_3OF6[i]==symbol) return i;
  return 0xff;
}

static size_t decode_3of6_test(const uint8_t *coded,size_t coded_length,
                               uint8_t *decoded,size_t capacity) {
  const size_t symbols=coded_length*8/6;
  if (symbols/2>capacity) return 0;
  memset(decoded,0,capacity);
  for (size_t i=0;i<symbols;i++) {
    const size_t bit=i*6;
    const size_t byte=bit/8;
    const size_t shift=bit%8;
    uint16_t pair=(uint16_t)coded[byte]<<8;
    if (byte+1<coded_length) pair|=coded[byte+1];
    const uint8_t symbol=(uint8_t)((pair>>(10-shift))&0x3f);
    const uint8_t nibble=decode_symbol(symbol);
    if (nibble==0xff) return 0;
    if ((i&1)==0) decoded[i/2]=(uint8_t)(nibble<<4);
    else decoded[i/2]|=nibble;
  }
  return symbols/2;
}

int main(void) {
  uint8_t clear[16]={0x2f,0x2f,0x0f,0x00,0xff,0xff,0x00,0x00,
                     0x00,0x00,0xb0,0x06,0x06,0x06,0x06,0x06};
  const uint8_t iv[16]={0x01,0x06,0x05,0x82,0x20,0x07,0x05,0x07,
                        3,3,3,3,3,3,3,3};
  const uint8_t expected[16]={0xe8,0x3c,0xbe,0x96,0x0d,0xf7,0x20,0x88,
                              0xa3,0xf4,0xce,0xa0,0x00,0xca,0x23,0xb4};
  for (size_t i=0;i<16;i++) clear[i]^=iv[i];
  aes_encrypt_zero_key(clear);
  assert(memcmp(clear,expected,16)==0);

  const uint8_t logical[10]={9,0x44,0x01,0x06,0x05,0x82,0x20,0x07,5,7};
  uint8_t physical[32],coded[64],decoded[32];
  const size_t physical_length=add_format_a_crc(logical,sizeof(logical),physical,sizeof(physical));
  assert(physical_length==12);
  const size_t coded_length=encode_3of6(physical,physical_length,coded,sizeof(coded));
  assert(coded_length==18);
  const size_t decoded_length=decode_3of6_test(coded,coded_length,decoded,sizeof(decoded));
  assert(decoded_length==physical_length);
  assert(memcmp(decoded,physical,physical_length)==0);
  assert(crc16_en13757(decoded,10)==((uint16_t)decoded[10]<<8|decoded[11]));
  puts("SX1276_MODEL_TEST_PASS");
  return 0;
}

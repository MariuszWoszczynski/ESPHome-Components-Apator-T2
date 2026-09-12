#include "wokwi-api.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_RSSI_VALUE 0x11
#define REG_IRQ_FLAGS_1 0x3e
#define REG_IRQ_FLAGS_2 0x3f
#define REG_DIO_MAPPING_1 0x40
#define REG_VERSION 0x42

#define MODE_STANDBY 0x01
#define MODE_TX 0x03
#define MODE_RX 0x05

#define IRQ1_MODE_READY 0x80
#define IRQ1_RX_READY 0x40
#define IRQ1_TX_READY 0x20
#define IRQ2_FIFO_FULL 0x80
#define IRQ2_FIFO_EMPTY 0x40
#define IRQ2_FIFO_LEVEL 0x20
#define IRQ2_FIFO_OVERRUN 0x10
#define IRQ2_PACKET_SENT 0x08
#define IRQ2_PAYLOAD_READY 0x04

#define RX_CAPACITY 256
#define TX_CAPACITY 256

typedef struct {
  pin_t nss;
  pin_t reset;
  pin_t dio1;
  spi_dev_t spi;
  timer_t uplink_timer;
  timer_t tx_timer;
  timer_t response_timer;

  uint32_t attr_meter_id;
  uint32_t attr_version;
  uint32_t attr_device_type;
  uint32_t attr_period_seconds;
  uint32_t attr_uplink_interval_ms;
  uint32_t attr_response_mode;

  uint8_t regs[128];
  uint8_t spi_byte;
  uint8_t spi_address;
  bool spi_has_address;
  bool spi_write;

  uint8_t rx_fifo[RX_CAPACITY];
  size_t rx_length;
  size_t rx_offset;
  uint8_t tx_fifo[TX_CAPACITY];
  size_t tx_length;
  uint32_t transmission_count;
  bool response_pending;
} chip_state_t;

static const uint8_t CODE_3OF6[16] = {
    0x16, 0x0d, 0x0e, 0x0b, 0x1c, 0x19, 0x1a, 0x13,
    0x2c, 0x25, 0x26, 0x23, 0x34, 0x31, 0x32, 0x29,
};

static uint16_t crc16_en13757(const uint8_t *data, size_t length) {
  uint16_t crc = 0;
  for (size_t i = 0; i < length; i++) {
    uint8_t value = data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (((crc & 0x8000) >> 8) ^ (value & 0x80))
        crc = (uint16_t)((crc << 1) ^ 0x3d65);
      else
        crc <<= 1;
      value <<= 1;
    }
  }
  return (uint16_t)~crc;
}

static size_t add_format_a_crc(const uint8_t *logical, size_t logical_length,
                               uint8_t *physical, size_t capacity) {
  size_t input = 0;
  size_t output = 0;
  while (input < logical_length) {
    size_t block = input == 0 ? 10 : 16;
    if (block > logical_length - input)
      block = logical_length - input;
    if (output + block + 2 > capacity)
      return 0;
    memcpy(physical + output, logical + input, block);
    const uint16_t crc = crc16_en13757(logical + input, block);
    physical[output + block] = (uint8_t)(crc >> 8);
    physical[output + block + 1] = (uint8_t)crc;
    input += block;
    output += block + 2;
  }
  return output;
}

static size_t encode_3of6(const uint8_t *input, size_t length, uint8_t *output,
                          size_t capacity) {
  const size_t encoded_length = (3 * length + 1) / 2;
  if (encoded_length > capacity)
    return 0;
  memset(output, 0, encoded_length);
  size_t output_bit = 0;
  for (size_t i = 0; i < length; i++) {
    const uint8_t nibbles[2] = {(uint8_t)(input[i] >> 4),
                                (uint8_t)(input[i] & 0x0f)};
    for (size_t n = 0; n < 2; n++) {
      const uint8_t code = CODE_3OF6[nibbles[n]];
      for (int bit = 5; bit >= 0; bit--) {
        if (code & (1 << bit))
          output[output_bit / 8] |= (uint8_t)(1 << (7 - output_bit % 8));
        output_bit++;
      }
    }
  }
  return encoded_length;
}

static void meter_id_bcd(uint32_t meter_id, uint8_t output[4]) {
  for (size_t i = 0; i < 4; i++) {
    const uint8_t low = meter_id % 10;
    meter_id /= 10;
    const uint8_t high = meter_id % 10;
    meter_id /= 10;
    output[i] = (uint8_t)((high << 4) | low);
  }
}

/* AES-128 encryption, reduced to the functions needed for one CBC block. */
static const uint8_t SBOX[256] = {
  0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
  0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
  0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
  0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
  0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
  0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
  0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
  0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
  0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
  0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
  0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
  0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
  0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
  0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
  0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
  0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
};
static const uint8_t RCON[11] = {0,1,2,4,8,0x10,0x20,0x40,0x80,0x1b,0x36};

static uint8_t xtime(uint8_t x) { return (uint8_t)((x << 1) ^ ((x >> 7) * 0x1b)); }

static void aes_key_expand_zero(uint8_t round_key[176]) {
  memset(round_key, 0, 16);
  uint8_t temp[4];
  size_t generated = 16;
  uint8_t round = 1;
  while (generated < 176) {
    memcpy(temp, round_key + generated - 4, 4);
    if (generated % 16 == 0) {
      const uint8_t first = temp[0];
      temp[0] = (uint8_t)(SBOX[temp[1]] ^ RCON[round++]);
      temp[1] = SBOX[temp[2]];
      temp[2] = SBOX[temp[3]];
      temp[3] = SBOX[first];
    }
    for (size_t i = 0; i < 4; i++) {
      round_key[generated] = (uint8_t)(round_key[generated - 16] ^ temp[i]);
      generated++;
    }
  }
}

static void aes_add_round_key(uint8_t state[16], const uint8_t *key) {
  for (size_t i = 0; i < 16; i++) state[i] ^= key[i];
}

static void aes_sub_bytes(uint8_t state[16]) {
  for (size_t i = 0; i < 16; i++) state[i] = SBOX[state[i]];
}

static void aes_shift_rows(uint8_t s[16]) {
  uint8_t t;
  t=s[1]; s[1]=s[5]; s[5]=s[9]; s[9]=s[13]; s[13]=t;
  t=s[2]; s[2]=s[10]; s[10]=t; t=s[6]; s[6]=s[14]; s[14]=t;
  t=s[15]; s[15]=s[11]; s[11]=s[7]; s[7]=s[3]; s[3]=t;
}

static void aes_mix_columns(uint8_t s[16]) {
  for (size_t i = 0; i < 4; i++) {
    uint8_t *c = s + 4*i;
    const uint8_t a=c[0], b=c[1], d=c[2], e=c[3];
    const uint8_t all=(uint8_t)(a^b^d^e);
    c[0]^=(uint8_t)(all^xtime((uint8_t)(a^b)));
    c[1]^=(uint8_t)(all^xtime((uint8_t)(b^d)));
    c[2]^=(uint8_t)(all^xtime((uint8_t)(d^e)));
    c[3]^=(uint8_t)(all^xtime((uint8_t)(e^a)));
  }
}

static void aes_encrypt_zero_key(uint8_t block[16]) {
  uint8_t round_key[176];
  aes_key_expand_zero(round_key);
  aes_add_round_key(block, round_key);
  for (size_t round = 1; round < 10; round++) {
    aes_sub_bytes(block); aes_shift_rows(block); aes_mix_columns(block);
    aes_add_round_key(block, round_key + 16*round);
  }
  aes_sub_bytes(block); aes_shift_rows(block);
  aes_add_round_key(block, round_key + 160);
}

static void update_irq_flags(chip_state_t *chip) {
  const uint8_t mode = chip->regs[REG_OP_MODE] & 7;
  uint8_t irq1 = IRQ1_MODE_READY;
  if (mode == MODE_RX) irq1 |= IRQ1_RX_READY;
  if (mode == MODE_TX) irq1 |= IRQ1_TX_READY;
  chip->regs[REG_IRQ_FLAGS_1] = irq1;

  uint8_t irq2 = chip->regs[REG_IRQ_FLAGS_2] & (IRQ2_PACKET_SENT | IRQ2_FIFO_OVERRUN);
  const size_t available = chip->rx_length - chip->rx_offset;
  if (mode == MODE_RX) {
    if (available == 0) irq2 |= IRQ2_FIFO_EMPTY;
    if (available >= 32) irq2 |= IRQ2_FIFO_LEVEL;
    if (available >= RX_CAPACITY) irq2 |= IRQ2_FIFO_FULL;
    if (available > 0) irq2 |= IRQ2_PAYLOAD_READY;
  } else if (mode == MODE_TX) {
    if (chip->tx_length == 0) irq2 |= IRQ2_FIFO_EMPTY;
    // Model immediate FIFO draining. Keep FifoLevel clear so the driver can
    // stream every software chunk into the emulated transmitter.
  } else {
    irq2 |= IRQ2_FIFO_EMPTY;
  }
  chip->regs[REG_IRQ_FLAGS_2] = irq2;
}

static void update_dio1(chip_state_t *chip) {
  update_irq_flags(chip);
  const uint8_t mapping = (chip->regs[REG_DIO_MAPPING_1] >> 4) & 3;
  bool high = false;
  if (mapping == 0)
    high = (chip->regs[REG_IRQ_FLAGS_2] & IRQ2_FIFO_LEVEL) != 0;
  else if (mapping == 1)
    high = (chip->regs[REG_IRQ_FLAGS_2] & IRQ2_FIFO_EMPTY) != 0;
  else if (mapping == 2)
    high = (chip->regs[REG_IRQ_FLAGS_2] & IRQ2_FIFO_FULL) != 0;
  pin_write(chip->dio1, high ? HIGH : LOW);
}

static bool load_logical_frame(chip_state_t *chip, const uint8_t *logical,
                               size_t logical_length, const char *name) {
  if ((chip->regs[REG_OP_MODE] & 7) != MODE_RX || chip->rx_offset < chip->rx_length)
    return false;
  uint8_t physical[160];
  const size_t physical_length = add_format_a_crc(logical, logical_length,
                                                   physical, sizeof(physical));
  const size_t encoded_length = encode_3of6(physical, physical_length,
                                             chip->rx_fifo, sizeof(chip->rx_fifo));
  if (physical_length == 0 || encoded_length == 0)
    return false;
  chip->rx_offset = 0;
  chip->rx_length = encoded_length;
  printf("SX1276: inject %s (%u logical, %u encoded bytes)\n", name,
         (unsigned)logical_length, (unsigned)encoded_length);
  update_dio1(chip);
  return true;
}

static bool inject_uplink(chip_state_t *chip) {
  uint8_t id[4]; meter_id_bcd(attr_read(chip->attr_meter_id), id);
  uint8_t frame[10] = {9, 0x44, 0x01, 0x06, id[0], id[1], id[2], id[3],
                       (uint8_t)attr_read(chip->attr_version),
                       (uint8_t)attr_read(chip->attr_device_type)};
  return load_logical_frame(chip, frame, sizeof(frame), "meter uplink");
}

static bool inject_write_ack(chip_state_t *chip) {
  uint8_t id[4]; meter_id_bcd(attr_read(chip->attr_meter_id), id);
  uint8_t frame[21] = {0};
  frame[0] = sizeof(frame)-1; frame[1] = 0x00; frame[2] = 0x01; frame[3] = 0x06;
  memcpy(frame+4, id, 4);
  const uint32_t response_mode = attr_read(chip->attr_response_mode);
  frame[20] = response_mode == 2 ? 0x32 : 0x02;
  return load_logical_frame(chip, frame, sizeof(frame),
                            response_mode == 2 ? "write rejection" : "write ACK");
}

static bool inject_read_reply(chip_state_t *chip) {
  uint8_t id[4]; meter_id_bcd(attr_read(chip->attr_meter_id), id);
  const uint8_t version = (uint8_t)attr_read(chip->attr_version);
  const uint8_t device = (uint8_t)attr_read(chip->attr_device_type);
  uint32_t seconds = attr_read(chip->attr_period_seconds);
  if (seconds < 10) seconds = 10;
  if (seconds > 2550) seconds = 2550;
  const uint8_t period = (uint8_t)(seconds / 10);
  uint8_t frame[31] = {0};
  frame[0]=sizeof(frame)-1; frame[1]=0x08; frame[2]=0x01; frame[3]=0x06;
  memcpy(frame+4,id,4); frame[8]=version; frame[9]=device; frame[10]=0x7a; frame[11]=3;
  uint8_t clear[16] = {0x2f,0x2f,0x0f,0x00,0xff,0xff,0x00,0x00,
                       0x00,0x00,0xb0,period,period,period,period,period};
  const uint8_t iv[16] = {0x01,0x06,id[0],id[1],id[2],id[3],version,device,
                          3,3,3,3,3,3,3,3};
  for (size_t i=0;i<16;i++) clear[i]^=iv[i];
  aes_encrypt_zero_key(clear);
  memcpy(frame+15,clear,16);
  return load_logical_frame(chip, frame, sizeof(frame), "register 0xB0 readback");
}

static void reset_chip(chip_state_t *chip) {
  memset(chip->regs, 0, sizeof(chip->regs));
  chip->regs[REG_OP_MODE]=MODE_STANDBY;
  chip->regs[REG_VERSION]=0x12;
  chip->regs[REG_RSSI_VALUE]=150;
  chip->rx_length=chip->rx_offset=chip->tx_length=0;
  chip->transmission_count=0;
  chip->response_pending=false;
  chip->spi_has_address=false;
  chip->spi_write=false;
  chip->spi_address=0;
  chip->spi_byte=0;
  update_dio1(chip);
}

static uint8_t read_register(chip_state_t *chip, uint8_t address) {
  address &= 0x7f;
  if (address == REG_FIFO) {
    if (chip->rx_offset < chip->rx_length) {
      const uint8_t value=chip->rx_fifo[chip->rx_offset++];
      return value;
    }
    return 0;
  }
  if (address == REG_IRQ_FLAGS_1 || address == REG_IRQ_FLAGS_2)
    update_irq_flags(chip);
  return chip->regs[address];
}

static void tx_complete(void *user_data) {
  chip_state_t *chip=(chip_state_t *)user_data;
  chip->regs[REG_IRQ_FLAGS_2]|=IRQ2_PACKET_SENT;
  printf("SX1276: TX #%u captured (%u bytes): ",
         (unsigned)(chip->transmission_count+1), (unsigned)chip->tx_length);
  for (size_t i=0;i<chip->tx_length;i++) printf("%02x",chip->tx_fifo[i]);
  printf("\n");
  chip->transmission_count++;
  chip->response_pending=true;
  update_dio1(chip);
  timer_start(chip->response_timer, 10000, false);
}

static void response_due(void *user_data) {
  chip_state_t *chip=(chip_state_t *)user_data;
  if (!chip->response_pending) return;
  if ((chip->regs[REG_OP_MODE]&7)!=MODE_RX || chip->rx_length!=0) {
    timer_start(chip->response_timer, 1000, false);
    return;
  }
  const uint32_t mode=attr_read(chip->attr_response_mode);
  if (mode==1) {
    printf("SX1276: response suppressed (timeout mode)\n");
    chip->response_pending=false;
    return;
  }
  const bool odd=(chip->transmission_count&1)!=0;
  const bool sent=odd ? inject_write_ack(chip) : inject_read_reply(chip);
  if (sent) chip->response_pending=false;
  else timer_start(chip->response_timer,1000,false);
}

static void uplink_due(void *user_data) {
  chip_state_t *chip=(chip_state_t *)user_data;
  if (!chip->response_pending) inject_uplink(chip);
}

static void write_register(chip_state_t *chip, uint8_t address, uint8_t value) {
  address &= 0x7f;
  if (address == REG_FIFO) {
    if (chip->tx_length < sizeof(chip->tx_fifo)) chip->tx_fifo[chip->tx_length++]=value;
    update_dio1(chip);
    return;
  }
  if (address == REG_VERSION) return;
  if (address == REG_IRQ_FLAGS_2) {
    if (value&IRQ2_FIFO_OVERRUN) chip->regs[address]&=(uint8_t)~IRQ2_FIFO_OVERRUN;
    return;
  }
  const uint8_t old_mode=chip->regs[REG_OP_MODE]&7;
  chip->regs[address]=value;
  if (address == REG_OP_MODE) {
    const uint8_t new_mode=value&7;
    if (new_mode==MODE_STANDBY && old_mode!=MODE_STANDBY) chip->tx_length=0;
    if (new_mode==MODE_TX && old_mode!=MODE_TX) {
      chip->regs[REG_IRQ_FLAGS_2]&=(uint8_t)~IRQ2_PACKET_SENT;
      timer_start(chip->tx_timer,20000,false);
    }
  }
  update_dio1(chip);
}

static void schedule_spi_byte(chip_state_t *chip) {
  if (pin_read(chip->nss)!=LOW) return;
  if (!chip->spi_has_address || chip->spi_write)
    chip->spi_byte=0;
  else
    chip->spi_byte=read_register(chip,chip->spi_address);
  spi_start(chip->spi,&chip->spi_byte,1);
}

static void spi_done(void *user_data, uint8_t *buffer, uint32_t count) {
  chip_state_t *chip=(chip_state_t *)user_data;
  if (count==0 || pin_read(chip->nss)!=LOW) return;
  const uint8_t received=buffer[0];
  if (!chip->spi_has_address) {
    chip->spi_write=(received&0x80)!=0;
    chip->spi_address=received&0x7f;
    chip->spi_has_address=true;
  } else if (chip->spi_write) {
    const uint8_t current=chip->spi_address;
    write_register(chip,current,received);
    if (current!=REG_FIFO) chip->spi_address=(uint8_t)((current+1)&0x7f);
  } else if (chip->spi_address!=REG_FIFO) {
    chip->spi_address=(uint8_t)((chip->spi_address+1)&0x7f);
  } else if (chip->rx_length != 0 && chip->rx_offset >= chip->rx_length) {
    // A FIFO byte is prefetched for the next SPI exchange. Raise FifoEmpty
    // only after that last prefetched byte has actually crossed the bus.
    chip->rx_offset=chip->rx_length=0;
    update_dio1(chip);
  }
  schedule_spi_byte(chip);
}

static void pin_changed(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *chip=(chip_state_t *)user_data;
  if (pin==chip->reset && value==LOW) {
    reset_chip(chip);
    printf("SX1276: reset\n");
    return;
  }
  if (pin==chip->nss) {
    if (value==LOW) {
      chip->spi_has_address=false;
      chip->spi_write=false;
      chip->spi_address=0;
      chip->spi_byte=0;
      spi_start(chip->spi,&chip->spi_byte,1);
    } else {
      spi_stop(chip->spi);
      chip->spi_has_address=false;
    }
  }
}

void chip_init(void) {
  setvbuf(stdout,NULL,_IOLBF,1024);
  chip_state_t *chip=(chip_state_t *)calloc(1,sizeof(chip_state_t));
  pin_init("VCC",INPUT); pin_init("GND",INPUT);
  chip->nss=pin_init("NSS",INPUT_PULLUP);
  chip->reset=pin_init("RESET",INPUT_PULLUP);
  chip->dio1=pin_init("DIO1",OUTPUT_LOW);

  chip->attr_meter_id=attr_init("meterId",7208205);
  chip->attr_version=attr_init("version",5);
  chip->attr_device_type=attr_init("deviceType",7);
  chip->attr_period_seconds=attr_init("periodSeconds",60);
  chip->attr_uplink_interval_ms=attr_init("uplinkIntervalMs",3000);
  chip->attr_response_mode=attr_init("responseMode",0);

  const spi_config_t spi_config={.user_data=chip,
    .sck=pin_init("SCK",INPUT),.mosi=pin_init("MOSI",INPUT),
    .miso=pin_init("MISO",INPUT),.mode=0,.done=spi_done};
  chip->spi=spi_init(&spi_config);
  const pin_watch_config_t watch={.user_data=chip,.edge=BOTH,.pin_change=pin_changed};
  pin_watch(chip->nss,&watch); pin_watch(chip->reset,&watch);

  const timer_config_t uplink_config={.callback=uplink_due,.user_data=chip};
  const timer_config_t tx_config={.callback=tx_complete,.user_data=chip};
  const timer_config_t response_config={.callback=response_due,.user_data=chip};
  chip->uplink_timer=timer_init(&uplink_config);
  chip->tx_timer=timer_init(&tx_config);
  chip->response_timer=timer_init(&response_config);
  reset_chip(chip);
  uint32_t interval=attr_read(chip->attr_uplink_interval_ms);
  if (interval<100) interval=100;
  timer_start(chip->uplink_timer,interval*1000,true);
  printf("SX1276: wM-Bus T2 model ready; DIO1=FIFO flags, RegVersion=0x12\n");
}

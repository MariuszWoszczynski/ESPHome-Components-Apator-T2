#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

typedef uint32_t pin_t;
typedef void *spi_dev_t;
typedef void *timer_t;

enum { LOW=0, HIGH=1, INPUT=0, INPUT_PULLUP=1, OUTPUT_LOW=2, BOTH=3 };
typedef struct { void *user_data; pin_t sck, mosi, miso; uint32_t mode; void (*done)(void *,uint8_t *,uint32_t); } spi_config_t;
typedef struct { void *user_data; uint32_t edge; void (*pin_change)(void *,pin_t,uint32_t); } pin_watch_config_t;
typedef struct { void (*callback)(void *); void *user_data; } timer_config_t;

static inline pin_t pin_init(const char *name, uint32_t mode) {(void)name;(void)mode;return 1;}
static inline uint32_t pin_read(pin_t pin) {(void)pin;return HIGH;}
static inline void pin_write(pin_t pin,uint32_t value) {(void)pin;(void)value;}
static inline void pin_watch(pin_t pin,const pin_watch_config_t *cfg) {(void)pin;(void)cfg;}
static inline spi_dev_t spi_init(const spi_config_t *cfg) {(void)cfg;return 0;}
static inline void spi_start(spi_dev_t spi,uint8_t *buffer,uint32_t count) {(void)spi;(void)buffer;(void)count;}
static inline void spi_stop(spi_dev_t spi) {(void)spi;}
static inline timer_t timer_init(const timer_config_t *cfg) {(void)cfg;return (timer_t)0;}
static inline void timer_start(timer_t timer,uint32_t micros,bool repeat) {(void)timer;(void)micros;(void)repeat;}
static inline uint32_t attr_init(const char *name,uint32_t value) {(void)name;return value;}
static inline uint32_t attr_read(uint32_t value) {return value;}

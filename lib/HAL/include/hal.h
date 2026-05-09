#pragma once
#include <stdint.h>
#include <stdarg.h>

// ----- Lifecycle -----
void     hal_init();

// ----- Timing -----
void     hal_delay_ms(uint32_t ms);
void     hal_delay_us(uint32_t us);
uint32_t hal_micros();          // microsecond counter, wraps ~71 min

// ----- I2C -----
// All addresses are 7-bit. Each HAL impl shifts as required by its framework.
bool     hal_i2c_write(uint8_t addr, uint8_t reg, uint8_t data);
bool     hal_i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);

// ----- Logging -----
void     hal_log(const char *fmt, ...);

// ----- Status LEDs -----
void     hal_led1_toggle();
void     hal_led2_toggle();

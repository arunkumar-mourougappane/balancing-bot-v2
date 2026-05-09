#ifdef HAL_MBED
#include "mbed.h"
#include "hal.h"
#include <cstdio>
#include <cstdarg>

// ---- I2C ----
// LPC1768  (Adafruit #834): I2C_SDA = p28, I2C_SCL = p27
// FRDM-K64F:                I2C_SDA = PTE25, I2C_SCL = PTE24
static I2C s_i2c(I2C_SDA, I2C_SCL);

// ---- Timer ----
static Timer s_timer;

// ---- LEDs ----
// LPC1768:  LED1 (p1.18) and LED2 (p1.20) are active-high.
// FRDM-K64F: LED1 (PTB22, red) and LED_BLUE (PTB21) are active-low.
#if defined(BOARD_LPC1768)
static DigitalOut s_led1(LED1);
static DigitalOut s_led2(LED2);
static const bool LED_ACTIVE_LOW = false;
#elif defined(BOARD_FRDM_K64F)
static DigitalOut s_led1(LED1);
static DigitalOut s_led2(LED_BLUE);
static const bool LED_ACTIVE_LOW = true;
#else
#error "Unsupported mbed board. Define BOARD_LPC1768 or BOARD_FRDM_K64F."
#endif

// ---- Lifecycle ----

void hal_init() {
    s_i2c.frequency(400000);
    s_timer.start();
    // Initial LED state: off
    s_led1 = LED_ACTIVE_LOW ? 1 : 0;
    s_led2 = LED_ACTIVE_LOW ? 1 : 0;
}

// ---- Timing ----

void hal_delay_ms(uint32_t ms) {
    wait_us((uint32_t)(ms) * 1000u);
}

void hal_delay_us(uint32_t us) {
    wait_us(us);
}

uint32_t hal_micros() {
    return (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(
               s_timer.elapsed_time()).count();
}

// ---- I2C ----
// Addresses passed in are 7-bit; mbed requires 8-bit (left-shifted by 1).

bool hal_i2c_write(uint8_t addr, uint8_t reg, uint8_t data) {
    char buf[2] = { (char)reg, (char)data };
    return s_i2c.write((int)(addr << 1), buf, 2, false) == 0;
}

bool hal_i2c_read(uint8_t addr, uint8_t reg, uint8_t *out, uint8_t len) {
    char w = (char)reg;
    if (s_i2c.write((int)(addr << 1), &w, 1, true) != 0) return false;
    return s_i2c.read((int)(addr << 1), (char *)out, (int)len, false) == 0;
}

// ---- Logging ----
// printf is redirected to USB serial by mbed runtime.
// Baud rate is set to 38400 via mbed_app.json (platform.stdio-baud-rate).

void hal_log(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

// ---- Status LEDs ----

void hal_led1_toggle() {
    s_led1 = !s_led1;
}

void hal_led2_toggle() {
    s_led2 = !s_led2;
}

#endif // HAL_MBED

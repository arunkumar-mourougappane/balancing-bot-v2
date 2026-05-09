#ifdef HAL_ARDUINO
#include <Arduino.h>
#include <Wire.h>
#include "hal.h"
#include <cstdarg>
#include <cstdio>

// ---- Board pin map ----
// Adafruit ESP32-S3 TFT Feather:
//   I2C : SDA = GPIO 3, SCL = GPIO 4 (Feather header + STEMMA QT)
//   LED : GPIO 13 (connect external LED; onboard NeoPixel needs separate lib)
#if defined(BOARD_ESP32S3_TFT)
static const int PIN_SDA  = 3;
static const int PIN_SCL  = 4;
static const int PIN_LED1 = 13;
static const int PIN_LED2 = 13;    // same pin; extend if a second LED is wired
#else
#error "Unsupported Arduino board. Define BOARD_ESP32S3_TFT."
#endif

static bool s_led1_state = false;
static bool s_led2_state = false;

// ---- Lifecycle ----

void hal_init() {
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    Serial.begin(38400);
    // Wait up to 3 s for USB CDC to enumerate; non-blocking on hardware UART.
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 3000) {}

    pinMode(PIN_LED1, OUTPUT);
    digitalWrite(PIN_LED1, LOW);
    if (PIN_LED2 != PIN_LED1) {
        pinMode(PIN_LED2, OUTPUT);
        digitalWrite(PIN_LED2, LOW);
    }
}

// ---- Timing ----

void hal_delay_ms(uint32_t ms) { delay(ms); }
void hal_delay_us(uint32_t us) { delayMicroseconds(us); }
uint32_t hal_micros()          { return (uint32_t)micros(); }

// ---- I2C ----
// Addresses are 7-bit; Arduino Wire uses 7-bit natively.

bool hal_i2c_write(uint8_t addr, uint8_t reg, uint8_t data) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data);
    return Wire.endTransmission() == 0;
}

bool hal_i2c_read(uint8_t addr, uint8_t reg, uint8_t *out, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)addr, (uint8_t)len, (uint8_t) true);
    for (uint8_t i = 0; i < len && Wire.available(); ++i) {
        out[i] = (uint8_t)Wire.read();
    }
    return true;
}

// ---- Logging ----

void hal_log(const char *fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
}

// ---- Status LEDs ----

void hal_led1_toggle() {
    s_led1_state = !s_led1_state;
    digitalWrite(PIN_LED1, s_led1_state ? HIGH : LOW);
}

void hal_led2_toggle() {
    s_led2_state = !s_led2_state;
    digitalWrite(PIN_LED2, s_led2_state ? HIGH : LOW);
}

#endif // HAL_ARDUINO

// Balancing-bot sensor fusion entry point.
// Reads MPU-9250 9-DOF IMU, fuses with Mahony filter, prints gyro + YPR.
//
// Targets:
//   lpc1768    — mbed, Adafruit LPC1768 (#834), I2C p28/p27, LEDs active-high
//   frdm_k64f  — mbed, NXP FRDM-K64F, I2C PTE25/PTE24, LEDs active-low
//   esp32s3_tft— Arduino, Adafruit ESP32-S3 TFT Feather, I2C GPIO3/4
//
// Magnetic declination (yaw correction):
//   Update MAG_DECLINATION_DEG for your location.
//   Lookup: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml

#ifdef HAL_ARDUINO
#include <Arduino.h>
#endif

#include "hal.h"
#include "MPU9250.h"
#include "Mahony.h"

static const float MAG_DECLINATION_DEG = -13.8f;   // Danville CA, 2014 — update for your site
static const float kDegToRad           = 3.14159265f / 180.0f;

static MPU9250 imu;
static Mahony  ahrs(10.0f, 0.0f);  // Kp=10, Ki=0

static uint32_t lastUs = 0;

// Returns dt in seconds since last call, clamped to [0, 0.5].
static float computeDt() {
    uint32_t now = hal_micros();
    uint32_t elapsed = (now >= lastUs) ? (now - lastUs) : (0xFFFFFFFFu - lastUs + now + 1u);
    lastUs = now;
    float dt = (float)elapsed * 1e-6f;
    if (dt < 0.0f || dt > 0.5f) dt = 0.005f;   // clamp on first call / timer wrap
    return dt;
}

static void app_init() {
    hal_init();
    hal_led1_toggle();

    if (!imu.begin()) {
        hal_log("ERROR: MPU9250 not found — halting.\r\n");
        while (1) { hal_delay_ms(500); hal_led1_toggle(); }
    }

    // Optional: override mag bias for your environment [mG]
    // imu.setMagBias(470.0f, 120.0f, 125.0f);

    lastUs = hal_micros();
    hal_log("MPU9250 ready. Streaming gyro [deg/s] + YPR [deg].\r\n");
    hal_led2_toggle();
}

static void app_loop() {
    float dt = computeDt();

    bool newData = imu.readSensors();

    if (newData) {
        // Update filter with gyro converted to rad/s
        ahrs.update(imu.ax, imu.ay, imu.az,
                    imu.gx * kDegToRad,
                    imu.gy * kDegToRad,
                    imu.gz * kDegToRad,
                    imu.my, imu.mx, imu.mz,  // NED: swap X/Y for Mahony convention
                    dt);

        float yaw   = ahrs.getYaw() - MAG_DECLINATION_DEG;
        float pitch = ahrs.getPitch();
        float roll  = ahrs.getRoll();

        // Output: gx gy gz [deg/s]  yaw pitch roll [deg]
        hal_log("%.3f %.3f %.3f  %.2f %.2f %.2f\r\n",
                imu.gx, imu.gy, imu.gz,
                yaw, pitch, roll);

        hal_led1_toggle();
    }
}

// ---- Framework entry points ----

#ifdef HAL_ARDUINO
void setup() { app_init(); }
void loop()  { app_loop(); }
#else
int main() {
    app_init();
    while (1) {
        app_loop();
    }
}
#endif

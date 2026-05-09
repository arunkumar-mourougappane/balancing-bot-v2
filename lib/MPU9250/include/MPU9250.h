#pragma once
#include <stdint.h>
#include "MPU9250_registers.h"

class MPU9250 {
public:
    // Calibrated sensor readings, updated by readSensors()
    float ax, ay, az;       // acceleration [g]
    float gx, gy, gz;       // angular rate [deg/s]
    float mx, my, mz;       // magnetic field [mG]
    float temperature;      // [°C]

    MPU9250(uint8_t ascale  = AFS_2G,
            uint8_t gscale  = GFS_250DPS,
            uint8_t mscale  = MFS_16BITS,
            uint8_t mmode   = 0x06);        // 0x02 = 8 Hz, 0x06 = 100 Hz mag ODR

    // One-shot startup: reset → self-test → calibrate → init IMU → init mag.
    // Returns false if WHO_AM_I check fails.
    bool begin();

    // Read all sensors. Returns true when new accel/gyro data was ready.
    bool readSensors();

    // Override the hard-coded environmental mag bias [mG].
    void setMagBias(float x, float y, float z);

    // Access raw calibration results from begin().
    const float* gyroBias()  const { return _gyroBias; }
    const float* accelBias() const { return _accelBias; }

private:
    uint8_t _Ascale, _Gscale, _Mscale, _Mmode;
    float   _aRes, _gRes, _mRes;
    float   _gyroBias[3];
    float   _accelBias[3];
    float   _magCalibration[3];  // factory sensitivity from Fuse ROM
    float   _magBias[3];         // user environmental bias

    // I2C helpers (thin wrappers around hal_i2c_*)
    uint8_t readByte(uint8_t addr, uint8_t reg);
    void    writeByte(uint8_t addr, uint8_t reg, uint8_t data);
    bool    readBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *dest);

    // Sensor reads
    void    readAccelData(int16_t *dest);
    void    readGyroData(int16_t *dest);
    bool    readMagData(int16_t *dest);
    int16_t readTempData();

    // Init helpers
    void reset();
    void initMPU9250();
    void initAK8963();
    void calibrate();
    void selfTest(float *results);
    void computeAres();
    void computeGres();
    void computeMres();
};

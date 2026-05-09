#include "MPU9250.h"
#include "MPU9250_registers.h"
#include "hal.h"
#include <math.h>
#include <string.h>

// ---- Constructor ----

MPU9250::MPU9250(uint8_t ascale, uint8_t gscale, uint8_t mscale, uint8_t mmode)
    : ax(0), ay(0), az(0),
      gx(0), gy(0), gz(0),
      mx(0), my(0), mz(0),
      temperature(0),
      _Ascale(ascale), _Gscale(gscale), _Mscale(mscale), _Mmode(mmode),
      _aRes(0), _gRes(0), _mRes(0)
{
    memset(_gyroBias,      0, sizeof(_gyroBias));
    memset(_accelBias,     0, sizeof(_accelBias));
    memset(_magCalibration,0, sizeof(_magCalibration));
    // Default environmental mag bias (Danville CA, 2014-04-04).
    // Override with setMagBias() for your location.
    _magBias[0] = 470.0f;
    _magBias[1] = 120.0f;
    _magBias[2] = 125.0f;
}

// ---- Public API ----

bool MPU9250::begin() {
    uint8_t who = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    if (who != 0x71) {
        hal_log("MPU9250 WHO_AM_I = 0x%02X (expected 0x71)\r\n", who);
        return false;
    }

    reset();
    float st[6];
    selfTest(st);
    hal_log("Self-test: ax=%+.1f ay=%+.1f az=%+.1f gx=%+.1f gy=%+.1f gz=%+.1f %%\r\n",
            st[0], st[1], st[2], st[3], st[4], st[5]);

    calibrate();
    hal_delay_ms(2000);

    initMPU9250();
    initAK8963();
    hal_delay_ms(100);

    computeAres();
    computeGres();
    computeMres();
    return true;
}

bool MPU9250::readSensors() {
    bool newData = (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) != 0;

    if (newData) {
        int16_t accelRaw[3], gyroRaw[3];
        readAccelData(accelRaw);
        readGyroData(gyroRaw);

        ax = (float)accelRaw[0] * _aRes - _accelBias[0];
        ay = (float)accelRaw[1] * _aRes - _accelBias[1];
        az = (float)accelRaw[2] * _aRes - _accelBias[2];

        gx = (float)gyroRaw[0] * _gRes - _gyroBias[0];
        gy = (float)gyroRaw[1] * _gRes - _gyroBias[1];
        gz = (float)gyroRaw[2] * _gRes - _gyroBias[2];
    }

    int16_t magRaw[3];
    if (readMagData(magRaw)) {
        mx = (float)magRaw[0] * _mRes * _magCalibration[0] - _magBias[0];
        my = (float)magRaw[1] * _mRes * _magCalibration[1] - _magBias[1];
        mz = (float)magRaw[2] * _mRes * _magCalibration[2] - _magBias[2];
    }

    int16_t tempRaw = readTempData();
    temperature = (float)tempRaw / 333.87f + 21.0f;

    return newData;
}

void MPU9250::setMagBias(float x, float y, float z) {
    _magBias[0] = x;
    _magBias[1] = y;
    _magBias[2] = z;
}

// ---- I2C helpers ----

uint8_t MPU9250::readByte(uint8_t addr, uint8_t reg) {
    uint8_t val = 0;
    hal_i2c_read(addr, reg, &val, 1);
    return val;
}

void MPU9250::writeByte(uint8_t addr, uint8_t reg, uint8_t data) {
    hal_i2c_write(addr, reg, data);
}

bool MPU9250::readBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *dest) {
    return hal_i2c_read(addr, reg, dest, count);
}

// ---- Sensor reads ----

void MPU9250::readAccelData(int16_t *dest) {
    uint8_t raw[6];
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, raw);
    dest[0] = (int16_t)(((int16_t)raw[0] << 8) | raw[1]);
    dest[1] = (int16_t)(((int16_t)raw[2] << 8) | raw[3]);
    dest[2] = (int16_t)(((int16_t)raw[4] << 8) | raw[5]);
}

void MPU9250::readGyroData(int16_t *dest) {
    uint8_t raw[6];
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, raw);
    dest[0] = (int16_t)(((int16_t)raw[0] << 8) | raw[1]);
    dest[1] = (int16_t)(((int16_t)raw[2] << 8) | raw[3]);
    dest[2] = (int16_t)(((int16_t)raw[4] << 8) | raw[5]);
}

bool MPU9250::readMagData(int16_t *dest) {
    if (!(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)) return false;
    uint8_t raw[7];
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, raw);
    if (raw[6] & 0x08) return false;   // magnetic overflow
    dest[0] = (int16_t)(((int16_t)raw[1] << 8) | raw[0]);  // little-endian
    dest[1] = (int16_t)(((int16_t)raw[3] << 8) | raw[2]);
    dest[2] = (int16_t)(((int16_t)raw[5] << 8) | raw[4]);
    return true;
}

int16_t MPU9250::readTempData() {
    uint8_t raw[2];
    readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, raw);
    return (int16_t)(((int16_t)raw[0] << 8) | raw[1]);
}

// ---- Init helpers ----

void MPU9250::reset() {
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
    hal_delay_ms(100);
}

void MPU9250::initMPU9250() {
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // wake up
    hal_delay_ms(100);
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);   // PLL from x-gyro

    writeByte(MPU9250_ADDRESS, CONFIG, 0x03);        // DLPF 41 Hz, 1 kHz gyro rate
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);   // 200 Hz output rate

    uint8_t c;
    c = readByte(MPU9250_ADDRESS, GYRO_CONFIG);
    c &= ~0xE0;
    c &= ~0x18;
    c |= (uint8_t)(_Gscale << 3);
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c);

    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
    c &= ~0xE0;
    c &= ~0x18;
    c |= (uint8_t)(_Ascale << 3);
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c);

    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
    c &= ~0x0F;
    c |= 0x03;                                      // 41 Hz accel bandwidth
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c);

    writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);  // active-high, latched, I2C bypass
    writeByte(MPU9250_ADDRESS, INT_ENABLE,  0x01);  // data-ready interrupt
}

void MPU9250::initAK8963() {
    uint8_t raw[3];
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);  // power down
    hal_delay_ms(10);
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F);  // Fuse ROM access
    hal_delay_ms(10);
    readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, raw);
    _magCalibration[0] = (float)(raw[0] - 128) / 256.0f + 1.0f;
    _magCalibration[1] = (float)(raw[1] - 128) / 256.0f + 1.0f;
    _magCalibration[2] = (float)(raw[2] - 128) / 256.0f + 1.0f;
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);  // power down
    hal_delay_ms(10);
    // 16-bit continuous mode at selected ODR
    writeByte(AK8963_ADDRESS, AK8963_CNTL, (uint8_t)(_Mscale << 4) | _Mmode);
    hal_delay_ms(10);
}

void MPU9250::calibrate() {
    uint8_t data[12];
    uint16_t ii, packetCount, fifoCount;
    int32_t gBias[3] = {0, 0, 0};
    int32_t aBias[3] = {0, 0, 0};

    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
    hal_delay_ms(100);

    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
    hal_delay_ms(200);

    writeByte(MPU9250_ADDRESS, INT_ENABLE,   0x00);
    writeByte(MPU9250_ADDRESS, FIFO_EN,      0x00);
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1,   0x00);
    writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00);
    writeByte(MPU9250_ADDRESS, USER_CTRL,    0x00);
    writeByte(MPU9250_ADDRESS, USER_CTRL,    0x0C);
    hal_delay_ms(15);

    writeByte(MPU9250_ADDRESS, CONFIG,      0x01);   // 188 Hz LPF
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV,  0x00);   // 1 kHz
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);   // 250 dps full scale
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG,0x00);   // 2 g full scale

    static const uint16_t GYRO_SENS  = 131;
    static const uint16_t ACCEL_SENS = 16384;

    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);    // enable FIFO
    writeByte(MPU9250_ADDRESS, FIFO_EN,   0x78);    // gyro + accel
    hal_delay_ms(40);

    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);
    readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, data);
    fifoCount   = ((uint16_t)data[0] << 8) | data[1];
    packetCount = fifoCount / 12;

    for (ii = 0; ii < packetCount; ii++) {
        int16_t aTemp[3], gTemp[3];
        readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, data);
        aTemp[0] = (int16_t)(((int16_t)data[0]  << 8) | data[1]);
        aTemp[1] = (int16_t)(((int16_t)data[2]  << 8) | data[3]);
        aTemp[2] = (int16_t)(((int16_t)data[4]  << 8) | data[5]);
        gTemp[0] = (int16_t)(((int16_t)data[6]  << 8) | data[7]);
        gTemp[1] = (int16_t)(((int16_t)data[8]  << 8) | data[9]);
        gTemp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);
        aBias[0] += aTemp[0]; aBias[1] += aTemp[1]; aBias[2] += aTemp[2];
        gBias[0] += gTemp[0]; gBias[1] += gTemp[1]; gBias[2] += gTemp[2];
    }

    for (int i = 0; i < 3; i++) {
        aBias[i] /= (int32_t)packetCount;
        gBias[i] /= (int32_t)packetCount;
    }
    // Remove gravity from z-axis
    if (aBias[2] > 0) aBias[2] -= (int32_t)ACCEL_SENS;
    else              aBias[2] += (int32_t)ACCEL_SENS;

    // Store software biases (hardware registers left unchanged for portability)
    _gyroBias[0]  = (float)gBias[0] / (float)GYRO_SENS;
    _gyroBias[1]  = (float)gBias[1] / (float)GYRO_SENS;
    _gyroBias[2]  = (float)gBias[2] / (float)GYRO_SENS;
    _accelBias[0] = (float)aBias[0] / (float)ACCEL_SENS;
    _accelBias[1] = (float)aBias[1] / (float)ACCEL_SENS;
    _accelBias[2] = (float)aBias[2] / (float)ACCEL_SENS;
}

void MPU9250::selfTest(float *results) {
    uint8_t raw[6] = {0};
    uint8_t stCode[6];
    int16_t aAvg[3], gAvg[3], aSTAvg[3], gSTAvg[3];
    memset(aAvg, 0, sizeof(aAvg));
    memset(gAvg, 0, sizeof(gAvg));
    memset(aSTAvg, 0, sizeof(aSTAvg));
    memset(gSTAvg, 0, sizeof(gSTAvg));

    writeByte(MPU9250_ADDRESS, SMPLRT_DIV,    0x00);
    writeByte(MPU9250_ADDRESS, CONFIG,         0x02);
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG,    0x01);
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2,  0x02);
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG,   0x01);

    for (int ii = 0; ii < 200; ii++) {
        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, raw);
        aAvg[0] += (int16_t)(((int16_t)raw[0] << 8) | raw[1]);
        aAvg[1] += (int16_t)(((int16_t)raw[2] << 8) | raw[3]);
        aAvg[2] += (int16_t)(((int16_t)raw[4] << 8) | raw[5]);
        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, raw);
        gAvg[0] += (int16_t)(((int16_t)raw[0] << 8) | raw[1]);
        gAvg[1] += (int16_t)(((int16_t)raw[2] << 8) | raw[3]);
        gAvg[2] += (int16_t)(((int16_t)raw[4] << 8) | raw[5]);
    }
    for (int i = 0; i < 3; i++) { aAvg[i] /= 200; gAvg[i] /= 200; }

    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0);
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0);
    hal_delay_us(25000);

    for (int ii = 0; ii < 200; ii++) {
        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, raw);
        aSTAvg[0] += (int16_t)(((int16_t)raw[0] << 8) | raw[1]);
        aSTAvg[1] += (int16_t)(((int16_t)raw[2] << 8) | raw[3]);
        aSTAvg[2] += (int16_t)(((int16_t)raw[4] << 8) | raw[5]);
        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, raw);
        gSTAvg[0] += (int16_t)(((int16_t)raw[0] << 8) | raw[1]);
        gSTAvg[1] += (int16_t)(((int16_t)raw[2] << 8) | raw[3]);
        gSTAvg[2] += (int16_t)(((int16_t)raw[4] << 8) | raw[5]);
    }
    for (int i = 0; i < 3; i++) { aSTAvg[i] /= 200; gSTAvg[i] /= 200; }

    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
    hal_delay_us(25000);

    stCode[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL);
    stCode[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL);
    stCode[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL);
    stCode[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);
    stCode[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);
    stCode[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);

    float factoryTrim[6];
    for (int i = 0; i < 6; i++) {
        factoryTrim[i] = 2620.0f * powf(1.01f, (float)stCode[i] - 1.0f);
    }

    for (int i = 0; i < 3; i++) {
        results[i]   = 100.0f * (float)(aSTAvg[i] - aAvg[i]) / factoryTrim[i];
        results[i+3] = 100.0f * (float)(gSTAvg[i] - gAvg[i]) / factoryTrim[i+3];
    }
}

// ---- Resolution helpers ----

void MPU9250::computeAres() {
    static const float table[] = { 2.0f, 4.0f, 8.0f, 16.0f };
    _aRes = table[_Ascale] / 32768.0f;
}

void MPU9250::computeGres() {
    static const float table[] = { 250.0f, 500.0f, 1000.0f, 2000.0f };
    _gRes = table[_Gscale] / 32768.0f;
}

void MPU9250::computeMres() {
    _mRes = (_Mscale == MFS_14BITS) ? (10.0f * 4219.0f / 8190.0f)
                                    : (10.0f * 4219.0f / 32760.0f);
}

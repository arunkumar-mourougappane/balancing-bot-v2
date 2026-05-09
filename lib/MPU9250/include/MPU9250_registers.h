#pragma once

// 7-bit I2C addresses (ADO pin selects MPU9250 address)
#define MPU9250_ADDRESS     0x68    // ADO = 0
#define MPU9250_ADDRESS_AD1 0x69    // ADO = 1
#define AK8963_ADDRESS      0x0C    // Magnetometer (always fixed)

// AK8963 (magnetometer) registers
#define WHO_AM_I_AK8963  0x00
#define AK8963_ST1       0x02
#define AK8963_XOUT_L    0x03
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09
#define AK8963_CNTL      0x0A
#define AK8963_ASTC      0x0C
#define AK8963_I2CDIS    0x0F
#define AK8963_ASAX      0x10
#define AK8963_ASAY      0x11
#define AK8963_ASAZ      0x12

// MPU-9250 self-test registers
#define SELF_TEST_X_GYRO  0x00
#define SELF_TEST_Y_GYRO  0x01
#define SELF_TEST_Z_GYRO  0x02
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

// MPU-9250 gyro offset trim
#define XG_OFFSET_H 0x13
#define XG_OFFSET_L 0x14
#define YG_OFFSET_H 0x15
#define YG_OFFSET_L 0x16
#define ZG_OFFSET_H 0x17
#define ZG_OFFSET_L 0x18

// MPU-9250 configuration
#define SMPLRT_DIV    0x19
#define CONFIG        0x1A
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C
#define ACCEL_CONFIG2 0x1D
#define LP_ACCEL_ODR  0x1E
#define WOM_THR       0x1F
#define FIFO_EN       0x23

// I2C master / slave
#define I2C_MST_CTRL    0x24
#define I2C_SLV0_ADDR   0x25
#define I2C_SLV0_REG    0x26
#define I2C_SLV0_CTRL   0x27
#define I2C_MST_STATUS  0x36

// Interrupt
#define INT_PIN_CFG 0x37
#define INT_ENABLE  0x38
#define INT_STATUS  0x3A

// Sensor data output
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H   0x41
#define TEMP_OUT_L   0x42
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

// FIFO
#define USER_CTRL   0x6A
#define PWR_MGMT_1  0x6B
#define PWR_MGMT_2  0x6C
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W    0x74

#define WHO_AM_I_MPU9250 0x75   // register; expected value 0x71

// Accelerometer trim
#define XA_OFFSET_H 0x77
#define XA_OFFSET_L 0x78
#define YA_OFFSET_H 0x7A
#define YA_OFFSET_L 0x7B
#define ZA_OFFSET_H 0x7D
#define ZA_OFFSET_L 0x7E

// Accel full-scale options
enum Ascale { AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G };

// Gyro full-scale options
enum Gscale { GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS };

// Magnetometer resolution options
enum Mscale { MFS_14BITS = 0, MFS_16BITS };

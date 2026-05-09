# Program Flow

Sequential description of execution from power-on to steady-state loop.

---

## 1. Framework startup

### mbed (LPC1768 / FRDM-K64F)

The mbed runtime initialises clocks, the NVIC, the heap, and the C++ global constructors before handing control to `main()`. The two global objects (`imu` and `ahrs`) are constructed at this point with their default member values.

### Arduino (ESP32-S3)

The ESP-IDF bootloader and Arduino core run before `setup()` is called. Peripherals including USB CDC are already initialised by the core; `setup()` receives control with the chip fully clocked.

---

## 2. `app_init()`

```
app_init()
│
├── hal_init()
│   ├── [mbed]   i2c.frequency(400000)          — set I2C bus to 400 kHz
│   │            s_timer.start()                 — start μs counter
│   │            LED initial state = off
│   └── [arduino] Wire.begin(3, 4)              — SDA=GPIO3, SCL=GPIO4
│                 Wire.setClock(400000)
│                 Serial.begin(38400)
│                 wait up to 3 s for USB CDC
│                 pinMode(13, OUTPUT)
│
├── hal_led1_toggle()                            — LED1 on: signals init started
│
├── imu.begin()                                  — see §3
│   └── returns false → hal_log error, blink LED1 at 1 Hz, halt forever
│
├── lastUs = hal_micros()                        — seed dt timer
│
└── hal_led2_toggle()                            — LED2 on: init complete
```

---

## 3. `MPU9250::begin()` — one-shot sensor startup

Total blocking time: ≈3.4 seconds

```
begin()
│
├── readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250)
│   └── expect 0x71 — abort (return false) if mismatch
│
├── reset()
│   ├── writeByte(PWR_MGMT_1, 0x80)             — assert reset bit
│   └── hal_delay_ms(100)
│
├── selfTest(results[6])                         — ~200 ms
│   ├── Configure for self-test: SMPLRT_DIV=0, CONFIG=0x02
│   │   GYRO_CONFIG=0x01, ACCEL_CONFIG2=0x02, ACCEL_CONFIG=0x01
│   ├── Collect 200 samples of normal accel + gyro → aAvg[3], gAvg[3]
│   ├── Enable self-test: ACCEL_CONFIG=0xE0, GYRO_CONFIG=0xE0
│   ├── hal_delay_us(25000)
│   ├── Collect 200 samples with self-test enabled → aSTAvg[3], gSTAvg[3]
│   ├── Restore normal config: ACCEL_CONFIG=0, GYRO_CONFIG=0
│   ├── hal_delay_us(25000)
│   ├── Read factory trim codes from SELF_TEST_X/Y/Z_ACCEL and _GYRO
│   ├── factoryTrim[i] = 2620 * 1.01^(code[i] - 1)
│   └── results[i] = 100 * (STAvg[i] - Avg[i]) / factoryTrim[i]   (% deviation)
│       hal_log prints the six self-test percentages
│
├── calibrate()                                  — ~350 ms
│   ├── writeByte(PWR_MGMT_1, 0x80)             — reset
│   ├── hal_delay_ms(100)
│   ├── writeByte(PWR_MGMT_1, 0x01)             — clock from x-gyro PLL
│   ├── hal_delay_ms(200)
│   ├── Disable interrupts, FIFO, I2C master; reset FIFO+DMP
│   ├── hal_delay_ms(15)
│   ├── Configure: CONFIG=0x01 (188 Hz LPF), SMPLRT_DIV=0 (1 kHz)
│   │             GYRO_CONFIG=0 (250 dps), ACCEL_CONFIG=0 (2 g)
│   ├── Enable FIFO (USER_CTRL |= 0x40)
│   ├── Enable gyro+accel FIFO capture (FIFO_EN=0x78)
│   ├── hal_delay_ms(40)                         — collect ~40 packets
│   ├── Disable FIFO capture (FIFO_EN=0x00)
│   ├── Read FIFO_COUNT → packetCount = fifoCount / 12
│   ├── For each packet: accumulate raw accel + gyro into 32-bit sums
│   ├── Divide by packetCount → average bias per axis
│   ├── Subtract 1 g (16384 LSB) from accel Z-axis (gravity)
│   ├── _gyroBias[i]  = gyro_avg[i]  / 131         [deg/s]
│   └── _accelBias[i] = accel_avg[i] / 16384        [g]
│       (hardware offset registers left unchanged for portability)
│
├── hal_delay_ms(2000)                           — settling wait
│
├── initMPU9250()
│   ├── writeByte(PWR_MGMT_1, 0x00)             — wake all sensors
│   ├── hal_delay_ms(100)
│   ├── writeByte(PWR_MGMT_1, 0x01)             — PLL from x-gyro
│   ├── writeByte(CONFIG, 0x03)                  — DLPF 41 Hz, 1 kHz gyro rate
│   ├── writeByte(SMPLRT_DIV, 0x04)             — ÷5 → 200 Hz output
│   ├── GYRO_CONFIG: clear self-test bits, set Gscale (250 dps default)
│   ├── ACCEL_CONFIG: clear self-test bits, set Ascale (2 g default)
│   ├── ACCEL_CONFIG2: clear fchoice, set A_DLPF 41 Hz
│   ├── writeByte(INT_PIN_CFG, 0x22)            — active-high, latched, I2C bypass
│   └── writeByte(INT_ENABLE, 0x01)             — data-ready interrupt enable
│
├── initAK8963()
│   ├── writeByte(AK8963, AK8963_CNTL, 0x00)   — power down
│   ├── hal_delay_ms(10)
│   ├── writeByte(AK8963, AK8963_CNTL, 0x0F)   — Fuse ROM access
│   ├── hal_delay_ms(10)
│   ├── readBytes(AK8963_ASAX, 3) → rawData[3]
│   ├── _magCalibration[i] = (rawData[i] - 128) / 256.0 + 1.0
│   ├── writeByte(AK8963, AK8963_CNTL, 0x00)   — power down
│   ├── hal_delay_ms(10)
│   ├── writeByte(AK8963, AK8963_CNTL, Mscale<<4 | Mmode)
│   │   — 16-bit, 100 Hz continuous (default)
│   └── hal_delay_ms(10)
│
├── hal_delay_ms(100)
│
├── computeAres()   → _aRes = 2.0 / 32768.0  = 6.1e-5 g/LSB
├── computeGres()   → _gRes = 250.0 / 32768.0 = 7.6e-3 dps/LSB
└── computeMres()   → _mRes = 10 * 4219 / 32760 = 1.286 mG/LSB
```

---

## 4. Main loop — `app_loop()`

Called repeatedly (mbed: `while(1)`; Arduino: implicit by framework).

```
app_loop()
│
├── computeDt()
│   ├── now = hal_micros()
│   ├── elapsed = now - lastUs   (unsigned subtraction, handles 32-bit wrap)
│   ├── lastUs = now
│   └── dt = elapsed * 1e-6     (seconds); clamped to [0, 0.5]
│
├── imu.readSensors()
│   ├── readByte(INT_STATUS) & 0x01    — check data-ready bit
│   │   ├── true (newData):
│   │   │   ├── readAccelData() → 6 bytes from ACCEL_XOUT_H
│   │   │   │   ax = raw[0] * _aRes - _accelBias[0]     [g]
│   │   │   │   ay = raw[1] * _aRes - _accelBias[1]
│   │   │   │   az = raw[2] * _aRes - _accelBias[2]
│   │   │   └── readGyroData() → 6 bytes from GYRO_XOUT_H
│   │   │       gx = raw[0] * _gRes - _gyroBias[0]      [deg/s]
│   │   │       gy = raw[1] * _gRes - _gyroBias[1]
│   │   │       gz = raw[2] * _gRes - _gyroBias[2]
│   │   └── false: skip accel/gyro read
│   │
│   ├── readMagData()
│   │   ├── readByte(AK8963_ST1) & 0x01   — data ready?
│   │   ├── true: readBytes(AK8963_XOUT_L, 7) → raw[7] (little-endian + ST2)
│   │   │   ├── check ST2 bit 3 (overflow) — skip if set
│   │   │   ├── mx = raw[0,1] * _mRes * _magCalibration[0] - _magBias[0]  [mG]
│   │   │   ├── my = raw[2,3] * _mRes * _magCalibration[1] - _magBias[1]
│   │   │   └── mz = raw[4,5] * _mRes * _magCalibration[2] - _magBias[2]
│   │   └── false: leave mx/my/mz unchanged
│   │
│   ├── readTempData() → 2 bytes from TEMP_OUT_H
│   │   temperature = raw / 333.87 + 21.0                 [°C]
│   │
│   └── return newData
│
└── if (newData):
    │
    ├── ahrs.update(ax, ay, az,
    │              gx*kDegToRad, gy*kDegToRad, gz*kDegToRad,  — convert to rad/s
    │              my, mx, mz,                                 — X/Y swap for NED
    │              dt)
    │   │
    │   ├── Normalise accelerometer vector
    │   ├── Normalise magnetometer vector
    │   ├── Compute reference magnetic field in Earth frame (hx, hy, bx, bz)
    │   ├── Estimate expected gravity (vx, vy, vz) and magnetic (wx, wy, wz)
    │   │   directions from current quaternion
    │   ├── Error = cross product (measured × expected) for each reference
    │   │   ex = (ay*vz - az*vy) + (my*wz - mz*wy)
    │   │   ey = (az*vx - ax*vz) + (mz*wx - mx*wz)
    │   │   ez = (ax*vy - ay*vx) + (mx*wy - my*wx)
    │   ├── Integral feedback: eInt += error  (only if Ki > 0)
    │   ├── Corrected gyro: g += Kp*error + Ki*eInt
    │   ├── Integrate quaternion rate:
    │   │   dq/dt = 0.5 * q ⊗ [0, gx, gy, gz]
    │   │   q += dq * dt
    │   └── Normalise quaternion
    │
    ├── yaw   = ahrs.getYaw()   - MAG_DECLINATION_DEG    [deg]
    ├── pitch = ahrs.getPitch()                           [deg]
    ├── roll  = ahrs.getRoll()                            [deg]
    │
    ├── hal_log("%.3f %.3f %.3f  %.2f %.2f %.2f\r\n",
    │           gx, gy, gz, yaw, pitch, roll)
    │
    └── hal_led1_toggle()    — visual heartbeat at data rate (~200 Hz)
```

---

## 5. Timing summary

| Phase | Duration | Notes |
|---|---|---|
| `hal_init()` | < 5 ms | I2C config, timer start |
| `reset()` | 100 ms | Waits for MPU-9250 reset |
| `selfTest()` | ≈200 ms | 200×2 sample collections + 25 ms stabilisation waits |
| `calibrate()` | ≈355 ms | 100 ms reset + 200 ms PLL + 15 ms FIFO reset + 40 ms data |
| Settling wait | 2000 ms | Allows oscillations after calibration to damp out |
| `initMPU9250()` | 100 ms | Dominated by PWR_MGMT_1 PLL wait |
| `initAK8963()` | 40 ms | Four 10 ms delays between mode transitions |
| **Total init** | **≈3.4 s** | |
| **Steady-state loop** | **5 ms/cycle** | 200 Hz data rate (SMPLRT_DIV = 4) |

---

## 6. Error path

If `imu.begin()` returns `false` (WHO_AM_I mismatch — sensor absent or I2C wiring fault):

```
hal_log("ERROR: MPU9250 not found — halting.\r\n")
loop:
    hal_delay_ms(500)
    hal_led1_toggle()         — 1 Hz blink signals fault
```

The firmware does not attempt recovery. Power-cycle or reset the board after fixing the wiring.

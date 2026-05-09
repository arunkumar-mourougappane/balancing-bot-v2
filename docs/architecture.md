# Architecture

## Overview

The firmware is organised into three independent libraries under `lib/` and a thin application in `src/`. Each layer has a single responsibility and no knowledge of the layers above it.

```text
┌────────────────────────────────────────────────────────────────────────┐
│                      src/main.cpp  (Application)                       │
│                                                                        │
│   app_init()          ──────────────────────►  MPU9250.begin()         │
│   app_loop()          ──────────────────────►  MPU9250.readSensors()   │
│                       ──────────────────────►  Mahony.update()         │
│                       ──────────────────────►  hal_log()               │
└───────────────────────┬────────────────────────────────────────────────┘
                        │ includes
           ┌────────────┴──────────────────────────────┐
           │                                           │
           ▼                                           ▼
┌──────────────────────┐               ┌─────────────────────────┐
│    lib/MPU9250       │               │       lib/AHRS          │
│                      │               │                         │
│  MPU9250_registers.h │               │  Mahony.h / Mahony.cpp  │
│  MPU9250.h           │               │  Madgwick.h / Madgwick  │
│  MPU9250.cpp         │               │                         │
│                      │               │  Pure float arithmetic. │
│  Calls HAL for I2C,  │               │  Zero hardware deps.    │
│  timing, and logging │               │                         │
└──────────┬───────────┘               └─────────────────────────┘
           │ calls hal_i2c_*, hal_delay_*, hal_log
           ▼
┌──────────────────────────────────────────────────────────────┐
│                       lib/HAL                                │
│                                                              │
│               hal.h  (platform-independent interface)        │
│                                                              │
│  ┌───────────────────────┐   ┌──────────────────────────┐    │
│  │    hal_mbed.cpp       │   │    hal_arduino.cpp       │    │
│  │                       │   │                          │    │
│  │  mbed::I2C            │   │  Arduino Wire            │    │
│  │  mbed::Timer          │   │  millis() / micros()     │    │
│  │  mbed::DigitalOut     │   │  digitalWrite()          │    │
│  │  printf() → USB UART  │   │  Serial.print()          │    │
│  │                       │   │                          │    │
│  │  #ifdef HAL_MBED      │   │  #ifdef HAL_ARDUINO      │    │
│  └───────────────────────┘   └──────────────────────────┘    │
└──────────────────────────────────────────────────────────────┘
           │                               │
           ▼                               ▼
  mbed OS 5 (LPC1768 / FRDM-K64F)   Arduino / ESP-IDF (ESP32-S3)
```

---

## lib/HAL — Hardware Abstraction Layer

**Purpose:** Isolates every hardware call behind a single C-linkage interface so that `lib/MPU9250` and `src/main.cpp` compile identically on all three targets.

### Interface (`include/hal.h`)

| Function                            | Description                                                                    |
|-------------------------------------|--------------------------------------------------------------------------------|
| `hal_init()`                        | Configure I2C at 400 kHz, start timer, open serial port, set LED initial state |
| `hal_delay_ms(ms)`                  | Blocking millisecond delay                                                     |
| `hal_delay_us(us)`                  | Blocking microsecond delay                                                     |
| `hal_micros()`                      | 32-bit microsecond counter. Wraps after ≈71 minutes                            |
| `hal_i2c_write(addr, reg, data)`    | Write one byte to a 7-bit I2C device register                                  |
| `hal_i2c_read(addr, reg, buf, len)` | Burst-read `len` bytes starting at `reg`                                       |
| `hal_log(fmt, ...)`                 | `printf`-style formatted output over USB serial                                |
| `hal_led1_toggle()`                 | Toggle status LED 1                                                            |
| `hal_led2_toggle()`                 | Toggle status LED 2                                                            |

### I2C addressing

All I2C addresses throughout the codebase are **7-bit** (as in the MPU-9250 datasheet). Each HAL implementation translates to the convention its framework expects:

| Framework    | Convention                | Translation                                         |
|--------------|---------------------------|-----------------------------------------------------|
| mbed         | 8-bit (left-shifted by 1) | HAL shifts: `i2c.write(addr << 1, ...)`             |
| Arduino Wire | 7-bit                     | HAL passes directly: `Wire.beginTransmission(addr)` |

### mbed implementation (`src/hal_mbed.cpp`)

- Uses `mbed::I2C i2c(I2C_SDA, I2C_SCL)` — pin macros resolve per board from the mbed target definition.
- Uses `mbed::Timer` with `elapsed_time()` (chrono API, avoids deprecated `read_us()`).
- `hal_log` calls `vprintf`, which mbed routes to the USB serial. Baud rate is set to 38400 via `mbed_app.json` (`platform.stdio-baud-rate`).
- LED polarity is board-specific: **LPC1768** LEDs are active-high; **FRDM-K64F** RGB LEDs are active-low.

| Board     | LED 1 pin           | LED 2 pin          | Polarity    |
|-----------|---------------------|--------------------|-------------|
| LPC1768   | `LED1` (p1.18)      | `LED2` (p1.20)     | Active-high |
| FRDM-K64F | `LED1` (PTB22, red) | `LED_BLUE` (PTB21) | Active-low  |

### Arduino implementation (`src/hal_arduino.cpp`)

- Uses `Wire.begin(3, 4)` with `Wire.setClock(400000)` — GPIO 3/4 are the SDA/SCL lines on the ESP32-S3 TFT Feather Feather header and STEMMA QT connector.
- `hal_micros()` wraps `micros()`.
- Waits up to 3 seconds for USB CDC enumeration before continuing (`while (!Serial && millis() < 3000)`).
- GPIO 13 is used for both LED slots. Extend if a second LED is wired.

### Build-flag selection

PlatformIO passes `-DHAL_MBED` or `-DHAL_ARDUINO` via `build_flags` in `platformio.ini`. Both `.cpp` files are compiled in every build; the `#ifdef` at the top of each file ensures only the correct implementation emits object code.

---

## lib/MPU9250 — IMU Driver

**Purpose:** Full lifecycle management of the MPU-9250 sensor: WHO_AM_I check, self-test, FIFO-based calibration, register configuration, and per-sample data reads.

### Files

| File                          | Contents                                                                                               |
|-------------------------------|--------------------------------------------------------------------------------------------------------|
| `include/MPU9250_registers.h` | Complete register map. All I2C addresses are 7-bit. Enums for `Ascale`, `Gscale`, `Mscale`.            |
| `include/MPU9250.h`           | `MPU9250` class declaration. Public sensor values (`ax/ay/az`, `gx/gy/gz`, `mx/my/mz`, `temperature`). |
| `src/MPU9250.cpp`             | All method implementations. Calls only `hal_*` functions — no direct mbed or Arduino code.             |

### Class design

```text
MPU9250
├── Public data (updated by readSensors)
│   ax, ay, az      [g]
│   gx, gy, gz      [deg/s]
│   mx, my, mz      [mG]
│   temperature     [°C]
│
├── Public methods
│   begin()         — full startup sequence, returns false on WHO_AM_I mismatch
│   readSensors()   — poll INT_STATUS, read all axes, returns true when fresh data
│   setMagBias()    — override environmental mag bias
│   gyroBias()      — read-only access to calibrated gyro bias array
│   accelBias()     — read-only access to calibrated accel bias array
│
└── Private internals
    writeByte / readByte / readBytes  — HAL wrappers
    reset / initMPU9250 / initAK8963  — register configuration
    calibrate                         — FIFO-based at-rest bias estimation
    selfTest                          — factory self-test comparison
    computeAres / computeGres / computeMres  — LSB-to-unit scale factors
```

### Sensor configuration (defaults)

| Sensor        | Scale                  | Bandwidth | Output rate |
|---------------|------------------------|-----------|-------------|
| Accelerometer | ±2 g (16384 LSB/g)     | 41 Hz     | 200 Hz      |
| Gyroscope     | ±250 dps (131 LSB/dps) | 41 Hz     | 200 Hz      |
| Magnetometer  | 16-bit (0.15 mG/LSB)   | —         | 100 Hz      |

The 200 Hz accel/gyro rate is set by `SMPLRT_DIV = 4` applied to the 1 kHz internal rate.

### Calibration

`calibrate()` uses the hardware FIFO to collect 40 ms of at-rest samples (≈40 packets at 1 kHz), averages them, subtracts 1 g from the z-axis (gravity compensation), and stores the result as software bias arrays. Hardware offset registers are intentionally left unwritten for portability across mbed and Arduino targets.

### Magnetometer calibration

`initAK8963()` reads the AK8963 factory sensitivity adjustment values from Fuse ROM (`AK8963_ASAX/Y/Z`) and stores them as `_magCalibration[3]`. The environmental bias (`_magBias[3]`) is applied at read time:

```
mx = rawX * mRes * magCalibration[0] - magBias[0]
```

---

## lib/AHRS — Attitude & Heading Reference System

**Purpose:** Fuse accel, gyro, and mag into a quaternion orientation estimate and expose Tait-Bryan Euler angles. Contains zero hardware dependencies.

### Mahony filter (`Mahony.h / Mahony.cpp`)

A complementary filter that drives the quaternion toward the accelerometer and magnetometer reference vectors using proportional-integral feedback on the cross-product error.

**Tuning parameters:**

| Parameter | Default | Effect                                                                                              |
|-----------|---------|-----------------------------------------------------------------------------------------------------|
| `Kp`      | `10.0`  | Proportional gain. Higher = faster convergence to accel/mag reference; more noise in steady state   |
| `Ki`      | `0.0`   | Integral gain. > 0 integrates the error to cancel persistent gyro bias; disabled to prevent wind-up |

**State:** `_q[4]` (quaternion w,x,y,z), `_eInt[3]` (integral error accumulator).

**NED axis swap:** The application passes `(my, mx, mz)` to the filter (X and Y swapped) to convert from the sensor's body frame to the North-East-Down convention expected by the Mahony reference field computation.

### Madgwick filter (`Madgwick.h / Madgwick.cpp`)

A gradient-descent filter that minimises the difference between the quaternion-rotated reference vectors and measured sensor directions. More computationally intensive than Mahony but requires only one tuning parameter.

**Tuning parameter:**

| Parameter | Default | Typical range                                                                                           |
|-----------|---------|---------------------------------------------------------------------------------------------------------|
| `beta`    | `0.1`   | 0.01 (low noise, slow) – 1.0 (fast, noisy). Original paper recommends `sqrt(3/4) * gyroMeasError_rad_s` |

### Euler angle convention

Both filters compute Tait-Bryan angles from the quaternion using the aerospace convention (yaw applied first, then pitch, then roll):

```
roll  = atan2(2(q0*q1 + q2*q3),  q0² - q1² - q2² + q3²)  [deg]
pitch = -asin(2(q1*q3 - q0*q2))                            [deg]
yaw   = atan2(2(q1*q2 + q0*q3),  q0² + q1² - q2² - q3²)  [deg]
```

Positive yaw is clockwise looking down; positive pitch is nose-up; positive roll is right-side-up.

---

## src/main.cpp — Application

**Purpose:** Wire the three libraries together, compute loop timing, and output data.

### Entry-point switching

```cpp
#ifdef HAL_ARDUINO
void setup() { app_init(); }   // Arduino framework
void loop()  { app_loop(); }
#else
int main() {                   // mbed framework
    app_init();
    while (1) { app_loop(); }
}
#endif
```

`HAL_ARDUINO` is defined by `-DHAL_ARDUINO` in the `esp32s3_tft` build flags; `HAL_MBED` for the other two.

### dt computation

`computeDt()` reads `hal_micros()` on every call and returns the elapsed time in seconds. The timer is 32-bit and wraps after ≈71 minutes; the subtraction is unsigned so it handles the wrap correctly. The result is clamped to `[0, 0.5]` seconds to discard the anomalously large value on the first call (when `lastUs = 0`).

### Output

```
gx gy gz  yaw pitch roll
```

Printed via `hal_log()` only when `readSensors()` returns `true` (new data available), which happens at the 200 Hz data-ready rate.

---

## Build system

### Environments (`platformio.ini`)

```ini
[env:lpc1768]
platform    = nxplpc
board       = lpc1768
framework   = mbed
build_flags = -DHAL_MBED -DBOARD_LPC1768

[env:frdm_k64f]
platform    = freescalekinetis
board       = frdm_k64f
framework   = mbed
build_flags = -DHAL_MBED -DBOARD_FRDM_K64F

[env:esp32s3_tft]
platform    = espressif32
board       = adafruit_feather_esp32s3_tft
framework   = arduino
build_flags = -DHAL_ARDUINO -DBOARD_ESP32S3_TFT
```

### Library discovery

Each library under `lib/` has a `library.json` that declares `includeDir` and `srcDir`. PlatformIO's Library Dependency Finder (LDF) in `chain` mode traverses the dependency graph declared in `library.json` (`MPU9250` → `HAL`) and adds the correct include paths automatically. No `lib_deps` entry is needed in `platformio.ini` for local libs.

---

## Python compatibility patches

The `nxplpc` and `freescalekinetis` platforms ship `framework-mbed` (mbed OS 5.17), which was written against Python 2/3 and uses `imp` and `distutils` — both removed in Python 3.12. The following files in `~/.platformio/packages/framework-mbed/` were patched once:

| File                                                | Original import                               | Replacement                        |
|-----------------------------------------------------|-----------------------------------------------|------------------------------------|
| `platformio/package_deps/py3/past/builtins/misc.py` | `from imp import reload`                      | `from importlib import reload`     |
| `tools/toolchains/mbed_toolchain.py`                | `from distutils.spawn import find_executable` | `try/except` → `shutil.which`      |
| `tools/toolchains/gcc.py`                           | `from distutils.spawn import find_executable` | `try/except` → `shutil.which`      |
| `tools/toolchains/gcc.py`                           | `from distutils.version import LooseVersion`  | `try/except` → inline compat class |
| `tools/toolchains/arm.py`                           | `from distutils.version import LooseVersion`  | `try/except` → inline compat class |
| `tools/toolchains/iar.py`                           | `from distutils.version import LooseVersion`  | `try/except` → inline compat class |
| `tools/utils.py`                                    | `import imp` + `imp.find_module`              | `importlib.util.find_spec`         |

These patches are not part of the project source and will need to be reapplied if PlatformIO reinstalls the mbed framework package. The ESP32 target (Arduino framework) is unaffected.

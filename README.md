# balancing-bot-2

Embedded firmware for a self-balancing robot. Reads a 9-DOF InvenSense MPU-9250 IMU over I2C, fuses the sensor data with a Mahony complementary filter, and streams calibrated gyro rates plus roll/pitch/yaw angles over USB serial at 38400 baud.

Migrated from a single-file mbed sketch into a modular [PlatformIO](https://platformio.org/) project supporting three target boards out of the box.

## Supported targets

| Environment | Board | Framework | I2C pins | LEDs | RAM | Flash |
|---|---|---|---|---|---|---|
| `lpc1768` | [Adafruit mbed LPC1768 #834](https://www.adafruit.com/product/834) | mbed OS 5 | p28 (SDA) / p27 (SCL) | LED1 / LED2 (active-high) | 9.6 KB / 64 KB | 44.9 KB / 512 KB |
| `frdm_k64f` | NXP FRDM-K64F | mbed OS 5 | PTE25 (SDA) / PTE24 (SCL) | LED1 / LED_BLUE (active-low) | 12.6 KB / 256 KB | 59.4 KB / 1 MB |
| `esp32s3_tft` | Adafruit ESP32-S3 TFT Feather | Arduino (ESP-IDF) | GPIO 3 (SDA) / GPIO 4 (SCL) | GPIO 13 (external LED) | 31.3 KB / 320 KB | 321 KB / 1.4 MB |

## Hardware

- **Sensor:** InvenSense MPU-9250 (accelerometer + gyroscope + AK8963 magnetometer) connected via I2C at 400 kHz.
- **IMU address:** 7-bit `0x68` (ADO pin low); magnetometer `0x0C`.
- **Serial output:** USB UART, 38400 baud, one line per sensor update.

## Project structure

```text
balancing-bot-2/
├── src/
│   └── main.cpp             # Application entry point (shared across all targets)
├── lib/
│   ├── HAL/                 # Hardware Abstraction Layer
│   │   ├── include/hal.h    # Platform-independent interface
│   │   └── src/
│   │       ├── hal_mbed.cpp     # mbed implementation (LPC1768, FRDM-K64F)
│   │       └── hal_arduino.cpp  # Arduino implementation (ESP32-S3)
│   ├── MPU9250/             # IMU driver
│   │   ├── include/
│   │   │   ├── MPU9250_registers.h  # Full register map, 7-bit I2C addresses
│   │   │   └── MPU9250.h            # Driver class
│   │   └── src/MPU9250.cpp
│   └── AHRS/                # Attitude & Heading Reference System filters
│       ├── include/
│       │   ├── Mahony.h     # Mahony complementary filter
│       │   └── Madgwick.h   # Madgwick gradient-descent filter
│       └── src/
│           ├── Mahony.cpp
│           └── Madgwick.cpp
├── docs/
│   ├── architecture.md      # Layer design and component responsibilities
│   └── program_flow.md      # Boot and main-loop sequence
├── platformio.ini           # Three build environments
├── mbed_app.json            # mbed config: sets stdio baud rate to 38400
└── LICENSE                  # MIT
```

## Building

Install [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html) then:

```bash
# Build all three targets
pio run

# Build a specific target
pio run -e lpc1768
pio run -e frdm_k64f
pio run -e esp32s3_tft

# Build and upload
pio run -e lpc1768 -t upload

# Open serial monitor (38400 baud)
pio device monitor -e lpc1768
```

> **Python 3.12+ note:** The `framework-mbed` PlatformIO package uses `distutils` and `imp`, which were removed in Python 3.12. Six files in `~/.platformio/packages/framework-mbed/` require one-time patches. See [docs/architecture.md](docs/architecture.md#python-compatibility-patches) for the full list.

## Serial output format

One line per sensor update (≈200 Hz), space-separated:

```
gx gy gz  yaw pitch roll
```

| Field | Unit | Description |
|---|---|---|
| `gx gy gz` | deg/s | Calibrated gyroscope (bias subtracted) |
| `yaw` | deg | Heading relative to magnetic north, corrected for local declination |
| `pitch` | deg | Nose-up positive |
| `roll` | deg | Right-side-up positive |

Example:
```
0.183 -0.061 0.024  312.45 1.23 -0.87
```

## Configuration

All user-tunable constants are at the top of `src/main.cpp`:

| Constant | Default | Description |
|---|---|---|
| `MAG_DECLINATION_DEG` | `-13.8` | Magnetic declination for your location. Look up at [ngdc.noaa.gov](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml) |
| `Mahony Kp` | `10.0` | Proportional feedback gain — controls convergence speed |
| `Mahony Ki` | `0.0` | Integral gain — set > 0 to compensate long-term gyro drift |

To override the hard-coded magnetometer bias (currently set for Danville CA, 2014), uncomment and edit in `app_init()`:

```cpp
imu.setMagBias(470.0f, 120.0f, 125.0f);  // [mG] x, y, z
```

Sensor scales can be changed via the `MPU9250` constructor (defaults shown):

```cpp
MPU9250 imu(AFS_2G, GFS_250DPS, MFS_16BITS, 0x06 /* 100 Hz mag ODR */);
```

## Switching to the Madgwick filter

The `Madgwick` class in `lib/AHRS/` is a drop-in alternative. In `main.cpp`:

```cpp
// Replace:
#include "Mahony.h"
static Mahony ahrs(10.0f, 0.0f);

// With:
#include "Madgwick.h"
static Madgwick ahrs(0.1f);   // beta gain; higher = faster, noisier
```

The `update()` and `getRoll()`/`getPitch()`/`getYaw()` interface is identical.

## Documentation

- [Architecture overview](docs/architecture.md) — layer design, HAL interface, library responsibilities, build-flag selection
- [Program flow](docs/program_flow.md) — step-by-step boot sequence and main-loop timing

## License

Released under the [MIT License](./LICENSE) — Copyright 2026 Arunkumar Mourougappane.

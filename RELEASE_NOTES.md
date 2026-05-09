# Release Notes — v1.0.0 (2026-05-09)

Initial release of **balancing-bot-2**: a modular, multi-target PlatformIO
firmware for a self-balancing robot IMU pipeline.

---

## Overview

This release establishes the full sensor-fusion foundation migrated from a
single-file mbed sketch (`Gyro-mag-FRDM`) into a layered library structure
that compiles cleanly on three hardware targets without any source changes.

The firmware reads an InvenSense MPU-9250 9-DOF IMU over I2C, runs a Mahony
complementary filter to fuse accelerometer, gyroscope, and magnetometer data
into a quaternion, and streams calibrated gyro rates plus roll/pitch/yaw
angles over USB serial at 38400 baud — ready to feed a balancing controller.

---

## What's included

### Three ready-to-flash targets

| Target | Board | Framework | RAM | Flash |
|--------|-------|-----------|-----|-------|
| `lpc1768` | [Adafruit mbed LPC1768 #834](https://www.adafruit.com/product/834) | mbed OS 5 | 9.6 KB / 64 KB (14.7%) | 44.9 KB / 512 KB (8.6%) |
| `frdm_k64f` | NXP Freedom-K64F | mbed OS 5 | 12.6 KB / 256 KB (4.8%) | 59.4 KB / 1 MB (5.7%) |
| `esp32s3_tft` | Adafruit ESP32-S3 TFT Feather | Arduino / ESP-IDF | 31.3 KB / 320 KB (9.6%) | 321 KB / 1.4 MB (22.3%) |

Pre-compiled binaries for all three targets are attached to this release.

### Modular library architecture

```
lib/HAL/       Platform abstraction (I2C, timing, logging, GPIO)
lib/MPU9250/   IMU driver — self-test, calibration, sensor reads
lib/AHRS/      Mahony and Madgwick fusion filters (pure math, zero hardware deps)
src/main.cpp   Application — unified entry point for mbed and Arduino
```

Each library is independently versioned and has a `library.json` for
PlatformIO's dependency resolver.

### CI/CD pipeline

- **Every push / PR** builds all three targets in parallel and uploads
  firmware artifacts to the Actions run.
- **Version tag push** (`v*.*.*`) additionally creates a GitHub release and
  attaches the compiled binaries automatically.

---

## Flashing

### From pre-compiled binary

Download the `.bin` for your board from the Assets section below and flash
with your board's drag-and-drop USB mass-storage bootloader (LPC1768 and
FRDM-K64F ship with one), or use a compatible programmer.

### From source

```bash
git clone https://github.com/arunkumar-mourougappane/balancing-bot-v2.git
cd balancing-bot-v2

# Build and upload in one step
pio run -e lpc1768 -t upload        # or frdm_k64f / esp32s3_tft

# Open serial monitor
pio device monitor -e lpc1768
```

Requires [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html).

> **Python 3.12+ users:** `framework-mbed` needs seven one-time patches.
> See [docs/architecture.md](docs/architecture.md#python-compatibility-patches)
> for the complete list. CI targets Python 3.11 and is unaffected.

---

## Serial output

One line per sensor update at ≈200 Hz, 38400 baud:

```
gx gy gz  yaw pitch roll
```

| Field | Unit | Notes |
|-------|------|-------|
| `gx gy gz` | deg/s | Gyroscope, calibrated bias subtracted |
| `yaw` | deg | Magnetic heading, declination corrected |
| `pitch` | deg | Nose-up positive |
| `roll` | deg | Right-side-up positive |

---

## Configuration

Update these constants in `src/main.cpp` before flashing:

| Constant | Default | Action required |
|----------|---------|-----------------|
| `MAG_DECLINATION_DEG` | `-13.8` | **Must update** — set for your location via [NOAA calculator](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml) |
| `Mahony Kp` | `10.0` | Tune for convergence speed vs. noise |
| `Mahony Ki` | `0.0` | Enable to correct long-term gyro drift |

Override the environmental magnetometer bias (currently Danville CA, 2014):

```cpp
// In app_init(), uncomment and set values for your site [mG]:
imu.setMagBias(470.0f, 120.0f, 125.0f);
```

---

## Known limitations

- **Magnetometer bias is location-specific.** The hard-coded values must be
  measured or overridden for accurate yaw.
- **Magnetic declination is hard-coded.** Update `MAG_DECLINATION_DEG` for
  your location.
- **ESP32-S3 NeoPixel is not driven.** The onboard RGB NeoPixel requires a
  separate library; GPIO 13 is used as a simple external LED.
- **Hardware offset registers are not written.** Gyro and accelerometer
  biases are corrected in software for portability. Future releases may write
  the hardware trim registers where supported.
- **framework-mbed Python patches are local only.** CI pins Python 3.11 to
  avoid the issue; local builds on Python 3.12+ require one-time framework
  file patches documented in the architecture guide.

---

## Full changelog

See [CHANGELOG.md](CHANGELOG.md) for the complete list of changes in this
release.

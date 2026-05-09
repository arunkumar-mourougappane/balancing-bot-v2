# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.0.0] - 2026-05-09

Initial release. Migrated from a single-file mbed sketch (`Gyro-mag-FRDM`)
into a modular, multi-target PlatformIO firmware with CI/CD.

### Added

#### Libraries
- **`lib/HAL`** — platform-independent interface (`hal.h`) covering I2C
  (7-bit addressing), millisecond/microsecond delays, a 32-bit microsecond
  counter, `printf`-style logging, and LED toggle
- **`lib/HAL/src/hal_mbed.cpp`** — mbed OS 5 implementation: `mbed::I2C`
  at 400 kHz, `mbed::Timer` via chrono API, `vprintf` to USB serial; supports
  LPC1768 (active-high LEDs) and FRDM-K64F (active-low RGB LEDs) via
  `BOARD_*` compile flags
- **`lib/HAL/src/hal_arduino.cpp`** — Arduino implementation: `Wire` at
  400 kHz (GPIO 3/4 on ESP32-S3 TFT Feather), `Serial` at 38400 baud,
  3-second USB CDC wait, GPIO 13 LED
- **`lib/MPU9250`** — full-lifecycle driver for the InvenSense MPU-9250 9-DOF
  IMU: WHO_AM_I check, factory self-test, FIFO-based at-rest calibration,
  register initialisation (200 Hz / 41 Hz bandwidth), AK8963 magnetometer
  Fuse ROM sensitivity read, per-sample interrupt-driven reads
- **`lib/MPU9250/include/MPU9250_registers.h`** — complete register map;
  all I2C addresses canonical 7-bit
- **`lib/AHRS`** — attitude and heading reference system filters with no
  hardware dependencies
- **`lib/AHRS` — Mahony** complementary filter: proportional-integral
  feedback, configurable Kp/Ki, quaternion integration, Tait-Bryan Euler
  output (roll/pitch/yaw in degrees)
- **`lib/AHRS` — Madgwick** gradient-descent filter: configurable beta gain,
  same quaternion and Euler interface as Mahony

#### Application
- **`src/main.cpp`** — unified entry point: `app_init()` / `app_loop()`
  wrapped with `#ifdef HAL_ARDUINO` for `setup()`/`loop()` vs `main()`;
  wrap-safe 32-bit dt computation; 200 Hz sensor poll; Mahony-fused gyro +
  YPR output over serial

#### Build & CI
- **`platformio.ini`** — three environments: `lpc1768` (nxplpc/mbed),
  `frdm_k64f` (freescalekinetis/mbed), `esp32s3_tft` (espressif32/arduino);
  per-environment `HAL_MBED`/`HAL_ARDUINO` and `BOARD_*` compile flags;
  shared 38400 baud monitor speed
- **`mbed_app.json`** — sets `platform.stdio-baud-rate = 38400` and
  `platform.stdio-convert-newlines = true` via `target_overrides`
- **`.github/workflows/build.yml`** — GitHub Actions: matrix build across all
  three targets in parallel; Python 3.11 pin (mbed framework compatibility);
  per-environment `~/.platformio` cache; firmware artifact upload; tag-push
  release job using `softprops/action-gh-release@v2` with auto-generated
  changelog and pre-release detection from tag suffix

#### Documentation
- `docs/architecture.md` — layer diagram, HAL interface table, I2C addressing
  explanation, sensor configuration, AHRS filter tuning guide, build-flag
  selection, Python compat patch table
- `docs/program_flow.md` — sequential boot sequence with register writes and
  timing, full main-loop expansion including Mahony internals, timing summary
  table, error path

### Changed

- Converted all I2C addresses from the original mbed 8-bit convention to
  7-bit canonical values; each HAL implementation shifts as required by its
  framework
- Extracted `MahonyQuaternionUpdate` and `MadgwickQuaternionUpdate` from the
  monolithic `MPU9250.h` into the standalone `lib/AHRS` library
- Moved all filter and sensor state from file-scope globals into class instance
  variables (`MPU9250`, `Mahony`, `Madgwick`)
- Replaced deprecated `mbed::Timer::read_us()` with
  `elapsed_time().count()` (chrono API)
- Renamed `DEG_TO_RAD` constant to `kDegToRad` to avoid collision with the
  same-name macro defined in Arduino's `Arduino.h`
- `mbed_app.json`: moved `platform.stdio-baud-rate` from `config` section
  (invalid) to `target_overrides` (correct)

### Fixed

- **Python 3.12+ compatibility** — `framework-mbed` (mbed OS 5.17) uses
  `imp` and `distutils`, both removed in Python 3.12. Seven files patched in
  `~/.platformio/packages/framework-mbed/`:

  | File | Fix |
  |---|---|
  | `past/builtins/misc.py` | `from imp import reload` → `from importlib import reload` |
  | `tools/toolchains/mbed_toolchain.py` | `distutils.spawn.find_executable` → `shutil.which` |
  | `tools/toolchains/gcc.py` | `distutils.spawn.find_executable` → `shutil.which` |
  | `tools/toolchains/gcc.py` | `distutils.version.LooseVersion` → inline compat class |
  | `tools/toolchains/arm.py` | `distutils.version.LooseVersion` → inline compat class |
  | `tools/toolchains/iar.py` | `distutils.version.LooseVersion` → inline compat class |
  | `tools/utils.py` | `imp.find_module` → `importlib.util.find_spec` |

- Removed unused variables `avg`, `stAvg` from `MPU9250::selfTest()` and
  `timerWrapUs` from `main.cpp` (suppressed `-Wunused-variable` warnings)

### Known limitations

- Magnetometer environmental bias is hard-coded for Danville CA (2014);
  must be overridden via `imu.setMagBias()` for accurate heading
- Magnetic declination correction (`-13.8°`) is hard-coded; update
  `MAG_DECLINATION_DEG` in `main.cpp` for your location
- ESP32-S3 TFT Feather onboard NeoPixel is not driven; GPIO 13 is used as a
  simple LED output and requires an external LED
- Hardware gyro/accel offset registers are not written during calibration
  (software bias subtraction only) for cross-platform portability
- framework-mbed Python 3.12+ patches must be reapplied if PlatformIO
  reinstalls the mbed framework package; CI is unaffected (pins Python 3.11)

[Unreleased]: https://github.com/arunkumar-mourougappane/balancing-bot-v2/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/arunkumar-mourougappane/balancing-bot-v2/releases/tag/v1.0.0

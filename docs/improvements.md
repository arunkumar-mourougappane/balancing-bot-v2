# Improvements over Balancing-Bot v1

A detailed comparison between the original
[Balancing-Bot](https://github.com/arunkumar-mourougappane/Balancing-Bot)
(v1) and this project (v2), covering hardware, architecture, code quality,
build system, and CI/CD.

---

## 1. Sensor upgrade — MPU-6050 → MPU-9250

| Aspect | v1 (MPU-6050) | v2 (MPU-9250) |
|--------|---------------|---------------|
| DOF | 6 (accel + gyro) | 9 (accel + gyro + magnetometer) |
| Heading reference | None — no yaw correction | AK8963 magnetometer; absolute heading available |
| Yaw drift | Unbounded gyro integration | Mahony/Madgwick fusion corrects drift against magnetic north |
| Mag calibration | N/A | Factory Fuse ROM sensitivity + user environmental bias |
| I2C address (7-bit) | `0x69` | `0x68` (MPU-9250) + `0x0C` (AK8963) |
| Driver size | 133 KB (`MPU6050.cpp`) + 7 KB alt variant | Single focused `MPU9250.cpp` — no dead variants |

The magnetometer axis swap (`my, mx, mz`) passed to the filter converts
from the sensor's body frame to the North-East-Down convention required for
a correct horizontal heading reference.

---

## 2. Sensor fusion — fixed complementary filter → configurable 9-DOF filter

### v1 approach

v1 uses a **complementary filter** with hard-coded weights and an unused
Kalman filter:

```cpp
// v1 main.cpp — weights fixed at compile time
angle = 0.93 * gyroAngle + 0.07 * accelAngle;
```

Three separate Kalman implementations existed (`Kalman.h`, `kalman.c`,
`kalman2.h`), none of which were wired into the motor control output:

```cpp
// v1 main.cpp — syntax error; result discarded regardless
//updatePid(setPoint, kalAngleX;     ← missing closing parenthesis
```

The PID computation ran but its return value was thrown away. The motor
was driven directly by the raw filter angle, making the PID dead code.

### v2 approach

v2 provides two complete, interchangeable 9-DOF fusion filters in
`lib/AHRS/`:

| Filter | Algorithm | Tuning |
|--------|-----------|--------|
| **Mahony** | Proportional-integral feedback on accel/mag reference error | `Kp` (convergence speed), `Ki` (gyro bias correction) |
| **Madgwick** | Gradient-descent minimisation of quaternion error | `beta` (single gain) |

Both produce a unit quaternion from which Tait-Bryan roll/pitch/yaw angles
are derived. The filter is selected by swapping one `#include` and one
constructor call — the `update()` / `getRoll()` / `getPitch()` / `getYaw()`
interface is identical.

Gains are runtime-configurable (constructor parameters), not compile-time
constants. The Mahony filter defaults to `Kp = 10.0, Ki = 0.0`, matching
the original paper's recommendation for initial stabilisation.

---

## 3. Architecture — monolithic sketch → layered libraries

### v1 structure

```
Balancing-Bot/
├── main.cpp          ← control loop, sensor reads, motor drive, filter — all one file
├── MPU6050.h/.cpp    ← 133 KB monolithic driver with global I2C state
├── MPU60501.h/.cpp   ← duplicate simplified driver (purpose undocumented)
├── I2Cdev.h/.cpp     ← Arduino-ported I2C layer with heap allocation
├── i2c0.h/.cpp       ← hardware-level LPC1768 I2C (not thread-safe)
├── Kalman.h          ← class-based Kalman (unused in output)
├── kalman.c          ← C-style Kalman (unused; "XXXXXXX arevoir" TODO)
├── kalman2.h         ← empty placeholder
└── helper_3dmath.h   ← quaternion/vector math (pulled in but not wired up)
```

All global state lives in header files. Multiple redundant implementations
existed side by side with no documented rationale.

### v2 structure

```
balancing-bot-2/
├── src/main.cpp         ← wires the three libraries; ~95 lines
└── lib/
    ├── HAL/             ← platform boundary: I2C, timing, logging, GPIO
    ├── MPU9250/         ← single focused IMU driver; no alternate variants
    └── AHRS/            ← Mahony + Madgwick; zero hardware deps
```

Each library has a single responsibility. `lib/AHRS` contains no I2C
calls. `lib/MPU9250` contains no filter math. `lib/HAL` contains no
sensor logic. Dependencies only flow downward: `main → MPU9250/AHRS → HAL`.

---

## 4. I2C layer — heap-allocating port → native HAL

### v1 I2Cdev issues

`I2Cdev::readBytes()` allocated a temporary buffer on the heap for every
read:

```cpp
// v1 I2Cdev.cpp
uint8_t *buf = new uint8_t[length];   // ← heap allocation per read
// ... read into buf ...
delete[] buf;                          // ← fragmentation risk
```

`readWords()` returned 0 without reading. `writeWords()` returned `true`
without writing. Both were documented as stubs.

The underlying `i2c0` hardware driver was explicitly documented as
**not thread-safe** and used a software busy flag with no RTOS integration.

A second, separate I2C library (`I2Cdev`) sat on top of the low-level
`i2c0` driver creating an unnecessary abstraction gap.

### v2 HAL approach

`hal_i2c_read()` and `hal_i2c_write()` are thin wrappers over the
framework's own I2C implementation:

```cpp
// v2 hal_mbed.cpp — no heap, no duplication
bool hal_i2c_read(uint8_t addr, uint8_t reg, uint8_t *out, uint8_t len) {
    char w = (char)reg;
    if (s_i2c.write((int)(addr << 1), &w, 1, true) != 0) return false;
    return s_i2c.read((int)(addr << 1), (char *)out, (int)len, false) == 0;
}
```

All buffers are caller-supplied (stack-allocated at the driver level in
`MPU9250.cpp`). No heap is touched in the I2C path. Return values are
checked and propagated to the caller.

I2C addresses are kept as **7-bit canonical values** throughout (matching
the MPU-9250 datasheet) and shifted only at the framework boundary:
mbed shifts left by 1; Arduino Wire uses 7-bit directly.

---

## 5. Global state → class instance variables

### v1 globals

v1 scattered sensor state and configuration into file-scope globals,
including within header files (causing ODR violations if included in
multiple translation units):

```cpp
// v1 MPU6050.h — globals in a header
I2C i2c(p28, p27);
int16_t ax, ay, az, gx, gy, gz;
float kalAngleX, kalAngleY;
float gyroXrate, gyroYrate;
// ... dozens more
```

### v2 instance variables

All state lives inside class instances. The two objects in `main.cpp` are
the only top-level names:

```cpp
static MPU9250 imu;                    // owns all sensor state
static Mahony  ahrs(10.0f, 0.0f);     // owns quaternion + integral error
```

`MPU9250` constructor parameters select sensor scale and ODR without
modifying any global. Two `MPU9250` instances in the same program would
operate independently.

---

## 6. Build system — dual-IDE → PlatformIO

| Aspect | v1 | v2 |
|--------|----|----|
| Primary IDE | Keil µVision (proprietary, Windows) | PlatformIO (open-source, cross-platform) |
| Secondary IDE | Dev-C++ 5.5.1 (unmaintained since 2005) | None needed |
| Build reproducibility | Manual IDE state in `.uvopt`, `.uvgui_*.bak` | `platformio.ini` checked into version control |
| Supported targets | LPC1768 only | LPC1768, FRDM-K64F, ESP32-S3 (one codebase) |
| Framework | mbed OS (implicit, bundled) | mbed OS 5 (lpc1768, frdm_k64f), Arduino/ESP-IDF (esp32s3_tft) |
| Compiler | ARM Developer Suite (Keil bundled) | GCC ARM None EABI 9.2.1 (mbed), Xtensa GCC (ESP32) |
| Binary generation | Keil post-build `fromelf --bin` | `pio run` — `.bin` produced for all targets |

v2 also adds `mbed_app.json` to set the stdio baud rate via the mbed
configuration system rather than relying on Keil project settings.

---

## 7. Multi-board support

v1 targeted a single board (LPC1768) with hardware I2C pins hardcoded
in the `i2c0` driver (`PIO0.27` / `PIO0.28`).

v2 supports three boards with a single shared application:

| Board | I2C | Framework | Compile flags |
|-------|-----|-----------|---------------|
| LPC1768 | `I2C_SDA` (p28) / `I2C_SCL` (p27) | mbed | `-DHAL_MBED -DBOARD_LPC1768` |
| FRDM-K64F | `I2C_SDA` (PTE25) / `I2C_SCL` (PTE24) | mbed | `-DHAL_MBED -DBOARD_FRDM_K64F` |
| ESP32-S3 TFT Feather | GPIO 3 / GPIO 4 | Arduino | `-DHAL_ARDUINO -DBOARD_ESP32S3_TFT` |

The HAL compile-flag mechanism means adding a fourth board requires
only a new `hal_*.cpp` implementation and a new `[env:*]` block in
`platformio.ini`; no library or application code changes.

---

## 8. Code quality

| Issue in v1 | Resolution in v2 |
|-------------|-----------------|
| Syntax error: `updatePid(setPoint, kalAngleX;` (missing `)`) | No dead or broken code paths |
| PID output computed but discarded; motor driven by raw filter angle | Clean data flow: sensor → filter → output; no dead assignments |
| Three Kalman implementations (`Kalman.h`, `kalman.c`, `kalman2.h`) | Single `Mahony` and `Madgwick` classes, both wired to output |
| Two MPU6050 driver variants (`MPU6050`, `MPU60501`) with no documented distinction | Single `MPU9250` class |
| `maindev.cpp` — `#include "MPU60501.cpp"` (including a `.cpp`) | All includes are headers only |
| `I2Cdev::readByte` returns `int8_t` but comparison is done against unsigned values | HAL uses `uint8_t` and `bool` return types throughout |
| `readWords()` / `writeWords()` silently do nothing | All HAL functions return `bool`; callers check |
| Timer: `timer = 0` before first `t.read_us()` → large initial dt spike | `computeDt()` clamps dt to `[0, 0.5]` s on first call and timer wraps |
| Engineering sample workaround in driver (doubles accelerometer values silently) | No hardware-version workarounds; driver assumes production silicon |
| Compiler warnings treated as noise | Warnings resolved; no `-Wunused-variable` in any target |

---

## 9. Documentation

| Aspect | v1 | v2 |
|--------|----|-----|
| README | 93 bytes ("Balancing-Bot\n\nBalancing Bot on LPC1768") | Full README with badges, target table, build instructions, serial format, configuration reference |
| Architecture | None | `docs/architecture.md` — ASCII layer diagram, HAL interface table, I2C addressing rationale, filter tuning guide |
| Program flow | None | `docs/program_flow.md` — full boot sequence with register writes and timing, main-loop expansion, error path |
| Changelog | None | `CHANGELOG.md` (Keep a Changelog format) |
| Release notes | None | `RELEASE_NOTES.md` with flashing instructions, known limitations |

---

## 10. CI/CD and release automation

v1 had no automated build verification. All build outputs depended on a
local Keil µVision installation.

v2 introduces a GitHub Actions pipeline (`.github/workflows/build.yml`):

- **Every push and PR** builds all three targets in parallel; a broken
  commit is caught before it reaches `main`.
- **Tag push** (`v*.*.*`) additionally creates a GitHub release and attaches
  the compiled `.bin` for every board as a release asset automatically.
- **Python 3.11 pin** ensures the mbed framework's `imp`/`distutils`
  dependencies build without patches on the CI runner, independent of
  the host developer's Python version.
- **Per-environment cache** (`~/.platformio`) keeps toolchain download time
  under a few seconds on warm builds.

---

## Summary

| Dimension | v1 | v2 |
|-----------|----|----|
| Sensor DOF | 6 | 9 |
| Filter | Complementary (93/7, fixed) + broken Kalman | Mahony + Madgwick (runtime-tunable) |
| PID integration | Computed but output discarded | Foundation ready; PID connects to filter output |
| Boards supported | 1 (LPC1768) | 3 (LPC1768, FRDM-K64F, ESP32-S3) |
| Frameworks | mbed only | mbed OS 5 + Arduino |
| Build tool | Keil µVision (proprietary) | PlatformIO (open-source) |
| Architecture | Monolithic + globals in headers | Layered libraries, class instance state |
| I2C memory | Heap allocation per read | Stack buffers, caller-supplied |
| Dead code | PID output, unused Kalman variants, empty files | None |
| Syntax errors | 1 (unclosed parenthesis) | 0 |
| CI/CD | None | GitHub Actions — build + release automation |
| Documentation | 93-byte README | README + architecture + program flow + changelog + release notes |
| License | GPL-3.0 | MIT |

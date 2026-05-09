#pragma once
#include <stdint.h>

// Mahony complementary filter.
// Fuses accelerometer, gyroscope, and magnetometer via proportional-integral
// feedback to produce a quaternion orientation estimate.
class Mahony {
public:
    // Kp: proportional gain (higher = faster convergence, more noise).
    // Ki: integral gain (compensates gyro bias drift; 0 disables wind-up).
    explicit Mahony(float Kp = 10.0f, float Ki = 0.0f);

    // Update with 9-DOF data.
    // gx/gy/gz must be in rad/s; ax/ay/az and mx/my/mz can be any consistent unit.
    // dt is the elapsed time since the last call, in seconds.
    void update(float ax, float ay, float az,
                float gx, float gy, float gz,
                float mx, float my, float mz,
                float dt);

    // Euler angles derived from the internal quaternion [degrees].
    float getRoll()  const;
    float getPitch() const;
    float getYaw()   const;

    const float* getQuaternion() const { return _q; }

private:
    float _q[4];        // quaternion: w, x, y, z
    float _eInt[3];     // integral error accumulator
    float _Kp, _Ki;
};

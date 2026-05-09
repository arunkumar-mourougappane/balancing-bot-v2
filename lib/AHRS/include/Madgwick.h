#pragma once
#include <stdint.h>

// Madgwick gradient-descent orientation filter.
// Reference: S. Madgwick, "An efficient orientation filter for inertial and
// inertial/magnetic sensor arrays", 2010.
class Madgwick {
public:
    // beta: filter gain (sqrt(3/4) * gyro_measurement_error_rad_s).
    // A value of ~0.1 is typical for a stabilised system.
    explicit Madgwick(float beta = 0.1f);

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
    float _q[4];    // quaternion: w, x, y, z
    float _beta;
};

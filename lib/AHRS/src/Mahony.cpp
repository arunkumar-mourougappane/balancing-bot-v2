#include "Mahony.h"
#include <math.h>

Mahony::Mahony(float Kp, float Ki)
    : _Kp(Kp), _Ki(Ki)
{
    _q[0] = 1.0f; _q[1] = 0.0f; _q[2] = 0.0f; _q[3] = 0.0f;
    _eInt[0] = 0.0f; _eInt[1] = 0.0f; _eInt[2] = 0.0f;
}

void Mahony::update(float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float mx, float my, float mz,
                    float dt)
{
    float q1 = _q[0], q2 = _q[1], q3 = _q[2], q4 = _q[3];

    // Pre-compute repeated products
    float q1q1 = q1*q1, q1q2 = q1*q2, q1q3 = q1*q3, q1q4 = q1*q4;
    float q2q2 = q2*q2, q2q3 = q2*q3, q2q4 = q2*q4;
    float q3q3 = q3*q3, q3q4 = q3*q4;
    float q4q4 = q4*q4;

    // Normalise accelerometer
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm == 0.0f) return;
    norm = 1.0f / norm;
    ax *= norm; ay *= norm; az *= norm;

    // Normalise magnetometer
    norm = sqrtf(mx*mx + my*my + mz*mz);
    if (norm == 0.0f) return;
    norm = 1.0f / norm;
    mx *= norm; my *= norm; mz *= norm;

    // Reference direction of Earth's magnetic field (horizontal component)
    float hx = 2.0f*mx*(0.5f - q3q3 - q4q4) + 2.0f*my*(q2q3 - q1q4) + 2.0f*mz*(q2q4 + q1q3);
    float hy = 2.0f*mx*(q2q3 + q1q4) + 2.0f*my*(0.5f - q2q2 - q4q4) + 2.0f*mz*(q3q4 - q1q2);
    float bx = sqrtf(hx*hx + hy*hy);
    float bz = 2.0f*mx*(q2q4 - q1q3) + 2.0f*my*(q3q4 + q1q2) + 2.0f*mz*(0.5f - q2q2 - q3q3);

    // Estimated gravity and magnetic field directions
    float vx = 2.0f*(q2q4 - q1q3);
    float vy = 2.0f*(q1q2 + q3q4);
    float vz = q1q1 - q2q2 - q3q3 + q4q4;
    float wx = 2.0f*bx*(0.5f - q3q3 - q4q4) + 2.0f*bz*(q2q4 - q1q3);
    float wy = 2.0f*bx*(q2q3 - q1q4)         + 2.0f*bz*(q1q2 + q3q4);
    float wz = 2.0f*bx*(q1q3 + q2q4)         + 2.0f*bz*(0.5f - q2q2 - q3q3);

    // Cross-product error between estimated and measured directions
    float ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    float ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    float ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    // Integral feedback (only when Ki is non-zero to prevent wind-up)
    if (_Ki > 0.0f) {
        _eInt[0] += ex; _eInt[1] += ey; _eInt[2] += ez;
    } else {
        _eInt[0] = 0.0f; _eInt[1] = 0.0f; _eInt[2] = 0.0f;
    }

    // Corrected gyro rates
    gx += _Kp*ex + _Ki*_eInt[0];
    gy += _Kp*ey + _Ki*_eInt[1];
    gz += _Kp*ez + _Ki*_eInt[2];

    // Integrate rate of change of quaternion (preserving old q2/q3/q4 values)
    float pa = q2, pb = q3, pc = q4;
    q1 += (-q2*gx - q3*gy - q4*gz) * (0.5f*dt);
    q2 = pa + (q1*gx + pb*gz - pc*gy) * (0.5f*dt);
    q3 = pb + (q1*gy - pa*gz + pc*gx) * (0.5f*dt);
    q4 = pc + (q1*gz + pa*gy - pb*gx) * (0.5f*dt);

    // Normalise quaternion
    norm = sqrtf(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    norm = 1.0f / norm;
    _q[0] = q1*norm; _q[1] = q2*norm;
    _q[2] = q3*norm; _q[3] = q4*norm;
}

// Tait-Bryan angles (aerospace convention: yaw then pitch then roll)
float Mahony::getRoll() const {
    return atan2f(2.0f*(_q[0]*_q[1] + _q[2]*_q[3]),
                  _q[0]*_q[0] - _q[1]*_q[1] - _q[2]*_q[2] + _q[3]*_q[3])
           * (180.0f / 3.14159265f);
}

float Mahony::getPitch() const {
    return -asinf(2.0f*(_q[1]*_q[3] - _q[0]*_q[2]))
           * (180.0f / 3.14159265f);
}

float Mahony::getYaw() const {
    return atan2f(2.0f*(_q[1]*_q[2] + _q[0]*_q[3]),
                  _q[0]*_q[0] + _q[1]*_q[1] - _q[2]*_q[2] - _q[3]*_q[3])
           * (180.0f / 3.14159265f);
}

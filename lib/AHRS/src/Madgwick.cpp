#include "Madgwick.h"
#include <math.h>

Madgwick::Madgwick(float beta)
    : _beta(beta)
{
    _q[0] = 1.0f; _q[1] = 0.0f; _q[2] = 0.0f; _q[3] = 0.0f;
}

void Madgwick::update(float ax, float ay, float az,
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

    // Auxiliary variables
    float _2q1mx = 2.0f*q1*mx, _2q1my = 2.0f*q1*my;
    float _2q1mz = 2.0f*q1*mz, _2q2mx = 2.0f*q2*mx;
    float _2q1 = 2.0f*q1, _2q2 = 2.0f*q2;
    float _2q3 = 2.0f*q3, _2q4 = 2.0f*q4;
    float _2q1q3 = 2.0f*q1*q3, _2q3q4 = 2.0f*q3*q4;

    // Earth reference field (horizontal plane projection)
    float hx = mx*q1q1 - _2q1my*q4 + _2q1mz*q3 + mx*q2q2 + _2q2*my*q3 + _2q2*mz*q4 - mx*q3q3 - mx*q4q4;
    float hy = _2q1mx*q4 + my*q1q1 - _2q1mz*q2 + _2q2mx*q3 - my*q2q2 + my*q3q3 + _2q3*mz*q4 - my*q4q4;
    float _2bx = sqrtf(hx*hx + hy*hy);
    float _2bz = -_2q1mx*q3 + _2q1my*q2 + mz*q1q1 + _2q2mx*q4 - mz*q2q2 + _2q3*my*q4 - mz*q3q3 + mz*q4q4;
    float _4bx = 2.0f*_2bx, _4bz = 2.0f*_2bz;

    // Gradient descent corrective step
    float s1 = -_2q3*(2.0f*q2q4 - _2q1q3 - ax) + _2q2*(2.0f*q1q2 + _2q3q4 - ay)
               - _2bz*q3*(_2bx*(0.5f - q3q3 - q4q4) + _2bz*(q2q4 - q1q3) - mx)
               + (-_2bx*q4 + _2bz*q2)*(_2bx*(q2q3 - q1q4) + _2bz*(q1q2 + q3q4) - my)
               + _2bx*q3*(_2bx*(q1q3 + q2q4) + _2bz*(0.5f - q2q2 - q3q3) - mz);
    float s2 = _2q4*(2.0f*q2q4 - _2q1q3 - ax) + _2q1*(2.0f*q1q2 + _2q3q4 - ay)
               - 4.0f*q2*(1.0f - 2.0f*q2q2 - 2.0f*q3q3 - az)
               + _2bz*q4*(_2bx*(0.5f - q3q3 - q4q4) + _2bz*(q2q4 - q1q3) - mx)
               + (_2bx*q3 + _2bz*q1)*(_2bx*(q2q3 - q1q4) + _2bz*(q1q2 + q3q4) - my)
               + (_2bx*q4 - _4bz*q2)*(_2bx*(q1q3 + q2q4) + _2bz*(0.5f - q2q2 - q3q3) - mz);
    float s3 = -_2q1*(2.0f*q2q4 - _2q1q3 - ax) + _2q4*(2.0f*q1q2 + _2q3q4 - ay)
               - 4.0f*q3*(1.0f - 2.0f*q2q2 - 2.0f*q3q3 - az)
               + (-_4bx*q3 - _2bz*q1)*(_2bx*(0.5f - q3q3 - q4q4) + _2bz*(q2q4 - q1q3) - mx)
               + (_2bx*q2 + _2bz*q4)*(_2bx*(q2q3 - q1q4) + _2bz*(q1q2 + q3q4) - my)
               + (_2bx*q1 - _4bz*q3)*(_2bx*(q1q3 + q2q4) + _2bz*(0.5f - q2q2 - q3q3) - mz);
    float s4 = _2q2*(2.0f*q2q4 - _2q1q3 - ax) + _2q3*(2.0f*q1q2 + _2q3q4 - ay)
               + (-_4bx*q4 + _2bz*q2)*(_2bx*(0.5f - q3q3 - q4q4) + _2bz*(q2q4 - q1q3) - mx)
               + (-_2bx*q1 + _2bz*q3)*(_2bx*(q2q3 - q1q4) + _2bz*(q1q2 + q3q4) - my)
               + _2bx*q2*(_2bx*(q1q3 + q2q4) + _2bz*(0.5f - q2q2 - q3q3) - mz);

    norm = sqrtf(s1*s1 + s2*s2 + s3*s3 + s4*s4);
    norm = 1.0f / norm;
    s1 *= norm; s2 *= norm; s3 *= norm; s4 *= norm;

    // Rate of change of quaternion
    float qDot1 = 0.5f*(-q2*gx - q3*gy - q4*gz) - _beta*s1;
    float qDot2 = 0.5f*( q1*gx + q3*gz - q4*gy) - _beta*s2;
    float qDot3 = 0.5f*( q1*gy - q2*gz + q4*gx) - _beta*s3;
    float qDot4 = 0.5f*( q1*gz + q2*gy - q3*gx) - _beta*s4;

    q1 += qDot1*dt; q2 += qDot2*dt;
    q3 += qDot3*dt; q4 += qDot4*dt;

    norm = sqrtf(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    norm = 1.0f / norm;
    _q[0] = q1*norm; _q[1] = q2*norm;
    _q[2] = q3*norm; _q[3] = q4*norm;
}

// Tait-Bryan angles (aerospace convention)
float Madgwick::getRoll() const {
    return atan2f(2.0f*(_q[0]*_q[1] + _q[2]*_q[3]),
                  _q[0]*_q[0] - _q[1]*_q[1] - _q[2]*_q[2] + _q[3]*_q[3])
           * (180.0f / 3.14159265f);
}

float Madgwick::getPitch() const {
    return -asinf(2.0f*(_q[1]*_q[3] - _q[0]*_q[2]))
           * (180.0f / 3.14159265f);
}

float Madgwick::getYaw() const {
    return atan2f(2.0f*(_q[1]*_q[2] + _q[0]*_q[3]),
                  _q[0]*_q[0] + _q[1]*_q[1] - _q[2]*_q[2] - _q[3]*_q[3])
           * (180.0f / 3.14159265f);
}

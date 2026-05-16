// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/estimation/Attitude.h"

#include <math.h>

#include "src/Config.h"

// Madgwick AHRS, six-degree-of-freedom variant, implemented from
// S. O. H. Madgwick, "An efficient orientation filter for inertial and
// inertial/magnetic sensor arrays" (2010), and Madgwick, Harrison,
// Vaidyanathan, "Estimation of IMU and MARG orientation using a
// gradient descent algorithm" (IEEE ICORR 2011). The quaternion rate
// from the gyroscope is corrected by one gradient-descent step toward
// the orientation that best explains the measured gravity direction.

namespace cp::estimation::attitude {

namespace {

constexpr float kPi        = 3.14159265358979323846f;
constexpr float kDegToRad  = kPi / 180.0f;
constexpr float kRadToDeg  = 180.0f / kPi;

// Hover reference orientation: the airframe nose-up, 90 degrees from
// forward-flight level. The body frame is z-up (a level airframe reads
// +1 g on z), so a nose-up pitch is a negative rotation about the body
// y-axis. The quaternion is fromEuler(0, -90 deg, 0) = (cos45, 0,
// -sin45, 0). Used to re-reference the estimate for the hover Euler
// view.
constexpr float kSqrtHalf  = 0.70710678118654752440f;
constexpr Quaternion kHoverRef = {kSqrtHalf, 0.0f, -kSqrtHalf, 0.0f};

Quaternion s_q      = {1.0f, 0.0f, 0.0f, 0.0f};
BodyRates  s_rates  = {0.0f, 0.0f, 0.0f};
bool       s_seeded = false;

// Hamilton product a (x) b.
Quaternion mul(const Quaternion& a, const Quaternion& b) {
  return Quaternion{
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

Quaternion conjugate(const Quaternion& q) {
  return Quaternion{q.w, -q.x, -q.y, -q.z};
}

// Scale a quaternion to unit length. A non-finite or degenerate input
// resets to identity, which catches numerical drift per spec section 3.
Quaternion normalized(const Quaternion& q) {
  const float n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  if (!isfinite(n2) || n2 < 1.0e-12f) {
    return Quaternion{1.0f, 0.0f, 0.0f, 0.0f};
  }
  const float inv = 1.0f / sqrtf(n2);
  return Quaternion{q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

// Quaternion from 3-2-1 (yaw, pitch, roll) Euler angles in radians.
Quaternion fromEulerRad(float roll, float pitch, float yaw) {
  const float cr = cosf(roll * 0.5f);
  const float sr = sinf(roll * 0.5f);
  const float cp = cosf(pitch * 0.5f);
  const float sp = sinf(pitch * 0.5f);
  const float cy = cosf(yaw * 0.5f);
  const float sy = sinf(yaw * 0.5f);
  return Quaternion{
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  };
}

// Standard 3-2-1 Euler extraction from a body-to-world quaternion.
Euler toEuler(const Quaternion& q) {
  Euler e;
  e.roll_deg = atan2f(2.0f * (q.w * q.x + q.y * q.z),
                      1.0f - 2.0f * (q.x * q.x + q.y * q.y)) * kRadToDeg;

  float sin_pitch = 2.0f * (q.w * q.y - q.z * q.x);
  if (sin_pitch > 1.0f) {
    sin_pitch = 1.0f;
  } else if (sin_pitch < -1.0f) {
    sin_pitch = -1.0f;
  }
  e.pitch_deg = asinf(sin_pitch) * kRadToDeg;

  e.yaw_deg = atan2f(2.0f * (q.w * q.z + q.x * q.y),
                     1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * kRadToDeg;
  return e;
}

// Seed the quaternion from one accelerometer sample so the estimate
// starts level. The accelerometer at rest measures the gravity
// direction in the body frame. Yaw is unobservable and seeded to zero.
Quaternion seedFromAccel(float ax, float ay, float az) {
  const float n2 = ax * ax + ay * ay + az * az;
  if (!isfinite(n2) || n2 < 1.0e-6f) {
    return Quaternion{1.0f, 0.0f, 0.0f, 0.0f};
  }
  const float roll  = atan2f(ay, az);
  const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  return fromEulerRad(roll, pitch, 0.0f);
}

}  // anonymous namespace

void init() {
  s_q      = Quaternion{1.0f, 0.0f, 0.0f, 0.0f};
  s_rates  = BodyRates{0.0f, 0.0f, 0.0f};
  s_seeded = false;
}

void update(float gx_dps, float gy_dps, float gz_dps,
            float ax_g, float ay_g, float az_g,
            float dt_s) {
  s_rates = BodyRates{gx_dps, gy_dps, gz_dps};

  if (!s_seeded) {
    s_q      = seedFromAccel(ax_g, ay_g, az_g);
    s_seeded = true;
    return;
  }

  const float gx = gx_dps * kDegToRad;
  const float gy = gy_dps * kDegToRad;
  const float gz = gz_dps * kDegToRad;

  const Quaternion q = s_q;

  // Quaternion rate of change from the gyroscope: qDot = 0.5 q (x) w.
  float d_w = 0.5f * (-q.x * gx - q.y * gy - q.z * gz);
  float d_x = 0.5f * ( q.w * gx + q.y * gz - q.z * gy);
  float d_y = 0.5f * ( q.w * gy - q.x * gz + q.z * gx);
  float d_z = 0.5f * ( q.w * gz + q.x * gy - q.y * gx);

  // Gradient-descent correction toward the measured gravity direction.
  // Skipped when the accelerometer reads zero, which would not yield a
  // usable direction.
  const float a_n2 = ax_g * ax_g + ay_g * ay_g + az_g * az_g;
  if (a_n2 > 1.0e-12f) {
    const float a_inv = 1.0f / sqrtf(a_n2);
    const float ax = ax_g * a_inv;
    const float ay = ay_g * a_inv;
    const float az = az_g * a_inv;

    // Objective function: the gravity direction predicted by q minus
    // the measured gravity direction (Madgwick 2011, IMU variant).
    const float f0 = 2.0f * (q.x * q.z - q.w * q.y) - ax;
    const float f1 = 2.0f * (q.w * q.x + q.y * q.z) - ay;
    const float f2 = 2.0f * (0.5f - q.x * q.x - q.y * q.y) - az;

    // Gradient, the Jacobian transpose times the objective function.
    float g_w = -2.0f * q.y * f0 + 2.0f * q.x * f1;
    float g_x =  2.0f * q.z * f0 + 2.0f * q.w * f1 - 4.0f * q.x * f2;
    float g_y = -2.0f * q.w * f0 + 2.0f * q.z * f1 - 4.0f * q.y * f2;
    float g_z =  2.0f * q.x * f0 + 2.0f * q.y * f1;

    const float g_n2 = g_w * g_w + g_x * g_x + g_y * g_y + g_z * g_z;
    if (g_n2 > 1.0e-12f) {
      const float g_inv = MADGWICK_BETA / sqrtf(g_n2);
      d_w -= g_w * g_inv;
      d_x -= g_x * g_inv;
      d_y -= g_y * g_inv;
      d_z -= g_z * g_inv;
    }
  }

  // Integrate and renormalize.
  s_q = normalized(Quaternion{
    q.w + d_w * dt_s,
    q.x + d_x * dt_s,
    q.y + d_y * dt_s,
    q.z + d_z * dt_s,
  });
}

const Quaternion& quaternion() {
  return s_q;
}

const BodyRates& bodyRates() {
  return s_rates;
}

Euler eulerForwardFlight() {
  return toEuler(s_q);
}

Euler eulerHover() {
  // Re-reference the estimate so the nose-up hover orientation maps to
  // identity, then extract standard Euler angles from that.
  return toEuler(mul(conjugate(kHoverRef), s_q));
}

Euler errorTo(const Quaternion& reference) {
  // Rotation from the current estimate to the reference, in body axes.
  Quaternion delta = mul(conjugate(s_q), reference);

  // Choose the shortest rotation. A negative scalar part is the far
  // half of the double cover, so flip the sign.
  if (delta.w < 0.0f) {
    delta.w = -delta.w;
    delta.x = -delta.x;
    delta.y = -delta.y;
    delta.z = -delta.z;
  }

  // For a small rotation the vector part is half the rotation vector.
  Euler e;
  e.roll_deg  = 2.0f * delta.x * kRadToDeg;
  e.pitch_deg = 2.0f * delta.y * kRadToDeg;
  e.yaw_deg   = 2.0f * delta.z * kRadToDeg;
  return e;
}

Euler errorToReference(float fader,
                       float roll_setpoint_deg,
                       float pitch_setpoint_deg) {
  // Base orientation: nose-up by (fader * 90 degrees) from forward-
  // flight level. The body frame is z-up, so a nose-up pitch is a
  // negative rotation about body y. At fader 1 this is the hover
  // reference, at fader 0 it is identity.
  const Quaternion base = fromEulerRad(0.0f, fader * -90.0f * kDegToRad,
                                       0.0f);

  // Pilot setpoints applied as a body-frame offset after the base.
  const Quaternion offset = fromEulerRad(roll_setpoint_deg * kDegToRad,
                                         pitch_setpoint_deg * kDegToRad,
                                         0.0f);

  return errorTo(mul(base, offset));
}

}  // namespace cp::estimation::attitude

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// Rotational rigid-body model of the tailsitter for closed-loop SITL.
// See SimPhysics.h. Attitude dynamics only. The inertia and actuator
// effectiveness constants are plausible estimates for a 1 kg tailsitter,
// not measured values, so the model validates that the control loop is
// structurally sound and stabilizes a reasonable plant. It does not
// validate the tuning against the real airframe.

#include "src/hal/sim/SimPhysics.h"

#if BUILD_TARGET == BUILD_TARGET_HIL || BUILD_TARGET == BUILD_TARGET_SITL

#include <math.h>

namespace cp::sim {

namespace {

// Principal moments of inertia, kg m^2: roll, pitch, yaw.
constexpr float kInertia[3] = {0.012f, 0.012f, 0.020f};

// Actuator effectiveness, N m at full command.
constexpr float kElevonRoll  = 0.20f;  // differential elevon -> body x
constexpr float kElevonPitch = 0.20f;  // symmetric elevon    -> body y
constexpr float kMotorYaw    = 0.15f;  // differential thrust -> body z

// Aerodynamic rotational damping, N m per rad/s.
constexpr float kRotDamp = 0.015f;

constexpr float kRadToDeg = 57.2957795f;

// Hover attitude: body x points up, a -90 degree pitch from level.
const float kQHover[4] = {0.70710678f, 0.0f, -0.70710678f, 0.0f};

// Orientation quaternion [w, x, y, z], body to world. Body rates rad/s.
float s_q[4]     = {1.0f, 0.0f, 0.0f, 0.0f};
float s_omega[3] = {0.0f, 0.0f, 0.0f};

float s_motor[2] = {0.0f, 0.0f};  // thrust 0..1
float s_servo[2] = {0.0f, 0.0f};  // deflection -1..+1

void quatMul(const float a[4], const float b[4], float out[4]) {
  out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

void quatNormalize(float q[4]) {
  const float n =
      sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n > 1e-9f) {
    const float inv = 1.0f / n;
    q[0] *= inv;
    q[1] *= inv;
    q[2] *= inv;
    q[3] *= inv;
  }
}

float clampf(float v, float lo, float hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

}  // anonymous namespace

void physics_init() {
  // Start at hover tipped 15 degrees off vertical about the body y-axis,
  // a genuine tilt for the controller to right.
  const float half = 7.5f / kRadToDeg;
  const float q_disturb[4] = {cosf(half), 0.0f, sinf(half), 0.0f};

  quatMul(kQHover, q_disturb, s_q);
  quatNormalize(s_q);

  s_omega[0] = s_omega[1] = s_omega[2] = 0.0f;
  s_motor[0] = s_motor[1] = 0.0f;
  s_servo[0] = s_servo[1] = 0.0f;
}

void physics_set_motor(int idx, float thrust_norm) {
  if (idx >= 0 && idx < 2) {
    s_motor[idx] = clampf(thrust_norm, 0.0f, 1.0f);
  }
}

void physics_set_servo(int idx, float deflection) {
  if (idx >= 0 && idx < 2) {
    s_servo[idx] = clampf(deflection, -1.0f, 1.0f);
  }
}

void physics_step(float dt_s) {
  // Recover the control components from the two motor and two servo
  // commands. Symmetric elevon is pitch, differential elevon is roll,
  // differential thrust is yaw, per the airframe geometry.
  const float roll_cmd  = 0.5f * (s_servo[0] - s_servo[1]);
  const float pitch_cmd = 0.5f * (s_servo[0] + s_servo[1]);
  const float yaw_cmd   = 0.5f * (s_motor[0] - s_motor[1]);

  // Control torque, less aerodynamic damping.
  const float tau[3] = {
      kElevonRoll * roll_cmd   - kRotDamp * s_omega[0],
      kElevonPitch * pitch_cmd - kRotDamp * s_omega[1],
      kMotorYaw * yaw_cmd      - kRotDamp * s_omega[2],
  };

  // Euler's rigid-body equation with a diagonal inertia tensor.
  const float wx = s_omega[0];
  const float wy = s_omega[1];
  const float wz = s_omega[2];
  const float wd[3] = {
      (tau[0] - wy * wz * (kInertia[2] - kInertia[1])) / kInertia[0],
      (tau[1] - wz * wx * (kInertia[0] - kInertia[2])) / kInertia[1],
      (tau[2] - wx * wy * (kInertia[1] - kInertia[0])) / kInertia[2],
  };
  s_omega[0] += wd[0] * dt_s;
  s_omega[1] += wd[1] * dt_s;
  s_omega[2] += wd[2] * dt_s;

  // Integrate the orientation: q_dot = 0.5 * q (x) [0, omega].
  const float qd[4] = {
      0.5f * (-s_q[1] * s_omega[0] - s_q[2] * s_omega[1] -
              s_q[3] * s_omega[2]),
      0.5f * (s_q[0] * s_omega[0] + s_q[2] * s_omega[2] -
              s_q[3] * s_omega[1]),
      0.5f * (s_q[0] * s_omega[1] - s_q[1] * s_omega[2] +
              s_q[3] * s_omega[0]),
      0.5f * (s_q[0] * s_omega[2] + s_q[1] * s_omega[1] -
              s_q[2] * s_omega[0]),
  };
  s_q[0] += qd[0] * dt_s;
  s_q[1] += qd[1] * dt_s;
  s_q[2] += qd[2] * dt_s;
  s_q[3] += qd[3] * dt_s;
  quatNormalize(s_q);
}

void physics_gyro_dps(float& gx, float& gy, float& gz) {
  gx = s_omega[0] * kRadToDeg;
  gy = s_omega[1] * kRadToDeg;
  gz = s_omega[2] * kRadToDeg;
}

void physics_accel_g(float& ax, float& ay, float& az) {
  // The gravity-up world vector (0, 0, 1) expressed in body axes: the
  // third row of the body-to-world rotation matrix.
  const float w = s_q[0];
  const float x = s_q[1];
  const float y = s_q[2];
  const float z = s_q[3];
  ax = 2.0f * (x * z - w * y);
  ay = 2.0f * (y * z + w * x);
  az = 1.0f - 2.0f * (x * x + y * y);
}

float physics_tilt_deg() {
  // Angle of the body x-axis from world-up. Zero in true hover. This is
  // the "is it falling over" metric, independent of heading.
  float c = 2.0f * (s_q[1] * s_q[3] - s_q[0] * s_q[2]);
  if (c > 1.0f) {
    c = 1.0f;
  }
  if (c < -1.0f) {
    c = -1.0f;
  }
  return acosf(c) * kRadToDeg;
}

}  // namespace cp::sim

#endif  // SITL or HIL

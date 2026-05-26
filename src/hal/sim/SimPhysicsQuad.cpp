// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// Rotational rigid-body model of an X-quad for closed-loop SITL. See
// SimPhysics.h. Attitude dynamics only. The inertia and motor-effectiveness
// constants are plausible estimates for a small (sub-kilogram) quad, not
// measured values, so the model validates that the control loop is
// structurally sound and stabilizes a reasonable plant, and in particular
// that the X-mix signs close the loop as negative feedback. It does not
// validate the tuning against a real airframe.
//
// The torque map below is the true physics from the motor geometry (FLU body
// frame, x out the nose), independent of the mixer: each upward motor thrust
// makes a body moment r x F about the center.
//   roll  (about +x): left motors minus right motors
//   pitch (about +y): rear motors minus front motors
//   yaw   (about +z): one diagonal pair minus the other (prop reaction)
// If the mixer signs disagree with this, an axis diverges in sim, which is
// exactly the check this model exists for.

#include "src/hal/sim/SimPhysics.h"

#if (BUILD_TARGET == BUILD_TARGET_HIL || BUILD_TARGET == BUILD_TARGET_SITL) && \
    AIRFRAME == AIRFRAME_QUAD_X

#include <math.h>

#include "src/airframes/Airframe.h"

namespace cp::sim {

namespace {

// Principal moments of inertia, kg m^2: roll, pitch, yaw.
constexpr float kInertia[3] = {0.010f, 0.010f, 0.018f};

// Motor effectiveness, N m at a full unit of differential thrust.
constexpr float kRoll  = 0.25f;
constexpr float kPitch = 0.25f;
constexpr float kYaw   = 0.06f;

// Aerodynamic and motor-drag rotational damping, N m per rad/s.
constexpr float kRotDamp = 0.010f;

constexpr float kRadToDeg = 57.2957795f;
constexpr float kDegToRad = 1.0f / kRadToDeg;

// Orientation quaternion [w, x, y, z], body to world. Body rates rad/s.
// The hover reference is level, so the reference quaternion is identity.
float s_q[4]     = {1.0f, 0.0f, 0.0f, 0.0f};
float s_omega[3] = {0.0f, 0.0f, 0.0f};

float s_motor[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // thrust 0..1, per motor index

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
  // Start level but upset 15 degrees in roll and 10 in pitch, a genuine
  // disturbance for the controller to right.
  const float r = 15.0f * kDegToRad;
  const float p = 10.0f * kDegToRad;
  const float q_roll[4]  = {cosf(r * 0.5f), sinf(r * 0.5f), 0.0f, 0.0f};
  const float q_pitch[4] = {cosf(p * 0.5f), 0.0f, sinf(p * 0.5f), 0.0f};
  quatMul(q_roll, q_pitch, s_q);
  quatNormalize(s_q);

  s_omega[0] = s_omega[1] = s_omega[2] = 0.0f;
  for (int i = 0; i < 4; ++i) {
    s_motor[i] = 0.0f;
  }
}

void physics_set_motor(int idx, float thrust_norm) {
  if (idx >= 0 && idx < 4) {
    s_motor[idx] = clampf(thrust_norm, 0.0f, 1.0f);
  }
}

void physics_set_servo(int /*idx*/, float /*deflection*/) {
  // A quad has no servos.
}

void physics_step(float dt_s) {
  // Body moments from the four motor thrusts, by the geometry above.
  const float fr = s_motor[cp::airframes::MOTOR_FRONT_RIGHT];
  const float fl = s_motor[cp::airframes::MOTOR_FRONT_LEFT];
  const float rr = s_motor[cp::airframes::MOTOR_REAR_RIGHT];
  const float rl = s_motor[cp::airframes::MOTOR_REAR_LEFT];

  const float roll_moment  = (fl + rl) - (fr + rr);  // left minus right
  const float pitch_moment = (rr + rl) - (fr + fl);  // rear minus front
  const float yaw_moment   = (fr + rl) - (fl + rr);  // diagonal difference

  const float tau[3] = {
      kRoll * roll_moment   - kRotDamp * s_omega[0],
      kPitch * pitch_moment - kRotDamp * s_omega[1],
      kYaw * yaw_moment     - kRotDamp * s_omega[2],
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
  // The gravity-up world vector (0, 0, 1) expressed in body axes.
  const float w = s_q[0];
  const float x = s_q[1];
  const float y = s_q[2];
  const float z = s_q[3];
  ax = 2.0f * (x * z - w * y);
  ay = 2.0f * (y * z + w * x);
  az = 1.0f - 2.0f * (x * x + y * y);
}

float physics_roll_deg() {
  // Level reference, so the error quaternion is s_q itself (shortest sense).
  const float s = (s_q[0] < 0.0f) ? -1.0f : 1.0f;
  return 2.0f * s * s_q[1] * kRadToDeg;
}

float physics_pitch_deg() {
  const float s = (s_q[0] < 0.0f) ? -1.0f : 1.0f;
  return 2.0f * s * s_q[2] * kRadToDeg;
}

}  // namespace cp::sim

#endif  // SITL or HIL, quad

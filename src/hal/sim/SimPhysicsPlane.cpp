// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// Fixed-wing rotational model for closed-loop SITL. See SimPhysics.h.
// Models attitude dynamics about a level cruise: control-surface moments
// scaled by a constant cruise dynamic pressure, plus the airframe's
// natural pitch static stability and aerodynamic damping. Translation,
// airspeed, and altitude are not modelled.
//
// Roll has no natural restoring moment, only damping: a banked wing
// stays banked. Levelling it is exactly the wing-leveler's job, so that
// is what this model puts under test.
//
// The inertia and effectiveness constants are plausible estimates for a
// DHC-4 Caribou class twin (roughly 3 kg, 1.85 m span), not measured
// values. The model checks that the plane control law is structurally
// sound and stabilizes a reasonable plant. It does not validate the
// tuning against the real aircraft.

#include "src/hal/sim/SimPhysics.h"

#if (BUILD_TARGET == BUILD_TARGET_HIL || BUILD_TARGET == BUILD_TARGET_SITL) && \
    AIRFRAME != AIRFRAME_TAILSITTER_BICOPTER && AIRFRAME != AIRFRAME_QUAD_X

#include <math.h>

namespace cp::sim {

namespace {

// Principal moments of inertia, kg m^2: roll, pitch, yaw.
constexpr float kInertia[3] = {0.20f, 0.25f, 0.42f};

// Control effectiveness, N m at full surface command, cruise.
constexpr float kAileron  = 0.80f;
constexpr float kElevator = 1.00f;
constexpr float kRudder   = 0.60f;

// Natural pitch static stability, N m per unit sin(pitch). Restores the
// nose toward level. Roll has no equivalent term by design.
constexpr float kPitchStiffness = 4.0f;

// Aerodynamic rotational damping, N m per rad/s.
constexpr float kRollDamp  = 0.10f;
constexpr float kPitchDamp = 0.15f;
constexpr float kYawDamp   = 0.15f;

constexpr float kRadToDeg = 57.2957795f;

// Orientation quaternion [w, x, y, z], body to world. The reference
// attitude is level cruise, the identity quaternion. Body rates rad/s.
float s_q[4]     = {1.0f, 0.0f, 0.0f, 0.0f};
float s_omega[3] = {0.0f, 0.0f, 0.0f};

float s_motor[2] = {0.0f, 0.0f};  // thrust 0..1
float s_servo[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // deflection -1..+1

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

// Steady body-axis disturbance torques for SIM_SCENARIO_WIND. Zero for
// the other scenarios so the compiler folds them away.
#if SIM_SCENARIO == SIM_SCENARIO_WIND
constexpr float kWindRollTorque  = 0.020f;
constexpr float kWindPitchTorque = 0.015f;
#else
constexpr float kWindRollTorque  = 0.0f;
constexpr float kWindPitchTorque = 0.0f;
#endif

// Build the initial orientation quaternion from a roll tilt then a
// pitch tilt, both in degrees. Used by physics_init per SIM_SCENARIO.
void setInitialAttitude(float roll_deg, float pitch_deg) {
  const float roll_half  = (roll_deg  * 0.5f) / kRadToDeg;
  const float pitch_half = (pitch_deg * 0.5f) / kRadToDeg;
  const float q_roll[4]  = {cosf(roll_half), sinf(roll_half), 0.0f, 0.0f};
  const float q_pitch[4] = {cosf(pitch_half), 0.0f, sinf(pitch_half), 0.0f};
  quatMul(q_roll, q_pitch, s_q);
  quatNormalize(s_q);
}

}  // anonymous namespace

void physics_init() {
  // Pick the initial attitude per SIM_SCENARIO. The default upset
  // exercises the wing-leveler and the pitch-hold at once.
#if SIM_SCENARIO == SIM_SCENARIO_LEVEL
  setInitialAttitude(0.0f, 0.0f);
#elif SIM_SCENARIO == SIM_SCENARIO_LARGE_UPSET
  setInitialAttitude(45.0f, 30.0f);
#else  // DEFAULT or WIND
  setInitialAttitude(20.0f, 12.0f);
#endif

  s_omega[0] = s_omega[1] = s_omega[2] = 0.0f;
  s_motor[0] = s_motor[1] = 0.0f;
  for (int i = 0; i < 4; ++i) {
    s_servo[i] = 0.0f;
  }
}

void physics_set_motor(int idx, float thrust_norm) {
  if (idx >= 0 && idx < 2) {
    s_motor[idx] = clampf(thrust_norm, 0.0f, 1.0f);
  }
}

void physics_set_servo(int idx, float deflection) {
  if (idx >= 0 && idx < 4) {
    s_servo[idx] = clampf(deflection, -1.0f, 1.0f);
  }
}

void physics_step(float dt_s) {
  // Recover the surface commands from the servo deflections.
#if AIRFRAME == AIRFRAME_PLANE_FLYING_WING
  // Two elevons on servos 0 and 1. Roll is the differential, pitch is the
  // common mode, no rudder. Signs match the conventional case so the same
  // aerodynamic gains apply.
  const float aileron  = 0.5f * (s_servo[0] - s_servo[1]);
  const float elevator = 0.5f * (s_servo[0] + s_servo[1]);
  const float rudder   = 0.0f;
#else
  // Conventional layout. Servo indices per Airframe.h: 0 and 1 are the left
  // and right ailerons (differential is roll), 2 is the elevator, 3 is the
  // rudder.
  const float aileron  = 0.5f * (s_servo[0] - s_servo[1]);
  const float elevator = s_servo[2];
  const float rudder   = s_servo[3];
#endif

  // sin(pitch): the world-up z-component of the body x-axis.
  const float sin_pitch =
      2.0f * (s_q[0] * s_q[2] - s_q[1] * s_q[3]);

  // Body torque: control, the steady wind disturbance (zero outside the
  // WIND scenario), the natural pitch restoring moment, damping.
  const float tau[3] = {
      kAileron * aileron + kWindRollTorque - kRollDamp * s_omega[0],
      kElevator * elevator + kWindPitchTorque -
          kPitchStiffness * sin_pitch - kPitchDamp * s_omega[1],
      kRudder * rudder - kYawDamp * s_omega[2],
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

float physics_roll_deg() {
  const float w = s_q[0];
  const float x = s_q[1];
  const float y = s_q[2];
  const float z = s_q[3];
  return atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) *
         kRadToDeg;
}

float physics_pitch_deg() {
  const float w = s_q[0];
  const float x = s_q[1];
  const float y = s_q[2];
  const float z = s_q[3];
  return asinf(clampf(2.0f * (w * y - x * z), -1.0f, 1.0f)) * kRadToDeg;
}

}  // namespace cp::sim

#endif  // SITL or HIL, plane

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/airframes/Airframe.h"

// Quadcopter X mixer. One airframe compiles in at a time; this file is the
// mixer for AIRFRAME_QUAD_X.
//
// Geometry. Four motors at the arm tips, FLU body frame (x out the nose, y
// out the left, z up), the same frame the estimator uses. Diagonal pairs
// spin opposite ways so reaction torque cancels in a hover.
//
//        front
//      FL     FR
//        \   /
//         \ /          x (nose) up the page
//         / \          y (left)  to the left
//      RL     RR       z (up)    out of the page
//        rear
//
// Effector to body moment:
//   common thrust        -> lift along body z
//   left/right differential thrust  -> roll  (moment about x)
//   front/rear differential thrust  -> pitch (moment about y)
//   diagonal (CW vs CCW) differential -> yaw (moment about z)
//
// The per-axis signs below are validated in the SITL quad model: from a roll
// and pitch upset the closed loop converges to level with body rates settling
// to zero. A wrong sign flips the craft on arm, so this still needs a
// tethered-bench check before flight (the sim is a plausible plant, not the
// real airframe). There are no regime gains and no fader; a quad has one
// flight regime.

#if AIRFRAME == AIRFRAME_QUAD_X

namespace cp::airframes {

namespace {

Output s_out = {};

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

void init() {
  for (uint8_t i = 0; i < N_MOTORS; ++i) {
    s_out.motor[i] = 0.0f;
  }
}

void update(float throttle,
            float roll_pid, float pitch_pid, float yaw_pid,
            float roll_pt, float pitch_pt, float yaw_pt,
            float fader) {
  // A quad has one regime: the stick-feedforward and fader arguments the
  // shared mixer signature carries for the tailsitter are unused here.
  (void)roll_pt;
  (void)pitch_pt;
  (void)yaw_pt;
  (void)fader;

  const float r = roll_pid;
  const float p = pitch_pid;
  const float y = yaw_pid;

  // Standard X-mix. Each motor is common throttle plus its share of the
  // roll, pitch, and yaw demands. Simple clamping for now: at low throttle a
  // large correction can hit the 0 floor and lose authority on that motor.
  // Airmode-style desaturation is a later refinement.
  s_out.motor[MOTOR_FRONT_RIGHT] = clampf(throttle - r - p + y, 0.0f, 1.0f);
  s_out.motor[MOTOR_FRONT_LEFT]  = clampf(throttle + r - p - y, 0.0f, 1.0f);
  s_out.motor[MOTOR_REAR_RIGHT]  = clampf(throttle - r + p - y, 0.0f, 1.0f);
  s_out.motor[MOTOR_REAR_LEFT]   = clampf(throttle + r + p + y, 0.0f, 1.0f);
}

const Output& output() {
  return s_out;
}

}  // namespace cp::airframes

#endif  // AIRFRAME == AIRFRAME_QUAD_X

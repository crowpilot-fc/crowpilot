// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "airframes/Airframe.h"

// Tailsitter bicopter mixer, per TAILSITTER_CONTROL_SPEC.md section 9.
// One airframe compiles in at a time; this file is the mixer for the
// tailsitter bicopter. The plane airframes have their own mixer files.
//
// Geometry. Two motors on the wing, left and right of centerline,
// separated along the body y-axis, thrust along body x. One elevon
// behind each motor, in its propeller wash. Body frame: x out the
// nose, z up, y completing the right-handed triad, the same frame the
// attitude estimator uses.
//
// Effector to body moment, fixed by that geometry and so the same in
// both regimes:
//   common thrust         -> body x force (lift in hover, thrust in
//                            forward flight)
//   differential thrust   -> body z moment (yaw)
//   symmetric elevon       -> body y moment (pitch)
//   differential elevon    -> body x moment (roll)
//
// The regime difference is in authority, not geometry: the elevons
// work in propeller wash in hover and in free airspeed in forward
// flight. That authority difference is handled by the controller's
// regime-scheduled gains. The one allocation term that genuinely
// changes with regime is a direct-stick feedforward added to the
// elevons in forward flight, blended out toward hover by the fader.

#if AIRFRAME == AIRFRAME_TAILSITTER_BICOPTER

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
  s_out.motor[MOTOR_RIGHT] = 0.0f;
  s_out.motor[MOTOR_LEFT]  = 0.0f;
  s_out.servo[SERVO_RIGHT] = ELEVON_RIGHT_TRIM;
  s_out.servo[SERVO_LEFT]  = ELEVON_LEFT_TRIM;
}

void update(float throttle,
            float roll_pid, float pitch_pid, float yaw_pid,
            float roll_pt, float pitch_pt, float yaw_pt,
            float fader) {
  (void)yaw_pt;  // Yaw is a rate loop; it takes no stick feedforward.

  // Roll and pitch demand, blended between the two regime allocations
  // by the fader (1.0 hover, 0.0 forward). The hover allocation is pure
  // PID stabilization. The forward allocation adds a direct-stick
  // feedforward so the elevons respond crisply at airspeed.
  const float forward = 1.0f - fader;

  const float roll_forward  = roll_pid  + roll_pt  * ELEVON_FF_GAIN;
  const float pitch_forward = pitch_pid + pitch_pt * ELEVON_FF_GAIN;

  const float roll_cmd  = fader * roll_pid  + forward * roll_forward;
  const float pitch_cmd = fader * pitch_pid + forward * pitch_forward;

  // Motors: common throttle plus differential thrust for yaw.
  const float yaw_diff = yaw_pid * MOTOR_YAW_GAIN;
  s_out.motor[MOTOR_RIGHT] = clampf(throttle + yaw_diff, 0.0f, 1.0f);
  s_out.motor[MOTOR_LEFT]  = clampf(throttle - yaw_diff, 0.0f, 1.0f);

  // Elevons: symmetric deflection for pitch, differential for roll,
  // about each elevon's mechanical trim. Deflection signs are verified
  // on the bench against the actual servo linkages.
  const float pitch_deflect = pitch_cmd * ELEVON_PITCH_GAIN;
  const float roll_deflect  = roll_cmd  * ELEVON_ROLL_GAIN;
  s_out.servo[SERVO_RIGHT] =
      clampf(ELEVON_RIGHT_TRIM + pitch_deflect + roll_deflect, 0.0f, 1.0f);
  s_out.servo[SERVO_LEFT] =
      clampf(ELEVON_LEFT_TRIM + pitch_deflect - roll_deflect, 0.0f, 1.0f);
}

const Output& output() {
  return s_out;
}

}  // namespace cp::airframes

#endif  // AIRFRAME == AIRFRAME_TAILSITTER_BICOPTER

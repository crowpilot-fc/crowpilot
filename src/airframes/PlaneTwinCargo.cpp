// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "airframes/PlaneTwinCargo.h"

#include "Config.h"

#if AIRFRAME == AIRFRAME_PLANE_TWIN_CARGO

#if !ENABLE_PLANE_STAB
  #error "AIRFRAME_PLANE_TWIN_CARGO requires ENABLE_PLANE_STAB = 1 in Config.h. The plane mixer is driven by the plane stabilizer."
#endif

#include "control/PlaneStab.h"

namespace cp::airframes {

namespace {

inline float clamp01(float x) {
  if (x > 1.0f) return 1.0f;
  if (x < 0.0f) return 0.0f;
  return x;
}

}  // anonymous namespace

}  // namespace cp::airframes

namespace cp::airframes::plane_twin_cargo {

void mix(float throttle, float roll, float pitch, float yaw,
         cp::airframes::Output& out) {
  using namespace cp::airframes;

  // Throttle drives both motors. Optional differential thrust adds a yaw
  // moment by splitting the two motors around the throttle command.
  float motor_right = throttle;
  float motor_left  = throttle;
#if ENABLE_DIFF_THRUST_YAW
  motor_right += DIFF_THRUST_GAIN * yaw;
  motor_left  -= DIFF_THRUST_GAIN * yaw;
#endif
  out.motor[MOTOR_RIGHT] = clamp01(motor_right);
  out.motor[MOTOR_LEFT]  = clamp01(motor_left);

  // Control surfaces. 0.5 is geometric center; the travel ratios scale
  // the normalized command into the [0, 1] servo range. The two ailerons
  // move in opposite directions for roll.
  out.servo[SERVO_AILERON_LEFT]  = clamp01(0.5f + AILERON_TRAVEL  * roll);
  out.servo[SERVO_AILERON_RIGHT] = clamp01(0.5f - AILERON_TRAVEL  * roll);
  out.servo[SERVO_ELEVATOR]      = clamp01(0.5f + ELEVATOR_TRAVEL * pitch);
  out.servo[SERVO_RUDDER]        = clamp01(0.5f + RUDDER_TRAVEL   * yaw);
}

}  // namespace cp::airframes::plane_twin_cargo

namespace cp::airframes {

namespace {

Output s_output = {};

}  // anonymous namespace

void init() {
  s_output.motor[MOTOR_RIGHT] = 0.0f;
  s_output.motor[MOTOR_LEFT]  = 0.0f;
  for (uint8_t i = 0; i < N_SERVOS; ++i) {
    s_output.servo[i] = 0.5f;  // surfaces centered
  }
}

void update(float /*throttle*/,
            float /*roll_pid*/, float /*pitch_pid*/, float /*yaw_pid*/,
            float /*roll_pt*/,  float /*pitch_pt*/,  float /*yaw_pt*/,
            float /*fader*/) {
  // The plane mixer is driven by the plane stabilizer, not the tailsitter
  // angle PID, so the shared facade arguments are unused. The stabilizer
  // has already folded in pilot passthrough and altitude hold.
  const auto& stab = cp::control::plane_stab::output();
  cp::airframes::plane_twin_cargo::mix(
      stab.throttle, stab.roll, stab.pitch, stab.yaw, s_output);
}

const Output& output() {
  return s_output;
}

}  // namespace cp::airframes

#endif  // AIRFRAME == AIRFRAME_PLANE_TWIN_CARGO

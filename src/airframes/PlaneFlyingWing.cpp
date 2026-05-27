// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/airframes/PlaneFlyingWing.h"

#include "src/Config.h"

#if AIRFRAME == AIRFRAME_PLANE_FLYING_WING

#if !ENABLE_PLANE_STAB
  #error "AIRFRAME_PLANE_FLYING_WING requires ENABLE_PLANE_STAB = 1 in Config.h. The elevon mixer is driven by the plane stabilizer."
#endif

#include "src/control/PlaneStab.h"

namespace cp::airframes {

namespace {

inline float clamp01(float x) {
  if (x > 1.0f) return 1.0f;
  if (x < 0.0f) return 0.0f;
  return x;
}

}  // anonymous namespace

}  // namespace cp::airframes

namespace cp::airframes::plane_flying_wing {

void mix(float throttle, float roll, float pitch, float /*yaw*/,
         cp::airframes::Output& out) {
  using namespace cp::airframes;

  // One engine. Both motor pins carry the same signal; a single-engine build
  // connects one ESC and leaves the other pin idle.
  const float motor = clamp01(throttle);
  out.motor[MOTOR_RIGHT] = motor;
  out.motor[MOTOR_LEFT]  = motor;

  // Elevons. 0.5 is geometric center. Pitch is the common mode (both surfaces
  // deflect the same way); roll is the differential. The roll sign matches the
  // conventional plane's ailerons: a positive roll command raises the left
  // surface command and lowers the right, so the plane stabilizer's roll
  // output drives the same direction it does on an aileron plane. A pure
  // flying wing has no rudder, so the yaw command is unused.
  const float pitch_deflect = ELEVATOR_TRAVEL * pitch;
  const float roll_deflect  = AILERON_TRAVEL  * roll;
  out.servo[SERVO_ELEVON_LEFT]  =
      clamp01(0.5f + pitch_deflect + roll_deflect);
  out.servo[SERVO_ELEVON_RIGHT] =
      clamp01(0.5f + pitch_deflect - roll_deflect);
}

}  // namespace cp::airframes::plane_flying_wing

namespace cp::airframes {

namespace {

Output s_output = {};

}  // anonymous namespace

void init() {
  s_output.motor[MOTOR_RIGHT] = 0.0f;
  s_output.motor[MOTOR_LEFT]  = 0.0f;
  for (uint8_t i = 0; i < N_SERVOS; ++i) {
    s_output.servo[i] = 0.5f;  // elevons centered
  }
}

void update(float /*throttle*/,
            float /*roll_pid*/, float /*pitch_pid*/, float /*yaw_pid*/,
            float /*roll_pt*/,  float /*pitch_pt*/,  float /*yaw_pt*/,
            float /*fader*/) {
  // The mixer is driven by the plane stabilizer, not the tailsitter angle PID,
  // so the shared facade arguments are unused.
  const auto& stab = cp::control::plane_stab::output();
  cp::airframes::plane_flying_wing::mix(
      stab.throttle, stab.roll, stab.pitch, stab.yaw, s_output);
}

const Output& output() {
  return s_output;
}

}  // namespace cp::airframes

#endif  // AIRFRAME == AIRFRAME_PLANE_FLYING_WING

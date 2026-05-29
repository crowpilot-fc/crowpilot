// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/airframes/PlaneSingle.h"

#include "src/Config.h"

#if AIRFRAME == AIRFRAME_PLANE_SINGLE

#if !ENABLE_PLANE_STAB
  #error "AIRFRAME_PLANE_SINGLE requires ENABLE_PLANE_STAB = 1 in Config.h. The plane mixer is driven by the plane stabilizer."
#endif

#include "src/airframes/WingMix.h"
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

namespace cp::airframes::plane_single {

void mix(float throttle, float roll, float pitch, float yaw,
         float flap, float brake, cp::airframes::Output& out) {
  using namespace cp::airframes;

  // One engine. The firmware drives both motor pins with the same signal;
  // a single-engine build connects one ESC and leaves the other pin idle.
  const float motor = clamp01(throttle);
  out.motor[MOTOR_RIGHT] = motor;
  out.motor[MOTOR_LEFT]  = motor;

  // Ailerons via the shared wing mix: roll, plus differential, flaperon droop,
  // and airbrake reflex. Elevator and rudder are the plain stabilized surfaces.
  wing::aileronPair(roll, flap, brake,
                    out.servo[SERVO_AILERON_LEFT],
                    out.servo[SERVO_AILERON_RIGHT]);
  out.servo[SERVO_ELEVATOR] = clamp01(0.5f + ELEVATOR_TRAVEL * pitch);
  out.servo[SERVO_RUDDER]   = clamp01(0.5f + RUDDER_TRAVEL   * yaw);
}

}  // namespace cp::airframes::plane_single

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
  // angle PID, so the shared facade arguments are unused.
  const auto& stab = cp::control::plane_stab::output();
  cp::airframes::plane_single::mix(
      stab.throttle, stab.roll, stab.pitch, stab.yaw,
      stab.flap, stab.brake, s_output);
}

const Output& output() {
  return s_output;
}

}  // namespace cp::airframes

#endif  // AIRFRAME == AIRFRAME_PLANE_SINGLE

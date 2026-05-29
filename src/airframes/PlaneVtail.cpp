// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/airframes/PlaneVtail.h"

#include "src/Config.h"

#if AIRFRAME == AIRFRAME_PLANE_VTAIL

#if !ENABLE_PLANE_STAB
  #error "AIRFRAME_PLANE_VTAIL requires ENABLE_PLANE_STAB = 1 in Config.h. The V-tail mixer is driven by the plane stabilizer."
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

namespace cp::airframes::plane_vtail {

void mix(float throttle, float roll, float pitch, float yaw,
         float flap, float brake, cp::airframes::Output& out) {
  using namespace cp::airframes;

  // One engine. Both motor pins carry the same signal; connect one ESC.
  const float motor = clamp01(throttle);
  out.motor[MOTOR_RIGHT] = motor;
  out.motor[MOTOR_LEFT]  = motor;

  // Ailerons via the shared wing mix (differential, flaperon, airbrake).
  wing::aileronPair(roll, flap, brake,
                    out.servo[SERVO_AILERON_LEFT],
                    out.servo[SERVO_AILERON_RIGHT]);

  // Ruddervators: pitch is the common mode (both deflect the same way), yaw is
  // the differential. Signs match the conventional elevator and rudder so the
  // plane stabilizer output drives the same directions as a tailed plane.
  const float pitch_deflect = ELEVATOR_TRAVEL * pitch;
  const float yaw_deflect   = RUDDER_TRAVEL   * yaw;
  out.servo[SERVO_VTAIL_LEFT]  =
      clamp01(0.5f + pitch_deflect + yaw_deflect);
  out.servo[SERVO_VTAIL_RIGHT] =
      clamp01(0.5f + pitch_deflect - yaw_deflect);
}

}  // namespace cp::airframes::plane_vtail

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
  const auto& stab = cp::control::plane_stab::output();
  cp::airframes::plane_vtail::mix(
      stab.throttle, stab.roll, stab.pitch, stab.yaw,
      stab.flap, stab.brake, s_output);
}

const Output& output() {
  return s_output;
}

}  // namespace cp::airframes

#endif  // AIRFRAME == AIRFRAME_PLANE_VTAIL

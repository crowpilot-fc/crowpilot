// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/airframes/PlaneGeneric.h"

#include "src/Config.h"

#if AIRFRAME == AIRFRAME_PLANE_GENERIC

#if !ENABLE_PLANE_STAB
  #error "AIRFRAME_PLANE_GENERIC requires ENABLE_PLANE_STAB = 1 in Config.h. The generic plane mixer is driven by the plane stabilizer."
#endif

#include "src/airframes/WingMix.h"
#include "src/control/PlaneStab.h"
#include "src/params/Params.h"

namespace cp::airframes {

namespace {

inline float clamp01(float x) {
  if (x > 1.0f) return 1.0f;
  if (x < 0.0f) return 0.0f;
  return x;
}

}  // anonymous namespace

}  // namespace cp::airframes

namespace cp::airframes::plane_generic {

void mix(float throttle, float roll, float pitch, float yaw,
         float flap, float brake, cp::airframes::Output& out) {
  using namespace cp::airframes;
  namespace pr = cp::params;

  // Snap params to integers. Defaults guarantee valid values; clamps elsewhere
  // protect against out-of-range writes.
  const uint8_t motors_count  = static_cast<uint8_t>(pr::get(pr::PLANE_MOTORS_COUNT)  + 0.5f);
  const uint8_t tail_style    = static_cast<uint8_t>(pr::get(pr::PLANE_TAIL_STYLE)    + 0.5f);
  const uint8_t aileron_count = static_cast<uint8_t>(pr::get(pr::PLANE_AILERON_COUNT) + 0.5f);

  // ---- Motors ----
  // Single-engine builds drive motor[0]; motor[1] stays at 0 (ESC2 header
  // empty). Twin-engine builds mirror the throttle into both motors. The
  // pilot's transmitter handles any differential thrust mixing for yaw on
  // the radio side, so the firmware does not apply diff-thrust here.
  out.motor[MOTOR_RIGHT] = clamp01(throttle);
  out.motor[MOTOR_LEFT]  = (motors_count >= 2) ? clamp01(throttle) : 0.0f;

  // ---- Surfaces ----
  if (tail_style == PLANE_TAIL_NONE) {
    // Flying wing path. Tail = none forces aileron_count = 2 (elevons), per
    // the Companion validation rules. The two front surfaces carry both
    // pitch (common mode) and roll (differential), the way the existing
    // PLANE_FLYING_WING mixer does it.
    out.servo[0] = clamp01(0.5f + ELEVATOR_TRAVEL * pitch + AILERON_TRAVEL * roll);
    out.servo[1] = clamp01(0.5f + ELEVATOR_TRAVEL * pitch - AILERON_TRAVEL * roll);
    out.servo[2] = 0.5f;  // tail slot unused
    out.servo[3] = 0.5f;
    return;
  }

  // ---- Conventional tail builds (Traditional or V-tail) ----

  // Aileron section.
  if (aileron_count >= 2) {
    // Independent left/right ailerons. Use the shared wing mix so flaperon,
    // crow, and differential ailerons all keep working.
    wing::aileronPair(roll, flap, brake, out.servo[0], out.servo[1]);
  } else if (aileron_count == 1) {
    // One aileron servo drives roll directly. Pilot Y-cabled both surfaces
    // to one servo, or has one servo driving both ailerons via pushrods.
    out.servo[0] = clamp01(0.5f + AILERON_TRAVEL * roll);
    out.servo[1] = 0.5f;
  } else {
    // No ailerons (3-channel rudder-elevator trainer).
    out.servo[0] = 0.5f;
    out.servo[1] = 0.5f;
  }

  // Tail section.
  if (tail_style == PLANE_TAIL_VTAIL) {
    // Ruddervators. Pitch is the common mode; yaw is the differential. Signs
    // match the existing PLANE_VTAIL mixer so the plane stabilizer drives
    // the same directions as on a conventional tail.
    const float pitch_deflect = ELEVATOR_TRAVEL * pitch;
    const float yaw_deflect   = RUDDER_TRAVEL   * yaw;
    out.servo[2] = clamp01(0.5f + pitch_deflect + yaw_deflect);
    out.servo[3] = clamp01(0.5f + pitch_deflect - yaw_deflect);
  } else {
    // Traditional tail: elevator on servo[2], rudder on servo[3].
    out.servo[2] = clamp01(0.5f + ELEVATOR_TRAVEL * pitch);
    out.servo[3] = clamp01(0.5f + RUDDER_TRAVEL   * yaw);
  }
}

}  // namespace cp::airframes::plane_generic

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
  // The generic plane mixer is driven by the plane stabilizer, not the
  // tailsitter angle PID, so the shared facade arguments are unused. The
  // stabilizer has already folded in pilot passthrough, stab authority
  // blending, and altitude hold.
  const auto& stab = cp::control::plane_stab::output();
  cp::airframes::plane_generic::mix(
      stab.throttle, stab.roll, stab.pitch, stab.yaw,
      stab.flap, stab.brake, s_output);
}

const Output& output() {
  return s_output;
}

}  // namespace cp::airframes

#endif  // AIRFRAME == AIRFRAME_PLANE_GENERIC

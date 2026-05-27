// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/control/LaunchAssist.h"

#include "src/Config.h"

#if ENABLE_LAUNCH_ASSIST

#include <math.h>

namespace cp::control::launch {

namespace {

// IDLE    not armed, or armed with the throttle above idle. No launch.
// WAIT    armed, throttle at idle, watching for the throw.
// ACTIVE  throw detected, commanding the climb-out for the launch window.
// DONE    launch finished. No re-trigger until the next arm cycle.
enum class State : uint8_t { IDLE, WAIT, ACTIVE, DONE };

State s_state   = State::IDLE;
float s_timer_s = 0.0f;

}  // anonymous namespace

void init() {
  s_state   = State::IDLE;
  s_timer_s = 0.0f;
}

Override update(bool armed, float ax_g, float throttle,
                float roll_stick, float pitch_stick, float dt_s) {
  Override ov{false, 0.0f, 0.0f, 0.0f};

  // Disarm resets the whole sequence so the next arm cycle can launch again.
  if (!armed) {
    s_state   = State::IDLE;
    s_timer_s = 0.0f;
    return ov;
  }

  // Transitions first, so a throw detected this tick activates the override
  // on the same tick rather than the next one.
  switch (s_state) {
    case State::IDLE:
      // Arm the detector once the pilot holds the throttle at idle.
      if (throttle <= LAUNCH_IDLE_MAX) {
        s_state = State::WAIT;
      }
      break;

    case State::WAIT:
      // A forward acceleration spike is the throw.
      if (ax_g >= LAUNCH_DETECT_ACCEL_G) {
        s_state   = State::ACTIVE;
        s_timer_s = 0.0f;
      }
      break;

    case State::ACTIVE:
      s_timer_s += dt_s;
      // End the climb-out on timeout or when the pilot moves a stick.
      if (s_timer_s * 1000.0f >= static_cast<float>(LAUNCH_DURATION_MS) ||
          fabsf(roll_stick) > LAUNCH_STICK_EXIT ||
          fabsf(pitch_stick) > LAUNCH_STICK_EXIT) {
        s_state = State::DONE;
      }
      break;

    case State::DONE:
      break;
  }

  // Output: the override is on whenever the climb-out is active.
  if (s_state == State::ACTIVE) {
    ov.active    = true;
    ov.throttle  = LAUNCH_THROTTLE;
    ov.roll_deg  = 0.0f;
    ov.pitch_deg = LAUNCH_PITCH_DEG;
  }

  return ov;
}

}  // namespace cp::control::launch

#endif  // ENABLE_LAUNCH_ASSIST

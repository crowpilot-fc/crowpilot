// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/control/DesiredState.h"

#include "src/Config.h"

namespace cp::control::desired {

namespace {

State s_state = {};

float clampf(float v, float lo, float hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

// Channel microseconds to a symmetric stick deflection in [-1, 1],
// centered on RC_MID_US. The RC range is treated as symmetric about
// the center.
float stick(uint16_t us) {
  const float half = static_cast<float>(RC_MAX_US) -
                     static_cast<float>(RC_MID_US);
  const float d = (static_cast<float>(us) -
                   static_cast<float>(RC_MID_US)) / half;
  return clampf(d, -1.0f, 1.0f);
}

}  // anonymous namespace

void init() {
  s_state = State{};
}

void update(const uint16_t* channels) {
  // Throttle: the low end of the RC range is zero thrust, the high end
  // is full thrust.
  const float span = static_cast<float>(RC_MAX_US) -
                     static_cast<float>(RC_MIN_US);
  const float thr = (static_cast<float>(channels[CHANNEL_THROTTLE - 1]) -
                     static_cast<float>(RC_MIN_US)) / span;
  s_state.throttle = clampf(thr, 0.0f, 1.0f);

  // Unshaped stick deflections.
  s_state.roll_passthru  = stick(channels[CHANNEL_ROLL  - 1]);
  s_state.pitch_passthru = stick(channels[CHANNEL_PITCH - 1]);
  s_state.yaw_passthru   = stick(channels[CHANNEL_YAW   - 1]);

  // Roll and pitch sticks command attitude setpoints, the yaw stick
  // commands a rate setpoint.
  s_state.roll_deg     = s_state.roll_passthru  * MAX_ROLL_ANGLE_DEG;
  s_state.pitch_deg    = s_state.pitch_passthru * MAX_PITCH_ANGLE_DEG;
  s_state.yaw_rate_dps = s_state.yaw_passthru   * MAX_YAW_RATE_DPS;

  s_state.arm_us = channels[CHANNEL_ARM - 1];
}

const State& current() {
  return s_state;
}

}  // namespace cp::control::desired

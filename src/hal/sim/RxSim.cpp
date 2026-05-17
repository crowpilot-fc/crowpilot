// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// Simulated receiver HAL for closed-loop SITL. Scripts a pilot that
// holds level sticks, lets the estimator settle disarmed, auto-arms at
// throttle minimum, then steps to hover throttle. The controller is
// then on its own to correct the attitude offset SimPhysics starts in.

#include "src/hal/Hal.h"

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_HIL || BUILD_TARGET == BUILD_TARGET_SITL

#include <Arduino.h>

namespace cp::hal {

namespace {
uint32_t s_tick = 0;
}  // anonymous namespace

bool rx_init() {
  s_tick = 0;
  return true;
}

void rx_poll(RxState& out) {
  ++s_tick;
  out = {};
  for (uint8_t i = 0; i < 16; ++i) {
    out.channel_us[i] = 1500;  // roll, pitch, yaw sticks centred
  }
  out.channel_us[5] = 2000;  // ch6 high: hover

  const uint32_t arm_tick   = LOOP_HZ / 2;  // 0.5 s: settle disarmed
  const uint32_t hover_tick = LOOP_HZ;      // 1.0 s: step to hover
  if (s_tick < arm_tick) {
    out.channel_us[0] = 1000;  // throttle minimum
    out.channel_us[4] = 2000;  // ch5 high: disarmed
  } else if (s_tick < hover_tick) {
    out.channel_us[0] = 1000;  // throttle still minimum, so auto-arm runs
    out.channel_us[4] = 1000;  // ch5 low: arm
  } else {
    out.channel_us[0] = 1500;  // hover throttle
    out.channel_us[4] = 1000;  // armed
  }

  out.channels_valid = true;
  out.last_frame_us  = micros();
}

}  // namespace cp::hal

#endif  // SITL or HIL

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// Simulated receiver HAL for the SITL host build. Scripts a steady,
// disarmed, level stick input. A later physics phase will script stick
// sequences (arm, throttle up, transition) to exercise the controller.

#include "src/hal/Hal.h"

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_HIL || BUILD_TARGET == BUILD_TARGET_SITL

#include <Arduino.h>

namespace cp::hal {

bool rx_init() {
  return true;
}

void rx_poll(RxState& out) {
  out = {};
  for (uint8_t i = 0; i < 16; ++i) {
    out.channel_us[i] = 1500;
  }
  out.channel_us[0] = 1000;  // throttle stick at minimum
  out.channel_us[4] = 2000;  // ch5 high: throttle cut, aircraft disarmed
  out.channel_us[5] = 2000;  // ch6 high: hover end of the transition
  out.channels_valid = true;
  out.last_frame_us  = micros();
}

}  // namespace cp::hal

#endif  // SITL or HIL

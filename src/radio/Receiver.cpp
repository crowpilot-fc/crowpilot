// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/radio/Receiver.h"

#include <Arduino.h>

#include "src/Config.h"
#include "src/hal/Hal.h"

namespace cp::radio {

namespace {

State s_state = {};

}  // anonymous namespace

bool init() {
  s_state = {};
  for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
    s_state.channel_us[i] = 1500;
  }
  return cp::hal::rx_init();
}

void poll() {
  cp::hal::RxState h;
  cp::hal::rx_poll(h);
  for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
    s_state.channel_us[i] = h.channel_us[i];
  }
  s_state.ch17              = h.ch17;
  s_state.ch18              = h.ch18;
  s_state.channels_valid    = h.channels_valid;
  s_state.failsafe_active   = h.failsafe_active;
  s_state.frame_lost_flag   = h.frame_lost_flag;
  s_state.lost_frames_count = h.lost_frames_count;
  s_state.last_frame_us     = h.last_frame_us;
}

const State& state() {
  return s_state;
}

}  // namespace cp::radio

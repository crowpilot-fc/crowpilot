// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "failsafe/Failsafe.h"

#include <Arduino.h>

#include "Config.h"
#include "radio/Receiver.h"

namespace cp::failsafe {

namespace {

// RC-range sanity bounds, derived from the receiver signal range. The
// SBUS decoder maps an 11-bit channel (raw 0 to 2047) to microseconds
// through cp::libs::sbus::rawToMicroseconds, raw * 5 / 8 + 880. That
// produces 880 us at raw 0 and 2159 us at raw 2047, so every channel a
// valid SBUS frame can yield falls in [880, 2159]. A value outside this
// band cannot come from the decode path and is treated as corruption.
constexpr uint16_t kChannelMinUs = 880;
constexpr uint16_t kChannelMaxUs = 2159;
constexpr uint8_t  kCheckedChannels = cp::radio::MAX_CHANNELS;

// Default at boot. active=true so the loop never carries garbage channel
// values forward; link_timeout=true because the receiver has not yet seen
// a frame (cp::radio::state().channels_valid is false until then).
State    s_state    = {
  /*active=*/true,
  /*channel_out_of_range=*/false,
  /*rx_protocol_failsafe=*/false,
  /*rx_frame_lost=*/false,
  /*link_timeout=*/true,
  /*entered_us=*/0,
};
Channels s_channels = {};

// Fill the effective channel snapshot with the configured failsafe defaults
// per SPEC.md §5.11. Auxiliary channels (ch7 through ch16) sit at centered
// 1500 us so any future user-hook consumer sees a benign value.
void applyFailsafeDefaults(Channels& out) {
  out.ch_us[0] = FS_CH1_THROTTLE;
  out.ch_us[1] = FS_CH2_ROLL;
  out.ch_us[2] = FS_CH3_PITCH;
  out.ch_us[3] = FS_CH4_YAW;
  out.ch_us[4] = FS_CH5_ARM;
  out.ch_us[5] = FS_CH6_TRANSITION;
  for (uint8_t i = 6; i < cp::radio::MAX_CHANNELS; ++i) {
    out.ch_us[i] = 1500;
  }
}

// Copy the raw receiver channels through to the effective snapshot.
void passThroughReceiverChannels(Channels& out, const cp::radio::State& rx) {
  for (uint8_t i = 0; i < cp::radio::MAX_CHANNELS; ++i) {
    out.ch_us[i] = rx.channel_us[i];
  }
}

}  // anonymous namespace

void init() {
  s_state = {
    /*active=*/true,
    /*channel_out_of_range=*/false,
    /*rx_protocol_failsafe=*/false,
    /*rx_frame_lost=*/false,
    /*link_timeout=*/true,
    /*entered_us=*/0,
  };
  applyFailsafeDefaults(s_channels);
}

void update() {
  const auto& rx  = cp::radio::state();
  const uint32_t now = micros();

  bool out_of_range = false;
  if (rx.channels_valid) {
    for (uint8_t i = 0; i < kCheckedChannels; ++i) {
      const uint16_t v = rx.channel_us[i];
      if (v < kChannelMinUs || v > kChannelMaxUs) {
        out_of_range = true;
        break;
      }
    }
  }

  // Link timeout. The boot case is covered by !channels_valid so the
  // timing arithmetic against last_frame_us=0 is never relied on.
  const bool link_to = !rx.channels_valid ||
                       (now - rx.last_frame_us) > FS_LINK_TIMEOUT_US;

  s_state.channel_out_of_range = out_of_range;
  s_state.rx_protocol_failsafe = rx.failsafe_active;
  s_state.rx_frame_lost        = rx.frame_lost_flag;
  s_state.link_timeout         = link_to;

  const bool any_trigger = out_of_range ||
                           rx.failsafe_active ||
                           link_to;

  if (any_trigger) {
    if (!s_state.active) {
      s_state.entered_us = now;
    }
    s_state.active = true;
    applyFailsafeDefaults(s_channels);
  } else {
    s_state.active = false;
    passThroughReceiverChannels(s_channels, rx);
  }
}

const State& state() {
  return s_state;
}

const Channels& channels() {
  return s_channels;
}

}  // namespace cp::failsafe

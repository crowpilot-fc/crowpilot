// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

namespace cp::radio {

constexpr uint8_t MAX_CHANNELS = 16;

// Receiver state. Populated by poll() each tick from the underlying
// protocol driver (SBUS in v1.0). The struct is a snapshot; failsafe
// and frame-lost flags reflect the last received frame's status bits.
struct State {
  uint16_t channel_us[MAX_CHANNELS];
  bool     ch17;
  bool     ch18;
  bool     channels_valid;
  bool     failsafe_active;
  bool     frame_lost_flag;
  uint32_t lost_frames_count;
  uint32_t last_frame_us;
};

// Bring up the protocol driver (SBUS over PIO for v1.0). Returns false
// if the underlying PIO program cannot be loaded.
bool init();

// Drain any pending receive data and update the state struct. Called
// near the top of each loop tick per ARCHITECTURE.md §2.
void poll();

// Latest snapshot of receiver state. Always safe to read after init.
const State& state();

}  // namespace cp::radio

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

#include "radio/Receiver.h"

// Failsafe controlled-fall override per SPEC.md §5.11 and ALGORITHMS.md §9.
// Reads the receiver state each tick, evaluates three trigger conditions,
// and exposes a per-channel snapshot that downstream consumers read instead
// of the raw receiver channels:
//
//   1. Any RC channel outside [800, 2200] us.
//   2. The SBUS protocol-level failsafe flag is set.
//   3. No fresh frame seen for more than 100 ms.
//
// When any trigger is true the module sets active, records the entry time,
// and replaces the channel snapshot with Config.h defaults that drive a
// gentle nose-up descent on the tailsitter (low throttle, centered sticks,
// hover-mode fader). When all triggers clear the module exits failsafe and
// the raw receiver channels pass through.

namespace cp::failsafe {

struct State {
  bool     active;
  bool     channel_out_of_range;
  bool     rx_protocol_failsafe;
  bool     rx_frame_lost;
  bool     link_timeout;
  uint32_t entered_us;        // micros() at the most recent enter event; 0 since boot
};

struct Channels {
  uint16_t ch_us[cp::radio::MAX_CHANNELS];
};

// Set the module to its boot state. Active at boot per SPEC.md §9.4 so the
// firmware never carries unsafe channel values forward from uninitialized
// memory; the first call to update() that sees a healthy receiver will
// clear the active flag.
void init();

// Evaluate the trigger conditions against the latest receiver state and
// either pass the raw channels through or apply the Config.h defaults.
void update();

const State&    state();
const Channels& channels();

}  // namespace cp::failsafe

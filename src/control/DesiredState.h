// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// Pilot desired-state generation, per TAILSITTER_CONTROL_SPEC.md
// section 7.
//
// Maps the pilot stick channels to control setpoints. Throttle becomes
// a normalized thrust command. Roll and pitch sticks become attitude
// setpoints scaled to configurable maximum angles. The yaw stick
// becomes a yaw-rate setpoint scaled to a configurable maximum rate.
// The unshaped stick deflections are also exposed for any open-loop
// feedforward the mixer applies.
//
// The mapping is the same in both regimes. The hover and forward-flight
// dynamics differ, but that difference is handled by the regime-
// scheduled controller gains, not by reshaping the setpoints here. The
// optional fixed-wing shaping that spec section 7 describes (a
// coordinated-turn term and a trim offset) is left out of v1: it is
// explicitly optional and would need bench-derived values.

namespace cp::control::desired {

struct State {
  float throttle;        // normalized thrust command, [0, 1]
  float roll_deg;        // roll attitude setpoint
  float pitch_deg;       // pitch attitude setpoint
  float yaw_rate_dps;    // yaw-rate setpoint, degrees per second
  float roll_passthru;   // unshaped roll stick, [-1, 1]
  float pitch_passthru;  // unshaped pitch stick, [-1, 1]
  float yaw_passthru;    // unshaped yaw stick, [-1, 1]
  uint16_t ch5_us;       // arm-channel microseconds, for the actuator stage
};

// Reset the desired state to zero.
void init();

// One step. `channels` is the failsafe-effective channel array
// (microseconds, 1-based channels at 0-based indices).
void update(const uint16_t* channels);

// The latest desired state. Always safe to call after init.
const State& current();

}  // namespace cp::control::desired

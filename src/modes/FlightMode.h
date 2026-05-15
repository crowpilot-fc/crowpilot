// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// Transition management, per TAILSITTER_CONTROL_SPEC.md section 6.
//
// A pilot transmitter channel commands the move between hover and
// forward flight. The raw channel maps to a normalized command, 1.0 at
// the hover end and 0.0 at the forward end. The value the rest of the
// control core acts on, the fader, is rate-limited: it slews toward the
// command at a bounded rate. The airframe cannot rotate 90 degrees
// instantly, so an instantaneous reference change would command an
// impossible maneuver. The slew makes even a snapped switch produce a
// controllable transition.
//
// A named flight-mode enumeration is derived from the fader for
// telemetry and display. It does not feed back into the control law.
// The control law is continuous in the fader value.

namespace cp::modes {

enum class FlightMode : uint8_t {
  HOVER         = 0,
  TRANSITIONING = 1,
  FORWARD       = 2,
};

// Reset to the hover regime. A tailsitter rests on its tail, so boot is
// the hover end of the transition.
void init();

// One transition step. `channels` is the failsafe-effective channel
// array (microseconds, 1-based channels at 0-based indices). `dt_s` is
// the elapsed loop period in seconds.
void update(const uint16_t* channels, float dt_s);

// The rate-limited transition value, 1.0 hover to 0.0 forward.
float fader();

// The named regime derived from the fader. For telemetry and display
// only.
FlightMode mode();

}  // namespace cp::modes

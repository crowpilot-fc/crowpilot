// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "modes/FlightMode.h"

#include "Config.h"

namespace cp::modes {

namespace {

// Fader bands that name the regime. The middle band is the transition.
// These classify for display only and do not affect the control law.
constexpr float kHoverThreshold   = 0.95f;
constexpr float kForwardThreshold = 0.05f;

float      s_fader = 1.0f;
FlightMode s_mode  = FlightMode::HOVER;

float clamp01(float v) {
  if (v < 0.0f) {
    return 0.0f;
  }
  if (v > 1.0f) {
    return 1.0f;
  }
  return v;
}

}  // anonymous namespace

void init() {
  s_fader = 1.0f;
  s_mode  = FlightMode::HOVER;
}

void update(const uint16_t* channels, float dt_s) {
  const uint16_t us = channels[CHANNEL_TRANSITION - 1];

  // Map the channel to the transition command. The hover end is the
  // high pulse and the forward end is the low pulse, matching the
  // transmitter setup the documentation describes.
  const float span = static_cast<float>(RC_MAX_US) -
                      static_cast<float>(RC_MIN_US);
  const float frac = (static_cast<float>(us) -
                      static_cast<float>(RC_MIN_US)) / span;
  const float command = clamp01(frac);

  // Slew the fader toward the command at the bounded rate.
  const float max_step = TRANSITION_SLEW_RATE * dt_s;
  float step = command - s_fader;
  if (step > max_step) {
    step = max_step;
  } else if (step < -max_step) {
    step = -max_step;
  }
  s_fader = clamp01(s_fader + step);

  if (s_fader >= kHoverThreshold) {
    s_mode = FlightMode::HOVER;
  } else if (s_fader <= kForwardThreshold) {
    s_mode = FlightMode::FORWARD;
  } else {
    s_mode = FlightMode::TRANSITIONING;
  }
}

float fader() {
  return s_fader;
}

FlightMode mode() {
  return s_mode;
}

}  // namespace cp::modes

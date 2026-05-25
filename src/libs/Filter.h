// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <math.h>

// Generic real-time filter primitives shared by the estimator and the
// controllers. Header-only: each is a tiny per-instance state machine, cheap
// enough to run on every axis at the 1 kHz loop rate.

namespace cp::libs::filter {

constexpr float kTwoPi = 6.28318530717958647692f;

// First-order (PT1) low-pass. Seeds on the first sample to avoid a startup
// transient. A non-positive cutoff or dt passes the input through unchanged,
// so a zero cutoff is the disabled state.
class Pt1 {
 public:
  float apply(float x, float cutoff_hz, float dt_s) {
    if (cutoff_hz <= 0.0f || dt_s <= 0.0f) {
      return x;
    }
    if (!seeded_) {
      y_      = x;
      seeded_ = true;
      return y_;
    }
    const float rc    = 1.0f / (kTwoPi * cutoff_hz);
    const float alpha = dt_s / (rc + dt_s);
    y_ += alpha * (x - y_);
    return y_;
  }

  void reset() {
    y_      = 0.0f;
    seeded_ = false;
  }

 private:
  float y_      = 0.0f;
  bool  seeded_ = false;
};

// Second-order notch (RBJ audio-EQ cookbook), Direct Form II transposed.
// Removes a narrow band around a vibration peak. configure() with a center
// of 0, a bad sample rate, or a bad Q leaves it disabled and passing through.
// Higher Q is a narrower, more surgical notch.
class BiquadNotch {
 public:
  void configure(float center_hz, float sample_hz, float q) {
    if (center_hz <= 0.0f || sample_hz <= 0.0f || q <= 0.0f) {
      enabled_ = false;
      return;
    }
    const float w0    = kTwoPi * center_hz / sample_hz;
    const float cw    = cosf(w0);
    const float sw    = sinf(w0);
    const float alpha = sw / (2.0f * q);
    const float a0    = 1.0f + alpha;
    b0_ = 1.0f / a0;
    b1_ = (-2.0f * cw) / a0;
    b2_ = 1.0f / a0;
    a1_ = (-2.0f * cw) / a0;
    a2_ = (1.0f - alpha) / a0;
    z1_ = 0.0f;
    z2_ = 0.0f;
    enabled_ = true;
  }

  float apply(float x) {
    if (!enabled_) {
      return x;
    }
    const float y = b0_ * x + z1_;
    z1_ = b1_ * x - a1_ * y + z2_;
    z2_ = b2_ * x - a2_ * y;
    return y;
  }

  void reset() {
    z1_ = 0.0f;
    z2_ = 0.0f;
  }

 private:
  float b0_ = 1.0f, b1_ = 0.0f, b2_ = 0.0f;
  float a1_ = 0.0f, a2_ = 0.0f;
  float z1_ = 0.0f, z2_ = 0.0f;
  bool  enabled_ = false;
};

}  // namespace cp::libs::filter

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/sensors/Barometer.h"

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#include "src/Config.h"
#include "src/hal/Hal.h"

namespace cp::sensors::baro {

namespace {

// International Standard Atmosphere altitude conversion per ALGORITHMS.md
// §4.3. Stays in the facade since the chip layer reports pressure only;
// altitude is a derived value relative to the ground baseline captured
// at the first successful read.
constexpr float kAltitudeScale = 44330.0f;
constexpr float kAltitudeExp   = 1.0f / 5.255f;

// Lowest plausible atmospheric pressure for a ground-level baseline.
// 30 kPa is roughly 9000 m, well below any realistic launch site, so a
// reading under it means the chip has not produced a real sample yet.
constexpr float kMinValidPressurePa = 30000.0f;

Sample   s_latest          = {};
bool     s_healthy         = false;
bool     s_have_baseline   = false;
float    s_ground_pressure = 0.0f;
uint32_t s_tick_count      = 0;

// Baseline settling. The BMP IIR filter takes several samples to
// converge after power-up, so the first plausible reading is still
// drifting. Discard the first reads, then average a few, before locking
// the ground-pressure reference.
constexpr uint8_t kBaselineSkip    = 8;
constexpr uint8_t kBaselineSamples = 8;
uint8_t  s_baseline_skipped = 0;
uint8_t  s_baseline_count   = 0;
float    s_baseline_accum   = 0.0f;

}  // anonymous namespace

bool init() {
  s_latest           = {};
  s_healthy          = false;
  s_have_baseline    = false;
  s_tick_count       = 0;
  s_baseline_skipped = 0;
  s_baseline_count   = 0;
  s_baseline_accum   = 0.0f;

  if (!cp::hal::baro_init()) {
    return false;
  }
  s_healthy = cp::hal::baro_present();
  return true;
}

bool read() {
  if (!cp::hal::baro_present()) {
    s_latest.valid        = false;
    s_latest.timestamp_us = micros();
    return false;
  }

  if (++s_tick_count < BARO_READ_INTERVAL_TICKS) {
    return s_latest.valid;
  }
  s_tick_count = 0;

  cp::hal::BaroSample h;
  if (!cp::hal::baro_read(h)) {
    s_latest.valid = false;
    s_healthy      = false;
    return false;
  }

  // Capture the ground-pressure baseline. Only plausible readings count:
  // a sub-30 kPa value means the chip has not produced a real sample
  // yet. The first plausible reads are still settling through the IIR
  // filter, so they are skipped and the next few are averaged.
  if (!s_have_baseline && h.pressure_pa > kMinValidPressurePa) {
    if (s_baseline_skipped < kBaselineSkip) {
      ++s_baseline_skipped;
    } else {
      s_baseline_accum += h.pressure_pa;
      if (++s_baseline_count >= kBaselineSamples) {
        s_ground_pressure =
            s_baseline_accum / static_cast<float>(kBaselineSamples);
        s_have_baseline = true;
      }
    }
  }

  float altitude_m = 0.0f;
  if (s_ground_pressure > 1.0f && h.pressure_pa > 1.0f) {
    const float ratio = h.pressure_pa / s_ground_pressure;
    altitude_m = kAltitudeScale * (1.0f - powf(ratio, kAltitudeExp));
  }

  s_latest.pressure_pa   = h.pressure_pa;
  s_latest.temperature_c = h.temperature_c;
  s_latest.altitude_m    = altitude_m;
  s_latest.timestamp_us  = micros();
  s_latest.valid         = true;
  s_healthy              = true;
  return true;
}

const Sample& latest() {
  return s_latest;
}

bool is_present() {
  return cp::hal::baro_present();
}

bool is_healthy() {
  return s_healthy;
}

}  // namespace cp::sensors::baro

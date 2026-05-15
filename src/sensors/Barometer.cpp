// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "sensors/Barometer.h"

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#include "Config.h"
#include "hal/Hal.h"

namespace cp::sensors::baro {

namespace {

// International Standard Atmosphere altitude conversion per ALGORITHMS.md
// §4.3. Stays in the facade since the chip layer reports pressure only;
// altitude is a derived value relative to the ground baseline captured
// at the first successful read.
constexpr float kAltitudeScale = 44330.0f;
constexpr float kAltitudeExp   = 1.0f / 5.255f;

Sample   s_latest          = {};
bool     s_healthy         = false;
bool     s_have_baseline   = false;
float    s_ground_pressure = 0.0f;
uint32_t s_tick_count      = 0;

}  // anonymous namespace

bool init() {
  s_latest        = {};
  s_healthy       = false;
  s_have_baseline = false;
  s_tick_count    = 0;

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

  if (!s_have_baseline) {
    s_ground_pressure = h.pressure_pa;
    s_have_baseline   = true;
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

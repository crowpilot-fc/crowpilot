// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/sensors/Imu.h"

#include <Arduino.h>

#include "src/Config.h"
#include "src/hal/Hal.h"

namespace cp::sensors::imu {

namespace {

Sample s_latest  = {};
bool   s_healthy = false;

// A single transient I2C NAK should not flip the IMU unhealthy. Health
// drops only after this many consecutive failed reads.
constexpr uint8_t kMaxConsecFailures = 3;
uint8_t s_consec_failures = 0;

}  // anonymous namespace

bool init() {
  if (!cp::hal::imu_init()) {
    s_healthy = false;
    return false;
  }
  s_latest          = {};
  s_healthy         = true;
  s_consec_failures = 0;
  return true;
}

bool read() {
  cp::hal::ImuSample h;
  if (!cp::hal::imu_read(h)) {
    s_latest.valid = false;
    if (s_consec_failures < kMaxConsecFailures) {
      ++s_consec_failures;
    }
    if (s_consec_failures >= kMaxConsecFailures) {
      s_healthy = false;
    }
    return false;
  }

  // Pass-through the raw register values to the facade Sample. The
  // telemetry logger reads these. In native mode they are the int16
  // chip values; in sim mode they are reverse-scaled (or zero) and the
  // log records that approximation.
  s_latest.raw_ax   = h.raw_ax;
  s_latest.raw_ay   = h.raw_ay;
  s_latest.raw_az   = h.raw_az;
  s_latest.raw_gx   = h.raw_gx;
  s_latest.raw_gy   = h.raw_gy;
  s_latest.raw_gz   = h.raw_gz;
  s_latest.raw_temp = h.raw_temp;

  // Apply bias subtraction on the scaled fields. Bias values come from
  // Config.h, populated by the Phase 3 calibration routine.
  s_latest.ax_g   = h.ax_g   - ACC_BIAS_X;
  s_latest.ay_g   = h.ay_g   - ACC_BIAS_Y;
  s_latest.az_g   = h.az_g   - ACC_BIAS_Z;
  s_latest.gx_dps = h.gx_dps - GYRO_BIAS_X;
  s_latest.gy_dps = h.gy_dps - GYRO_BIAS_Y;
  s_latest.gz_dps = h.gz_dps - GYRO_BIAS_Z;
  s_latest.temp_c = h.temp_c;

  s_latest.timestamp_us = micros();
  s_latest.valid        = true;
  s_healthy             = true;
  s_consec_failures     = 0;
  return true;
}

const Sample& latest() {
  return s_latest;
}

bool is_healthy() {
  return s_healthy;
}

}  // namespace cp::sensors::imu

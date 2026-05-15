// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

namespace cp::sensors::imu {

// One IMU sample with both raw register values and scaled engineering units.
// Bias subtraction (per SPEC.md §5.10 / §4.2) is applied to the scaled
// fields. Population happens in cp::sensors::imu::read().
struct Sample {
  int16_t  raw_ax;
  int16_t  raw_ay;
  int16_t  raw_az;
  int16_t  raw_gx;
  int16_t  raw_gy;
  int16_t  raw_gz;
  int16_t  raw_temp;

  float    ax_g;
  float    ay_g;
  float    az_g;
  float    gx_dps;
  float    gy_dps;
  float    gz_dps;
  float    temp_c;

  uint32_t timestamp_us;
  bool     valid;
};

// Bring up the I2C0 bus, probe and configure the selected IMU. Returns
// false on any failure. Caller halts the boot on a false return per
// CODING_STANDARDS.md §7.1.
bool init();

// One non-blocking burst read followed by scaling and bias subtraction.
// Returns false if the I2C transaction failed. Sample valid flag mirrors
// the return value.
bool read();

// Latest sample. Always valid to call after init returns true, but the
// returned sample may carry valid=false if a transient I2C failure was
// last seen.
const Sample& latest();

// True once init has succeeded and read has not seen a transient failure
// since.
bool is_healthy();

}  // namespace cp::sensors::imu

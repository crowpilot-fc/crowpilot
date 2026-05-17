// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// Simulated IMU HAL for the SITL host build. The host-build checkpoint
// scripts a level, static aircraft. A later physics phase will replace
// the scripted sample with one derived from the simulated dynamics.

#include "src/hal/Hal.h"

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_HIL || BUILD_TARGET == BUILD_TARGET_SITL

namespace cp::hal {

bool imu_init() {
  return true;
}

bool imu_read(ImuSample& out) {
  // Static, in the nose-up hover attitude a tailsitter rests in. Gravity
  // reaction acts along the body x-axis, which points up in hover, so
  // accel reads +1 g on x and the gyro reads zero. The raw register
  // fields stay zero; the SITL build does not run the telemetry logger,
  // their only consumer.
  out = {};
  out.ax_g   = 1.0f;
  out.ay_g   = 0.0f;
  out.az_g   = 0.0f;
  out.gx_dps = 0.0f;
  out.gy_dps = 0.0f;
  out.gz_dps = 0.0f;
  out.temp_c = 25.0f;
  return true;
}

}  // namespace cp::hal

#endif  // SITL or HIL

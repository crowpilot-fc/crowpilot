// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// Simulated IMU HAL for closed-loop SITL. The gyro and accelerometer
// samples are read back from the rigid-body model in SimPhysics, which
// OutSim drives from the controller's actuator output. The loop closes
// here: controller -> mixer -> OutSim -> physics -> ImuSim -> estimator.

#include "src/hal/Hal.h"

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_HIL || BUILD_TARGET == BUILD_TARGET_SITL

#include "src/hal/sim/SimPhysics.h"

namespace cp::hal {

bool imu_init() {
  cp::sim::physics_init();
  return true;
}

bool imu_read(ImuSample& out) {
  // The raw register fields stay zero; the SITL build does not run the
  // telemetry logger, their only consumer.
  out = {};
  cp::sim::physics_accel_g(out.ax_g, out.ay_g, out.az_g);
  cp::sim::physics_gyro_dps(out.gx_dps, out.gy_dps, out.gz_dps);
  out.temp_c = 25.0f;
  return true;
}

}  // namespace cp::hal

#endif  // SITL or HIL

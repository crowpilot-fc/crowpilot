// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// Simulated barometer HAL for the SITL host build. Reports a fixed
// sea-level atmosphere. A later physics phase will derive pressure from
// the simulated altitude.

#include "src/hal/Hal.h"

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_HIL || BUILD_TARGET == BUILD_TARGET_SITL

namespace cp::hal {

bool baro_present() {
  return true;
}

bool baro_init() {
  return true;
}

bool baro_read(BaroSample& out) {
  out.pressure_pa   = 101325.0f;  // ISA sea-level pressure
  out.temperature_c = 25.0f;
  return true;
}

}  // namespace cp::hal

#endif  // SITL or HIL

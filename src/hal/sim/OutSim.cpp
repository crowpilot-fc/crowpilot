// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// Simulated actuator-output HAL for the SITL host build. The host-build
// checkpoint discards the commanded pulse widths. A later physics phase
// will feed them into the simulated dynamics.

#include "src/hal/Hal.h"

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_HIL || BUILD_TARGET == BUILD_TARGET_SITL

namespace cp::hal {

void out_init() {}

void out_set_motor_us(uint8_t, uint16_t) {}

void out_set_servo_us(uint8_t, uint16_t) {}

void out_commit_motors() {}

}  // namespace cp::hal

#endif  // SITL or HIL

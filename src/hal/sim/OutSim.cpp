// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// Simulated actuator-output HAL for closed-loop SITL. The commanded
// pulse widths are converted back to normalised commands and fed into
// the rigid-body model. out_commit_motors steps the model one loop
// period, so the next imu_read reflects the response.

#include "src/hal/Hal.h"

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_HIL || BUILD_TARGET == BUILD_TARGET_SITL

#include "src/hal/sim/SimPhysics.h"

namespace cp::hal {

namespace {

// OneShot125 motor pulse: 125 us is zero thrust, 250 us is full. The
// sub-125 us disarm pulse clamps to zero.
float motorNorm(uint16_t pulse_us) {
  const float n = (static_cast<float>(pulse_us) - 125.0f) / 125.0f;
  return n < 0.0f ? 0.0f : (n > 1.0f ? 1.0f : n);
}

// Servo PWM: 1000 to 2000 us, 1500 us centred. Maps to -1..+1.
float servoDeflection(uint16_t pulse_us) {
  return (static_cast<float>(pulse_us) - 1500.0f) / 500.0f;
}

}  // anonymous namespace

void out_init() {}

void out_set_motor_us(uint8_t idx, uint16_t pulse_us) {
  cp::sim::physics_set_motor(idx, motorNorm(pulse_us));
}

void out_set_servo_us(uint8_t idx, uint16_t pulse_us) {
  cp::sim::physics_set_servo(idx, servoDeflection(pulse_us));
}

void out_commit_motors() {
  cp::sim::physics_step(1.0f / static_cast<float>(LOOP_HZ));
}

}  // namespace cp::hal

#endif  // SITL or HIL

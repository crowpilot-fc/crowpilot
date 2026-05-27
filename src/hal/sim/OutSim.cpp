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

#include "src/airframes/Airframe.h"
#include "src/hal/sim/SimPhysics.h"

namespace cp::hal {

namespace {

// Last normalised motor command, kept so out_get_erpm can synthesise a
// plausible eRPM for the dynamic-notch path in simulation.
float s_motor_norm[cp::airframes::N_MOTORS] = {};

// Motor pulse to normalised thrust, using the configured ESC pulse
// range so it tracks MOTOR_PROTOCOL. Idle is zero thrust, max is full.
// A sub-idle disarm pulse clamps to zero.
float motorNorm(uint16_t pulse_us) {
  const float span =
      static_cast<float>(ESC_MAX_PULSE_US - ESC_IDLE_PULSE_US);
  const float n =
      (static_cast<float>(pulse_us) - ESC_IDLE_PULSE_US) / span;
  return n < 0.0f ? 0.0f : (n > 1.0f ? 1.0f : n);
}

// Servo PWM: 1000 to 2000 us, 1500 us centred. Maps to -1..+1.
float servoDeflection(uint16_t pulse_us) {
  return (static_cast<float>(pulse_us) - 1500.0f) / 500.0f;
}

}  // anonymous namespace

void out_init() {
  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    s_motor_norm[i] = 0.0f;
  }
}

void out_set_motor_us(uint8_t idx, uint16_t pulse_us) {
  const float n = motorNorm(pulse_us);
  if (idx < cp::airframes::N_MOTORS) {
    s_motor_norm[idx] = n;
  }
  cp::sim::physics_set_motor(idx, n);
}

void out_set_servo_us(uint8_t idx, uint16_t pulse_us) {
  cp::sim::physics_set_servo(idx, servoDeflection(pulse_us));
}

void out_commit_motors() {
  cp::sim::physics_step(1.0f / static_cast<float>(LOOP_HZ));
}

uint32_t out_get_erpm(uint8_t idx) {
  // Synthesise an eRPM from the motor command so the dynamic notch can be
  // exercised in SITL. A stopped motor reports 0. A running motor maps the
  // command onto a plausible 6000 to 24000 mechanical RPM band (100 to 400 Hz
  // fundamental), then scales by the pole pairs to electrical RPM.
  if (idx >= cp::airframes::N_MOTORS || s_motor_norm[idx] <= 0.0f) {
    return 0;
  }
  const float mech_rpm = 6000.0f + s_motor_norm[idx] * 18000.0f;
  return static_cast<uint32_t>(mech_rpm * static_cast<float>(MOTOR_POLE_PAIRS));
}

}  // namespace cp::hal

#endif  // SITL or HIL

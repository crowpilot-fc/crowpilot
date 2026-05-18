// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/hal/Hal.h"

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_NATIVE

#include <Arduino.h>
#include <Servo.h>

#include "src/airframes/Airframe.h"

// Native actuator output. See docs/developer-guide/algorithms.md.
//
// MOTOR_PROTOCOL selects how the motor pins are driven. With
// MOTOR_PROTOCOL_ONESHOT125 the motor pulse widths are buffered by
// out_set_motor_us and emitted together by out_commit_motors as one
// synchronous 125 to 250 us burst. With MOTOR_PROTOCOL_PWM the motors
// are driven through the Servo library at standard 1000 to 2000 us,
// the same path as the servos, for ESCs that do not support OneShot.
//
// Servos always use the Servo library, which maintains its own 50 Hz
// PWM schedule, so out_set_servo_us applies the new width immediately.

namespace cp::hal {

namespace {

// Motor index to GPIO pin. The index order matches the airframe motor
// constants (MOTOR_RIGHT is 0, MOTOR_LEFT is 1).
const uint8_t kMotorPins[cp::airframes::N_MOTORS] = {
  PIN_MOTOR_RIGHT,
  PIN_MOTOR_LEFT,
};

Servo s_servos[cp::airframes::N_SERVOS];

#if MOTOR_PROTOCOL == MOTOR_PROTOCOL_PWM
Servo s_motors[cp::airframes::N_MOTORS];
#else
uint16_t s_motor_us[cp::airframes::N_MOTORS] = {};
#endif

}  // anonymous namespace

void out_init() {
#if MOTOR_PROTOCOL == MOTOR_PROTOCOL_PWM
  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    s_motors[i].attach(kMotorPins[i]);
    s_motors[i].writeMicroseconds(ESC_DISARM_PULSE_US);
  }
#else
  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    pinMode(kMotorPins[i], OUTPUT);
    digitalWriteFast(kMotorPins[i], LOW);
    s_motor_us[i] = ESC_DISARM_PULSE_US;
  }
#endif
  for (uint8_t i = 0; i < cp::airframes::N_SERVOS; ++i) {
    s_servos[i].attach(PIN_SERVOS[i]);
  }
}

void out_set_motor_us(uint8_t idx, uint16_t pulse_us) {
  if (idx < cp::airframes::N_MOTORS) {
#if MOTOR_PROTOCOL == MOTOR_PROTOCOL_PWM
    s_motors[idx].writeMicroseconds(pulse_us);
#else
    s_motor_us[idx] = pulse_us;
#endif
  }
}

void out_set_servo_us(uint8_t idx, uint16_t pulse_us) {
  if (idx < cp::airframes::N_SERVOS) {
    s_servos[idx].writeMicroseconds(pulse_us);
  }
}

void out_commit_motors() {
#if MOTOR_PROTOCOL == MOTOR_PROTOCOL_PWM
  // Standard PWM: the Servo library already drives the motor pins on
  // its own 50 Hz schedule, so there is nothing to commit.
#else
  // OneShot125: raise every motor pin together, then lower each one
  // when its own buffered pulse width has elapsed. The burst blocks for
  // at most the longest pulse, about 250 microseconds.
  bool lowered[cp::airframes::N_MOTORS] = {};
  uint8_t remaining = cp::airframes::N_MOTORS;

  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    digitalWriteFast(kMotorPins[i], HIGH);
  }
  const uint32_t start = micros();

  while (remaining > 0) {
    const uint32_t elapsed = micros() - start;
    for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
      if (!lowered[i] && elapsed >= s_motor_us[i]) {
        digitalWriteFast(kMotorPins[i], LOW);
        lowered[i] = true;
        --remaining;
      }
    }
  }
#endif
}

}  // namespace cp::hal

#endif  // BUILD_TARGET == BUILD_TARGET_NATIVE

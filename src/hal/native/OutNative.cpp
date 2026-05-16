// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "hal/Hal.h"

#include "Config.h"

#if BUILD_TARGET == BUILD_TARGET_NATIVE

#include <Arduino.h>
#include <Servo.h>

#include "airframes/Airframe.h"

// Native actuator output. See docs/developer-guide/algorithms.md.
//
// Motors are driven with the OneShot125 ESC protocol: a pulse of 125 to
// 250 microseconds, repeated once per loop tick. Motor pulse widths are
// buffered by out_set_motor_us and emitted together by
// out_commit_motors as one synchronous burst, so the two motors see
// pulses that start at the same instant. Servos use the Arduino Servo
// library, which maintains its own 20 ms PWM schedule, so out_set_
// servo_us applies the new width immediately.

namespace cp::hal {

namespace {

// Motor index to GPIO pin. The index order matches the airframe motor
// constants (MOTOR_RIGHT is 0, MOTOR_LEFT is 1).
const uint8_t kMotorPins[cp::airframes::N_MOTORS] = {
  PIN_MOTOR_RIGHT,
  PIN_MOTOR_LEFT,
};

Servo    s_servos[cp::airframes::N_SERVOS];
uint16_t s_motor_us[cp::airframes::N_MOTORS] = {};

}  // anonymous namespace

void out_init() {
  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    pinMode(kMotorPins[i], OUTPUT);
    digitalWriteFast(kMotorPins[i], LOW);
    s_motor_us[i] = ESC_DISARM_PULSE_US;
  }
  for (uint8_t i = 0; i < cp::airframes::N_SERVOS; ++i) {
    s_servos[i].attach(PIN_SERVOS[i]);
  }
}

void out_set_motor_us(uint8_t idx, uint16_t pulse_us) {
  if (idx < cp::airframes::N_MOTORS) {
    s_motor_us[idx] = pulse_us;
  }
}

void out_set_servo_us(uint8_t idx, uint16_t pulse_us) {
  if (idx < cp::airframes::N_SERVOS) {
    s_servos[idx].writeMicroseconds(pulse_us);
  }
}

void out_commit_motors() {
  // Raise every motor pin together, then lower each one when its own
  // buffered pulse width has elapsed. The burst blocks for at most the
  // longest pulse, about 250 microseconds.
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
}

}  // namespace cp::hal

#endif  // BUILD_TARGET == BUILD_TARGET_NATIVE

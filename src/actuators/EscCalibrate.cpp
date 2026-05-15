// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "actuators/EscCalibrate.h"

#include <Arduino.h>
#include <stdint.h>

#include "Config.h"
#include "airframes/Airframe.h"

namespace cp::actuators::esc_calibrate {

namespace {

// Motor pin lookup, duplicated from Output.cpp deliberately. The
// calibration runs once at boot before any flight code; keeping it
// self-contained avoids exposing Output's internal helpers to a path
// that the rest of the firmware never touches.
const uint8_t kMotorPins[cp::airframes::N_MOTORS] = {
  PIN_MOTOR_RIGHT,
  PIN_MOTOR_LEFT,
};

constexpr uint32_t kMaxPulseHoldUs  = 6UL * 1000UL * 1000UL;  // 6 seconds
constexpr uint32_t kIdlePulseHoldUs = 4UL * 1000UL * 1000UL;  // 4 seconds

void emitPulseAll(uint16_t pulse_us) {
  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    digitalWriteFast(kMotorPins[i], HIGH);
  }
  const uint32_t start = micros();
  while ((micros() - start) < pulse_us) {
    // spin
  }
  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    digitalWriteFast(kMotorPins[i], LOW);
  }
}

// Emit pulse_us on all motors continuously for hold_us total microseconds.
// Each pulse takes pulse_us microseconds; the loop back-to-back gives a
// pulse rate well above ESC requirements.
void holdPulse(uint16_t pulse_us, uint32_t hold_us) {
  const uint32_t start = micros();
  while ((micros() - start) < hold_us) {
    emitPulseAll(pulse_us);
  }
}

[[noreturn]] void haltForever() {
  // Volatile counter prevents the side-effect-free infinite loop from
  // being optimized away under C++17 UB rules.
  volatile uint32_t spin = 0;
  while (true) {
    spin = spin + 1;
  }
}

}  // anonymous namespace

[[noreturn]] void run() {
  // Configure the motor pins. Output::init() was called immediately
  // before us per the loop init order, so the pins are already OUTPUT
  // and LOW. The pinMode call is defensive and idempotent.
  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    pinMode(kMotorPins[i], OUTPUT);
    digitalWriteFast(kMotorPins[i], LOW);
  }

  if (Serial) {
    Serial.println("ESC calibration starting.");
    Serial.println("BATTERY DISCONNECTED from ESCs at this point.");
    Serial.println("Sending MAX pulse for 6 seconds.");
    Serial.println("CONNECT THE ESC BATTERY NOW. ESC should beep and enter calibration.");
  }

  holdPulse(ESC_MAX_PULSE_US, kMaxPulseHoldUs);

  if (Serial) {
    Serial.println("Switching to IDLE pulse for 4 seconds. ESC should store the range.");
  }

  holdPulse(ESC_IDLE_PULSE_US, kIdlePulseHoldUs);

  // Drive pins LOW so the ESC sees "no signal" and stops listening.
  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    digitalWriteFast(kMotorPins[i], LOW);
  }

  if (Serial) {
    Serial.println();
    Serial.println("ESC calibration complete.");
    Serial.println("Set ENABLE_ESC_CALIBRATION = 0 in src/Config.h and reflash.");
    Serial.println("Halted.");
  }

  haltForever();
}

}  // namespace cp::actuators::esc_calibrate

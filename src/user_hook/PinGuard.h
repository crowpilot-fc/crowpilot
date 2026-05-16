// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// Pin-claim guard for the user extension hook. Every GPIO the firmware
// drives (I2C, SPI/SD, SBUS, motors, servos, LEDs, companion UART) is
// claimed. User code that tries to reconfigure or write a claimed pin is
// ignored, with a one-time warning. Reads always pass through.
//
// SensorApi.h macro-redirects pinMode, digitalWrite, and analogWrite in the
// user-sketch translation unit to the guarded entry points below. Firmware
// code under src/ never includes SensorApi.h and so is never redirected.
//
// The guard reaches only those three calls. A library that drives a pin
// through its own path (Servo, Wire, SPI, tone) bypasses the macros, so a
// user Servo.attach() on a claimed pin is not caught. The user-sketch docs
// tell users to stay on the free pins for this reason.

namespace cp::user_hook::pins {

// True if `pin` is on the firmware claim list for the active board and
// airframe.
bool isClaimed(uint8_t pin);

// Guarded pinMode. Ignored (with a one-time warning) for claimed pins.
void guardPinMode(uint8_t pin, uint8_t mode);

// Guarded digitalWrite. Ignored (with a one-time warning) for claimed pins.
void guardDigitalWrite(uint8_t pin, uint8_t value);

// Guarded analogWrite. Ignored (with a one-time warning) for claimed pins.
void guardAnalogWrite(uint8_t pin, int value);

}  // namespace cp::user_hook::pins

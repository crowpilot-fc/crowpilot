// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// Pin assignments for the WeAct Studio RP2350A_V10.
// Functionally equivalent to the Waveshare RP2350-One (Pi-Pico-form RP2350A
// board). Reference: internal HARDWARE.md table for the canonical mapping.

namespace cp::boards::weact_rp2350a_v10 {

constexpr uint8_t PIN_I2C_SDA       = 4;
constexpr uint8_t PIN_I2C_SCL       = 5;

constexpr uint8_t PIN_MOTOR_RIGHT   = 10;
constexpr uint8_t PIN_MOTOR_LEFT    = 11;

// Servo signal pins, Servo lib. Indices follow the airframe SERVO_*
// constants. The tailsitter uses the first two; the twin-cargo plane uses
// four. The Pi-Pico-form board breaks GP6/GP7 out for servos, with GP8
// and GP9 carrying the two extra plane surfaces.
constexpr uint8_t PIN_SERVOS[4]     = {6, 7, 8, 9};

constexpr uint8_t PIN_SBUS_RX       = 1;

constexpr uint8_t PIN_SD_MOSI       = 19;
constexpr uint8_t PIN_SD_MISO       = 16;
constexpr uint8_t PIN_SD_SCK        = 18;
constexpr uint8_t PIN_SD_CS         = 17;

constexpr uint8_t PIN_LED_STATUS    = 14;
constexpr uint8_t PIN_LED_ONBOARD   = 25;

constexpr uint8_t PIN_COMPANION_TX  = 20;
constexpr uint8_t PIN_COMPANION_RX  = 21;

}  // namespace cp::boards::weact_rp2350a_v10

namespace cp {
using namespace cp::boards::weact_rp2350a_v10;
}  // namespace cp

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// Pin assignments for the Waveshare RP2350-Tiny.
// Reference: internal HARDWARE.md table for the canonical mapping.

namespace cp::boards::waveshare_rp2350_tiny {

constexpr uint8_t PIN_I2C_SDA       = 4;   // I2C0 SDA. IMU + barometer.
constexpr uint8_t PIN_I2C_SCL       = 5;   // I2C0 SCL.

constexpr uint8_t PIN_MOTOR_RIGHT   = 10;  // OneShot125, bit-banged.
constexpr uint8_t PIN_MOTOR_LEFT    = 11;

// Servo signal pins, Servo lib. Indices follow the airframe SERVO_*
// constants. The tailsitter uses the first two (left, right elevon); the
// twin-cargo plane uses four (ailerons, elevator, rudder). GP8 and GP9
// are free on the Tiny and carry the two extra plane surfaces.
constexpr uint8_t PIN_SERVOS[4]     = {12, 13, 8, 9};

constexpr uint8_t PIN_SBUS_RX       = 1;   // PIO inverted UART receive. No external inverter.

constexpr uint8_t PIN_SD_MOSI       = 19;  // SPI0 TX.
constexpr uint8_t PIN_SD_MISO       = 16;  // SPI0 RX.
constexpr uint8_t PIN_SD_SCK        = 18;  // SPI0 SCK.
constexpr uint8_t PIN_SD_CS         = 17;

constexpr uint8_t PIN_LED_STATUS    = 14;  // External 3 mm LED with 470 ohm resistor.
constexpr uint8_t PIN_LED_ONBOARD   = 25;  // RP2350 onboard LED. Boot indicator and panic blink.

constexpr uint8_t PIN_COMPANION_TX  = 20;  // UART1 TX. ESP companion (v1.1) or GPS (v2). Mutually exclusive.
constexpr uint8_t PIN_COMPANION_RX  = 21;  // UART1 RX.

}  // namespace cp::boards::waveshare_rp2350_tiny

// Hoist the pin constants into the global cp:: namespace so module code can
// refer to PIN_* without naming the board. Only one board profile is included
// per build, so there is no collision.
namespace cp {
using namespace cp::boards::waveshare_rp2350_tiny;
}  // namespace cp

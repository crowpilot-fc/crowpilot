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

// ESP passthrough flash control lines. See weact_rp2350a_v10.h for the role.
constexpr uint8_t PIN_ESP_EN        = 22;
constexpr uint8_t PIN_ESP_IO0       = 3;

// Per-channel PWM RC input pins used only when RX_PROTOCOL = RX_PWM.
// Each entry is one receiver channel wired to the listed GPIO. Order is
// (roll, pitch, throttle, yaw / 4th channel). The 4th channel is named
// "yaw" by convention but the firmware reads whatever role CHANNEL_*
// maps to that index. On the F-14 swing-wing build this fourth channel
// drives the wing-sweep servo from user-sketch.ino. Pins picked from
// the Tiny GPIOs that are free with everything else default. Edit if
// your build claims those pins for something else.
constexpr uint8_t PIN_PWM_RX[]      = {6, 7, 15, 26};

}  // namespace cp::boards::waveshare_rp2350_tiny

// Hoist the pin constants into the global cp:: namespace so module code can
// refer to PIN_* without naming the board. Only one board profile is included
// per build, so there is no collision.
namespace cp {
using namespace cp::boards::waveshare_rp2350_tiny;
}  // namespace cp

#define BOARD_HAS_ESP_FLASH 1

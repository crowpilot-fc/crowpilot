// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// Pin assignments for the SmartElex RP2350A NEO development board.
//
// This is a small castellated RP2350A board (Waveshare RP2350-Zero class) that
// breaks out 16 GPIO: GP0, GP1, GP2, GP3, GP14, GP15, GP16, GP17, GP19, GP20,
// GP24, GP25, GP26, GP27, GP28, GP29. It does NOT expose the pins the
// Waveshare RP2350-Tiny profile uses (GP4, GP5, GP8 to GP13, GP18), so it
// needs its own map. 16 GPIO is plenty for a basic plane: IMU, receiver,
// motor, and the surface servos all fit with room to spare.
//
// The onboard LED is a WS2812 (NeoPixel, hence "NEO"), which the simple
// on/off LED HAL does not drive. Use an external status LED on PIN_LED_STATUS.
//
// Pin budget note. With only 16 GPIO, the full output set, SD, and the
// companion UART cannot all coexist. The core plane path (IMU, receiver,
// motor, four servos, status LED, companion UART) is conflict-free. SD logging
// reuses GP2, which is the servo[2] (elevator) slot, so SD works on a
// two-servo airframe such as a flying wing (elevons on GP26/GP27, GP2 free)
// but not alongside a four-servo conventional plane. Keep ENABLE_TELEMETRY_LOG
// off on a four-servo build on this board.

namespace cp::boards::smartelex_rp2350a_neo {

constexpr uint8_t PIN_I2C_SDA       = 28;  // I2C0 SDA. IMU + barometer.
constexpr uint8_t PIN_I2C_SCL       = 29;  // I2C0 SCL.

constexpr uint8_t PIN_MOTOR_RIGHT   = 14;  // PWM / OneShot125 / DShot.
constexpr uint8_t PIN_MOTOR_LEFT    = 15;

// Servo signal pins, Servo lib. Indices follow the airframe SERVO_* constants.
// A flying wing uses the first two (left, right elevon) on GP26/GP27. A
// conventional plane uses all four (ailerons, elevator, rudder); note GP2 is
// shared with PIN_SD_SCK, so do not enable SD logging on a four-servo build.
constexpr uint8_t PIN_SERVOS[4]     = {26, 27, 2, 3};

constexpr uint8_t PIN_SBUS_RX       = 1;   // PIO inverted UART, or CRSF UART0 RX. CRSF TX is GP0.

constexpr uint8_t PIN_SD_MOSI       = 19;  // SPI0 TX.
constexpr uint8_t PIN_SD_MISO       = 16;  // SPI0 RX.
constexpr uint8_t PIN_SD_SCK        = 2;   // SPI0 SCK. Shared with servo[2]; see the budget note.
constexpr uint8_t PIN_SD_CS         = 17;

constexpr uint8_t PIN_LED_STATUS    = 24;  // External 3 mm LED with 470 ohm resistor.
constexpr uint8_t PIN_LED_ONBOARD   = 24;  // Onboard LED is a WS2812, not GPIO-drivable; reuse the external LED.

constexpr uint8_t PIN_COMPANION_TX  = 20;  // UART1 TX. ESP companion or GPS. Mutually exclusive.
constexpr uint8_t PIN_COMPANION_RX  = 25;  // UART1 RX.

}  // namespace cp::boards::smartelex_rp2350a_neo

// Hoist the pin constants into the global cp:: namespace so module code can
// refer to PIN_* without naming the board. Only one board profile is included
// per build, so there is no collision.
namespace cp {
using namespace cp::boards::smartelex_rp2350a_neo;
}  // namespace cp

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// Pin assignments for the Raspberry Pi Pico 2 W (RP2350A + CYW43439 radio).
//
// Same RP2350A package as the WeAct RP2350A_V10 and Waveshare RP2350-One
// profiles, so the flight pins are identical. The difference is the onboard
// radio. The CYW43439 claims GP23, GP24, GP25, and GP29, and the user LED
// sits on the radio chip rather than a plain GPIO. CrowPilot only used GP25
// of those (the onboard LED), so the heartbeat moves to the external status
// LED on GP14.
//
// With ENABLE_ONBOARD_WIFI this board serves the cp protocol and the phone UI
// from its own radio on core1, so a separate ESP companion is not required.
// An ESP companion on the companion UART (GP20 / GP21) still works if you
// prefer to keep the radio stack off the flight MCU.

// Integrated radio. Config.h reads this to default ENABLE_ONBOARD_WIFI on.
#define BOARD_HAS_WIFI 1

namespace cp::boards::pico2w {

constexpr uint8_t PIN_I2C_SDA       = 4;
constexpr uint8_t PIN_I2C_SCL       = 5;

constexpr uint8_t PIN_MOTOR_RIGHT   = 10;
constexpr uint8_t PIN_MOTOR_LEFT    = 11;

// Servo signal pins. Indices follow the airframe SERVO_* constants: the
// tailsitter uses the first two, the twin-cargo plane uses all four.
constexpr uint8_t PIN_SERVOS[4]     = {6, 7, 8, 9};

constexpr uint8_t PIN_SBUS_RX       = 1;

constexpr uint8_t PIN_SD_MOSI       = 19;
constexpr uint8_t PIN_SD_MISO       = 16;
constexpr uint8_t PIN_SD_SCK        = 18;
constexpr uint8_t PIN_SD_CS         = 17;

// The Pico 2 W onboard LED hangs off the CYW43439, not a GPIO, so the
// register-level digitalWriteFast the LED driver uses cannot reach it. The
// boot heartbeat and panic blink therefore use the external status LED on
// GP14. The onboard-WiFi module can blink the radio LED (LED_BUILTIN) as a
// link indicator from core1, separate from this heartbeat.
constexpr uint8_t PIN_LED_STATUS    = 14;
constexpr uint8_t PIN_LED_ONBOARD   = 14;

constexpr uint8_t PIN_COMPANION_TX  = 20;
constexpr uint8_t PIN_COMPANION_RX  = 21;

// ESP passthrough flash control lines. See weact_rp2350a_v10.h for the role.
constexpr uint8_t PIN_ESP_EN        = 22;
constexpr uint8_t PIN_ESP_IO0       = 3;

// Reserved by the CYW43439 radio, do not use for flight or aux:
// GP23 (WL_ON), GP24 (WL_DATA), GP25 (WL_CS), GP29 (WL_CLK / VSYS sense).

}  // namespace cp::boards::pico2w

namespace cp {
using namespace cp::boards::pico2w;
}  // namespace cp

#define BOARD_HAS_ESP_FLASH 1

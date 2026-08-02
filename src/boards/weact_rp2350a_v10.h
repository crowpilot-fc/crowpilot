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

// Optional ESP control lines used by the cp esp flash passthrough path. The
// FC pulses EN to reset the ESP and holds IO0 (GPIO9 on the ESP32-C3) low at
// reset to drop the chip into its UART ROM bootloader, then bridges USB CDC
// bytes to the ESP UART. Wire these two jumpers and the ESP can be flashed
// for the first time and updated forever without a USB cable on the ESP
// itself.
constexpr uint8_t PIN_ESP_EN        = 22;
constexpr uint8_t PIN_ESP_IO0       = 3;

// 2-axis pan/tilt gimbal servo outputs, used when GIMBAL_ENABLE = 1. GP27
// and GP28 are the last two free ADC-capable pins on this board once the
// eight PWM receiver inputs below are placed. The Servo lib attaches them
// only when the gimbal is enabled, so they stay free GPIOs otherwise.
//
// GP28 is also BATTERY_ADC_PIN's default. ENABLE_BATTERY_MONITOR and the
// gimbal cannot both use it; the static_assert in Config.h catches the
// clash. Move the battery divider to another ADC pin to run both.
constexpr uint8_t PIN_GIMBAL_PAN    = 27;
constexpr uint8_t PIN_GIMBAL_TILT   = 28;
#define BOARD_HAS_GIMBAL 1

// Per-channel PWM RC input pins used only when RX_PROTOCOL = RX_PWM. Each
// entry is one receiver channel wired to the listed GPIO. Order follows the
// CHANNEL_* role map: roll, pitch, throttle, yaw, then four aux.
//
// Eight entries so a conventional plane can carry AETR plus a head-tracker
// pan/tilt pair plus a flight-mode switch and an arm switch, which is the
// full channel budget of a common 8-channel PWM receiver.
//
// Unlike the Tiny, none of these collide with SD logging (GP16-GP19) or the
// companion UART (GP20/GP21), so a PWM build on this board keeps both flight
// logging and the Crowpilot Companion app. That is the reason to prefer this
// board over the Tiny for a PWM airframe.
//
// Two shared pins to know about:
//   GP1 is PIN_SBUS_RX, unused in a PWM build, so it is free here.
//   GP3 is PIN_ESP_IO0. The firmware only drives ESP_IO0 during a one-shot
//   ESP passthrough flash, never in flight, so the collision is benign. Do
//   not run cp esp flash with the receiver connected.
constexpr uint8_t PIN_PWM_RX[]      = {0, 1, 2, 3, 12, 13, 15, 26};

}  // namespace cp::boards::weact_rp2350a_v10

namespace cp {
using namespace cp::boards::weact_rp2350a_v10;
}  // namespace cp

// This board breaks out enough free GPIO for the ESP passthrough flash path.
#define BOARD_HAS_ESP_FLASH 1

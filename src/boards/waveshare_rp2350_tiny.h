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

// SD pins remapped to the top-edge castellations. The default SPI0 pin
// set on the rp2040 Arduino core lands on GP16-GP19, which on the
// Waveshare Tiny are bottom-side SMD pads (no through-hole, no
// castellation). Mounting an SD card to those pins requires reflow or
// fine-pitch magnet-wire work, which is impractical for a hand-built
// FC. The four pins below all sit on the top-edge castellations and
// solder cleanly to 0.1" perfboard.
//
// On the RP2350 SPI0 has multiple alt-function banks. The Tiny exposes
// these top-edge pins: RX on GP0 or GP4, SCK on GP2 or GP6, TX on GP3
// or GP7. The chosen set avoids GP3 (PIN_ESP_IO0) and GP4/GP5 (I2C),
// so SD logging is compatible with ESP companion flash and the IMU on
// the same build.
//
// PIN_SD_MOSI = GP7 collides with PIN_PWM_RX[1].
// PIN_SD_CS   = GP6 collides with PIN_PWM_RX[0].
// SD logging and PWM receiver are therefore MUTUALLY EXCLUSIVE on this
// board. (Picking SBUS, CRSF, or PPM for RX avoids the conflict.) The
// static_assert at the end of Config.h catches the conflict at build
// time if both are enabled.
//
// The firmware calls SPI.setRX/setTX/setSCK with these constants before
// SD.begin() so the remap actually takes effect (see SdLogger.cpp).
constexpr uint8_t PIN_SD_MOSI       = 7;   // SPI0 TX bank-1 alt. Conflicts with PIN_PWM_RX[1].
constexpr uint8_t PIN_SD_MISO       = 0;   // SPI0 RX bank-0 alt.
constexpr uint8_t PIN_SD_SCK        = 2;   // SPI0 SCK bank-0 alt.
constexpr uint8_t PIN_SD_CS         = 6;   // Software-driven CS. Conflicts with PIN_PWM_RX[0].

// The Waveshare RP2350-Tiny does NOT break out GP24 or GP25 as castellated
// pins. They are only available on the user-mode boot-strap SMD pads and
// carry no onboard LED. The Tiny's actual onboard LED is a WS2812 RGB on
// GP16, which is on a bottom-side SMD pad and uses serial colour data
// rather than digitalWrite, so it cannot serve as the firmware heartbeat
// without a WS2812 driver. For v1.0 the heartbeat and panic blink both
// drive a single external 3 mm LED on the GP14 castellated pin. The
// builder wires a 470 ohm series resistor to a small panel-mount LED on
// the airframe. The two constants point to the same pin so the rest of
// the firmware that references either keeps working.
constexpr uint8_t PIN_LED_STATUS    = 14;
constexpr uint8_t PIN_LED_ONBOARD   = 14;

// Companion UART pins. On the Tiny, the hardware UART alt-function pins
// (UART1: GP4/5, GP8/9, GP12/13, GP20/21) are either taken by I2C, the
// servos, or sit on bottom-side SMD pads. The companion UART is therefore
// routed via SerialPIO (the rp2040 Arduino core's PIO-based UART) on two
// free top-edge castellations. GP15 carries Tiny TX (to ESP RX), GP26
// carries Tiny RX (from ESP TX). Both are clear in SBUS / CRSF builds;
// they conflict with PIN_PWM_RX[2] and PIN_PWM_RX[3] when RX_PROTOCOL is
// RX_PWM, so PWM-RX builds must drop ENABLE_COMPANION_CLI or remap the
// PWM pins.
constexpr uint8_t PIN_COMPANION_TX  = 15;  // SerialPIO TX. Top-edge castellated.
constexpr uint8_t PIN_COMPANION_RX  = 26;  // SerialPIO RX. Top-edge castellated.
#define BOARD_COMPANION_USES_SERIALPIO 1

// ESP passthrough flash control lines. See weact_rp2350a_v10.h for the role.
constexpr uint8_t PIN_ESP_EN        = 22;
constexpr uint8_t PIN_ESP_IO0       = 3;

// 2-axis pan/tilt gimbal servo outputs, used when GIMBAL_ENABLE = 1.
// GP3 and GP28 are free in the default SBUS build (GP3 doubles as ESP_IO0
// only during ESP passthrough flash; GP28 is otherwise spare unless the
// battery monitor claims it). The Servo lib attaches these pins only when
// the gimbal is enabled, so they remain free GPIOs in builds that do not
// use the gimbal feature.
constexpr uint8_t PIN_GIMBAL_PAN    = 3;
constexpr uint8_t PIN_GIMBAL_TILT   = 28;

// Per-channel PWM RC input pins used only when RX_PROTOCOL = RX_PWM.
// Each entry is one receiver channel wired to the listed GPIO. Order is
// (roll, pitch, throttle, yaw, aux). The fifth channel is typically a
// mode-switch (e.g. STAB/MANUAL) but the firmware reads whatever role
// CHANNEL_* maps to that index. Five entries chosen so a single
// experimental board can carry a 4-channel AETR receiver plus one
// auxiliary switch channel, the most common conventional-plane setup.
// Pins picked from Tiny GPIOs that are free with default features.
//
// Conflicts when RX_PROTOCOL = RX_PWM is selected:
//   GP3  (PIN_PWM_RX[4]) collides with PIN_ESP_IO0  (ESP companion flash)
//   GP6  (PIN_PWM_RX[0]) collides with PIN_SD_CS    (SD logging)
//   GP7  (PIN_PWM_RX[1]) collides with PIN_SD_MOSI  (SD logging)
//   GP15 (PIN_PWM_RX[2]) collides with PIN_COMPANION_TX (ESP UART telemetry)
//   GP26 (PIN_PWM_RX[3]) collides with PIN_COMPANION_RX (ESP UART telemetry)
// Static_asserts in Config.h catch the SD and companion conflicts at
// build time. ESP flash collision on GP3 is benign because the firmware
// only drives ESP_IO0 during the brief one-shot passthrough flash, not
// during normal flight.
constexpr uint8_t PIN_PWM_RX[]      = {6, 7, 15, 26, 3};

}  // namespace cp::boards::waveshare_rp2350_tiny

// Hoist the pin constants into the global cp:: namespace so module code can
// refer to PIN_* without naming the board. Only one board profile is included
// per build, so there is no collision.
namespace cp {
using namespace cp::boards::waveshare_rp2350_tiny;
}  // namespace cp

#define BOARD_HAS_ESP_FLASH 1

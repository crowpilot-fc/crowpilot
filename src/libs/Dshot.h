// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// DShot frame builder. DShot is a digital ESC protocol: instead of an
// analog pulse width, the flight controller clocks out a 16-bit frame per
// motor update. The bits are pulse-width-encoded on the wire (a long high
// is a 1, a short high is a 0) and timed by a PIO state machine in the
// native HAL. This file only builds and validates the frame value, so it
// is pure integer code with no hardware dependency and is host-testable.
//
// Frame layout, most significant bit first:
//   bits 15..5  11-bit throttle or command value
//   bit  4      telemetry request
//   bits 3..0   CRC, the low nibble of value ^ (value >> 4) ^ (value >> 8)
//               over the 12-bit throttle-plus-telemetry field
//
// Throttle values: 0 is the motor-stop command. 1 through 47 are reserved
// command codes (beeps, direction, save settings) not used in v1. 48
// through 2047 are the 2000-step throttle band. This is standard
// (non-inverted-CRC) DShot. Bidirectional DShot, which inverts the CRC to
// free the line for ESC telemetry, is a later addition.
//
// Protocol reference cited in SPEC.md. No source code from other flight
// controller projects consulted.

namespace cp::libs::dshot {

constexpr uint16_t kCmdMotorStop = 0;     // value 0: motors stop
constexpr uint16_t kThrottleMin  = 48;    // first armed throttle step
constexpr uint16_t kThrottleMax  = 2047;  // full throttle, 11-bit max

// Build the 16-bit DShot frame from an 11-bit throttle-or-command value
// and the telemetry-request bit. The value is masked to 11 bits. The CRC
// is computed and packed into the low nibble.
uint16_t buildFrame(uint16_t value11, bool telemetry);

// Map a motor pulse width, in the firmware's standard microsecond
// abstraction, to a DShot throttle value. A width below idle_us (the
// disarm pulse) returns the motor-stop command 0. The [idle_us, max_us]
// band maps linearly onto [kThrottleMin, kThrottleMax], clamped.
uint16_t microsecondsToThrottle(uint16_t pulse_us, uint16_t idle_us,
                                uint16_t max_us);

}  // namespace cp::libs::dshot

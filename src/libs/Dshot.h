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

// ---------------------------------------------------------------------------
// Bidirectional DShot
// ---------------------------------------------------------------------------
// In bidirectional DShot the command frame is the same 16 bits but the CRC is
// inverted, and the physical line is inverted (idle high). The inverted CRC
// tells the ESC to reply, after the frame, with a GCR-encoded eRPM telemetry
// packet on the same wire. The line inversion is handled by the PIO program.
// These helpers cover the bit-level encode and decode, which is pure integer
// code and host-tested. The PIO turnaround and bit sampling are verified on
// the bench.

// Build a bidirectional-DShot command frame: same value, telemetry bit clear,
// CRC inverted.
uint16_t buildFrameBidir(uint16_t value11);

// Undo the differential line coding the ESC uses for the telemetry reply:
// each set bit in the running XOR marks a level change. Returns the GCR value.
uint32_t gcrDifferentialDecode(uint32_t raw);

// Map one 5-bit GCR quintet to its 4-bit nibble. Returns -1 for an invalid
// quintet (a corrupted reply).
int gcrQuintetToNibble(uint8_t quintet);

// Decode a 20-bit GCR telemetry value (four quintets) to an electrical RPM.
// Maps the quintets to a 16-bit value, verifies the 4-bit checksum, expands
// the 12-bit period field (3-bit exponent, 9-bit mantissa) to microseconds,
// and converts to eRPM. Returns the eRPM (0 means stopped), or -1 on an
// invalid quintet or a failed checksum.
int32_t decodeErpm(uint32_t gcr20);

}  // namespace cp::libs::dshot

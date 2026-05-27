// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// CRSF (Crossfire / ELRS) frame parser. Reads bytes one at a time, syncs on
// the flight-controller address, length-frames the packet, checks the CRC-8,
// and decodes the RC-channels frame into 16 channels. CRSF is a plain
// 420 kbaud UART protocol, not inverted, so unlike SBUS it needs no PIO.
//
// Frame layout: [address][length][type][payload...][crc8]. `length` counts
// the bytes after itself (type + payload + crc). The CRC-8 (poly 0xD5,
// DVB-S2) covers the type and payload. The RC-channels frame is type 0x16
// with a 22-byte payload of sixteen 11-bit channels, LSB first.
//
// Protocol reference cited in SPEC.md. No source code from other flight
// controller projects consulted.

namespace cp::libs::crsf {

constexpr uint8_t kNumChannels = 16;

struct DecodedFrame {
  uint16_t channel[kNumChannels];  // 11-bit raw, range 0..2047
};

// Reset the parser so the next call to feed re-syncs.
void reset();

// Feed one received byte. Returns true exactly when a well-formed
// RC-channels frame just completed and was CRC-checked; the caller reads the
// decoded channels from out. Telemetry and other frame types return false
// (they are not errors, just not RC).
bool feed(uint8_t byte, DecodedFrame& out);

// Count of frames dropped on a bad CRC. Each one also re-syncs the parser.
uint32_t lostFrameCount();

// Linear raw to microseconds map. CRSF shares the SBUS 11-bit scaling
// centred at 992, so 992 -> 1500 us, with 172 and 1811 near 1000 and 2000.
uint16_t rawToMicroseconds(uint16_t raw);

}  // namespace cp::libs::crsf

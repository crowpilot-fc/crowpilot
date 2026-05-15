// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// SBUS frame parser. Reads bytes one at a time, assembles 25-byte frames,
// validates the footer, and decodes 16 channels plus two digital channels
// and two status flags per the Futaba S.Bus spec. Implementation follows
// ALGORITHMS.md §3.2 through §3.5. Protocol reference cited in SPEC.md
// §16.3. No source code from other flight controller projects consulted.

namespace cp::libs::sbus {

constexpr uint8_t kNumChannels = 16;

struct DecodedFrame {
  uint16_t channel[kNumChannels];  // 11-bit raw, range 0..2047
  bool     ch17;
  bool     ch18;
  bool     frame_lost;
  bool     failsafe;
};

// Reset the internal byte buffer so the next call to feed re-syncs on
// the next 0x0F header.
void reset();

// Feed one byte from the receive path. Returns true exactly when a
// well-formed frame (header 0x0F, footer 0x00) just completed; the
// caller reads the decoded values from out.
bool feed(uint8_t byte, DecodedFrame& out);

// Count of frames where the footer was wrong. Each bad footer also
// resets the parser. Monotonically increasing across the firmware's
// lifetime.
uint32_t lostFrameCount();

// Linear raw to microseconds map per ALGORITHMS.md §3.4.
// raw 192 -> 1000 us, raw 992 -> 1500 us, raw 1792 -> 2000 us.
uint16_t rawToMicroseconds(uint16_t raw);

}  // namespace cp::libs::sbus

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/libs/Sbus.h"

namespace cp::libs::sbus {

namespace {

constexpr uint8_t kFrameSize  = 25;
constexpr uint8_t kHeaderByte = 0x0F;

uint8_t  s_buffer[kFrameSize];
uint8_t  s_buffer_len      = 0;
uint32_t s_lost_frame_count = 0;

// Channel bit unpack per ALGORITHMS.md §3.2. Each channel is an 11-bit
// value drawn from a packed sequence of bytes 1 through 22 of the frame.
void decodeChannels(DecodedFrame& out) {
  const uint8_t* d = s_buffer;
  // Promote each byte to uint32_t before shifting so the >= 8 bit shifts
  // do not invoke implementation-defined int promotion.
  const uint32_t b1  = d[1];
  const uint32_t b2  = d[2];
  const uint32_t b3  = d[3];
  const uint32_t b4  = d[4];
  const uint32_t b5  = d[5];
  const uint32_t b6  = d[6];
  const uint32_t b7  = d[7];
  const uint32_t b8  = d[8];
  const uint32_t b9  = d[9];
  const uint32_t b10 = d[10];
  const uint32_t b11 = d[11];
  const uint32_t b12 = d[12];
  const uint32_t b13 = d[13];
  const uint32_t b14 = d[14];
  const uint32_t b15 = d[15];
  const uint32_t b16 = d[16];
  const uint32_t b17 = d[17];
  const uint32_t b18 = d[18];
  const uint32_t b19 = d[19];
  const uint32_t b20 = d[20];
  const uint32_t b21 = d[21];
  const uint32_t b22 = d[22];

  out.channel[ 0] = static_cast<uint16_t>(((b1)         | (b2  << 8))                    & 0x07FFU);
  out.channel[ 1] = static_cast<uint16_t>(((b2  >> 3)   | (b3  << 5))                    & 0x07FFU);
  out.channel[ 2] = static_cast<uint16_t>(((b3  >> 6)   | (b4  << 2)  | (b5  << 10))     & 0x07FFU);
  out.channel[ 3] = static_cast<uint16_t>(((b5  >> 1)   | (b6  << 7))                    & 0x07FFU);
  out.channel[ 4] = static_cast<uint16_t>(((b6  >> 4)   | (b7  << 4))                    & 0x07FFU);
  out.channel[ 5] = static_cast<uint16_t>(((b7  >> 7)   | (b8  << 1)  | (b9  << 9))      & 0x07FFU);
  out.channel[ 6] = static_cast<uint16_t>(((b9  >> 2)   | (b10 << 6))                    & 0x07FFU);
  out.channel[ 7] = static_cast<uint16_t>(((b10 >> 5)   | (b11 << 3))                    & 0x07FFU);
  out.channel[ 8] = static_cast<uint16_t>(((b12)        | (b13 << 8))                    & 0x07FFU);
  out.channel[ 9] = static_cast<uint16_t>(((b13 >> 3)   | (b14 << 5))                    & 0x07FFU);
  out.channel[10] = static_cast<uint16_t>(((b14 >> 6)   | (b15 << 2)  | (b16 << 10))     & 0x07FFU);
  out.channel[11] = static_cast<uint16_t>(((b16 >> 1)   | (b17 << 7))                    & 0x07FFU);
  out.channel[12] = static_cast<uint16_t>(((b17 >> 4)   | (b18 << 4))                    & 0x07FFU);
  out.channel[13] = static_cast<uint16_t>(((b18 >> 7)   | (b19 << 1)  | (b20 << 9))      & 0x07FFU);
  out.channel[14] = static_cast<uint16_t>(((b20 >> 2)   | (b21 << 6))                    & 0x07FFU);
  out.channel[15] = static_cast<uint16_t>(((b21 >> 5)   | (b22 << 3))                    & 0x07FFU);
}

void decodeFlags(DecodedFrame& out) {
  // Flag byte sits at index 23. Bits 0, 1 are ch17 and ch18 (digital).
  // Bit 2 is the frame-lost transient flag; bit 3 is the receiver-level
  // failsafe flag. Bits 4 through 7 are reserved zero.
  const uint8_t flags = s_buffer[23];
  out.ch17       = (flags & 0x01) != 0;
  out.ch18       = (flags & 0x02) != 0;
  out.frame_lost = (flags & 0x04) != 0;
  out.failsafe   = (flags & 0x08) != 0;
}

}  // anonymous namespace

void reset() {
  s_buffer_len = 0;
}

bool feed(uint8_t byte, DecodedFrame& out) {
  if (s_buffer_len == 0) {
    if (byte != kHeaderByte) {
      return false;
    }
    s_buffer[0]  = byte;
    s_buffer_len = 1;
    return false;
  }

  s_buffer[s_buffer_len++] = byte;
  if (s_buffer_len < kFrameSize) {
    return false;
  }

  s_buffer_len = 0;
  // End byte. A standard SBUS1 frame ends in 0x00. SBUS2 receivers send
  // a non-zero end byte (0x04, 0x14, 0x24, 0x34, low nibble 0x4) to
  // carry telemetry-slot information. Accept both, otherwise an SBUS2
  // receiver looks like an unbroken run of lost frames and trips the
  // link-loss failsafe.
  const uint8_t end_byte = s_buffer[kFrameSize - 1];
  if (end_byte != 0x00 && (end_byte & 0x0F) != 0x04) {
    ++s_lost_frame_count;
    return false;
  }

  decodeChannels(out);
  decodeFlags(out);
  return true;
}

uint32_t lostFrameCount() {
  return s_lost_frame_count;
}

uint16_t rawToMicroseconds(uint16_t raw) {
  return static_cast<uint16_t>((static_cast<uint32_t>(raw) * 5U) / 8U + 880U);
}

}  // namespace cp::libs::sbus

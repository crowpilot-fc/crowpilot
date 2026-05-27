// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/libs/Crsf.h"

namespace cp::libs::crsf {

namespace {

constexpr uint8_t kAddrFlightController = 0xC8;  // sync byte for FC-bound frames
constexpr uint8_t kTypeRcChannels       = 0x16;  // RC_CHANNELS_PACKED
constexpr uint8_t kMaxLength            = 62;    // largest valid length byte
constexpr uint8_t kRcPayloadBytes       = 22;    // 16 channels x 11 bits

// Buffer holds address, length, then up to kMaxLength bytes (type, payload,
// crc): 2 + 62 = 64 at most.
uint8_t  s_buffer[2 + kMaxLength];
uint8_t  s_len             = 0;  // bytes accumulated so far
uint32_t s_lost_frame_count = 0;

// CRC-8 DVB-S2, polynomial 0xD5, over the type and payload bytes.
uint8_t crc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                         : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

// Unpack the 22-byte RC payload into sixteen 11-bit channels, LSB first.
// Same bit layout as SBUS, addressed from the payload start.
void decodeChannels(const uint8_t* p, DecodedFrame& out) {
  const uint32_t p0  = p[0];
  const uint32_t p1  = p[1];
  const uint32_t p2  = p[2];
  const uint32_t p3  = p[3];
  const uint32_t p4  = p[4];
  const uint32_t p5  = p[5];
  const uint32_t p6  = p[6];
  const uint32_t p7  = p[7];
  const uint32_t p8  = p[8];
  const uint32_t p9  = p[9];
  const uint32_t p10 = p[10];
  const uint32_t p11 = p[11];
  const uint32_t p12 = p[12];
  const uint32_t p13 = p[13];
  const uint32_t p14 = p[14];
  const uint32_t p15 = p[15];
  const uint32_t p16 = p[16];
  const uint32_t p17 = p[17];
  const uint32_t p18 = p[18];
  const uint32_t p19 = p[19];
  const uint32_t p20 = p[20];
  const uint32_t p21 = p[21];

  out.channel[ 0] = static_cast<uint16_t>(((p0)        | (p1  << 8))                 & 0x07FFU);
  out.channel[ 1] = static_cast<uint16_t>(((p1  >> 3)  | (p2  << 5))                 & 0x07FFU);
  out.channel[ 2] = static_cast<uint16_t>(((p2  >> 6)  | (p3  << 2)  | (p4  << 10))  & 0x07FFU);
  out.channel[ 3] = static_cast<uint16_t>(((p4  >> 1)  | (p5  << 7))                 & 0x07FFU);
  out.channel[ 4] = static_cast<uint16_t>(((p5  >> 4)  | (p6  << 4))                 & 0x07FFU);
  out.channel[ 5] = static_cast<uint16_t>(((p6  >> 7)  | (p7  << 1)  | (p8  << 9))   & 0x07FFU);
  out.channel[ 6] = static_cast<uint16_t>(((p8  >> 2)  | (p9  << 6))                 & 0x07FFU);
  out.channel[ 7] = static_cast<uint16_t>(((p9  >> 5)  | (p10 << 3))                 & 0x07FFU);
  out.channel[ 8] = static_cast<uint16_t>(((p11)       | (p12 << 8))                 & 0x07FFU);
  out.channel[ 9] = static_cast<uint16_t>(((p12 >> 3)  | (p13 << 5))                 & 0x07FFU);
  out.channel[10] = static_cast<uint16_t>(((p13 >> 6)  | (p14 << 2)  | (p15 << 10))  & 0x07FFU);
  out.channel[11] = static_cast<uint16_t>(((p15 >> 1)  | (p16 << 7))                 & 0x07FFU);
  out.channel[12] = static_cast<uint16_t>(((p16 >> 4)  | (p17 << 4))                 & 0x07FFU);
  out.channel[13] = static_cast<uint16_t>(((p17 >> 7)  | (p18 << 1)  | (p19 << 9))   & 0x07FFU);
  out.channel[14] = static_cast<uint16_t>(((p19 >> 2)  | (p20 << 6))                 & 0x07FFU);
  out.channel[15] = static_cast<uint16_t>(((p20 >> 5)  | (p21 << 3))                 & 0x07FFU);
}

}  // anonymous namespace

void reset() {
  s_len = 0;
}

bool feed(uint8_t byte, DecodedFrame& out) {
  // Sync on the flight-controller address.
  if (s_len == 0) {
    if (byte != kAddrFlightController) {
      return false;
    }
    s_buffer[0] = byte;
    s_len = 1;
    return false;
  }

  // Length byte. It counts type + payload + crc, so 2..62.
  if (s_len == 1) {
    if (byte < 2 || byte > kMaxLength) {
      s_len = 0;  // implausible length, re-sync
      return false;
    }
    s_buffer[1] = byte;
    s_len = 2;
    return false;
  }

  s_buffer[s_len++] = byte;

  const uint8_t length      = s_buffer[1];
  const uint8_t frame_total = static_cast<uint8_t>(2 + length);
  if (s_len < frame_total) {
    return false;
  }

  // Full frame. The CRC covers the type and payload (length - 1 bytes), and
  // the last byte is the received CRC.
  s_len = 0;
  const uint8_t crc_received = s_buffer[frame_total - 1];
  if (crc8(&s_buffer[2], static_cast<uint8_t>(length - 1)) != crc_received) {
    ++s_lost_frame_count;
    return false;
  }

  // Only the RC-channels frame carries sticks. Telemetry and link-stats
  // frames are valid but not RC, so report no new RC frame.
  const uint8_t type = s_buffer[2];
  if (type != kTypeRcChannels || length < (1 + kRcPayloadBytes + 1)) {
    return false;
  }

  decodeChannels(&s_buffer[3], out);
  return true;
}

uint32_t lostFrameCount() {
  return s_lost_frame_count;
}

uint16_t rawToMicroseconds(uint16_t raw) {
  return static_cast<uint16_t>((static_cast<uint32_t>(raw) * 5U) / 8U + 880U);
}

}  // namespace cp::libs::crsf

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/libs/Dshot.h"

namespace cp::libs::dshot {

uint16_t buildFrame(uint16_t value11, bool telemetry) {
  // 11-bit value plus the telemetry bit makes the 12-bit field the CRC
  // protects.
  const uint16_t value     = static_cast<uint16_t>(value11 & 0x07FFU);
  const uint16_t with_tlm  = static_cast<uint16_t>((value << 1) |
                                                   (telemetry ? 1U : 0U));
  const uint16_t crc =
      static_cast<uint16_t>((with_tlm ^ (with_tlm >> 4) ^ (with_tlm >> 8)) &
                            0x0FU);
  return static_cast<uint16_t>((with_tlm << 4) | crc);
}

uint16_t microsecondsToThrottle(uint16_t pulse_us, uint16_t idle_us,
                                uint16_t max_us) {
  if (pulse_us < idle_us) {
    return kCmdMotorStop;
  }
  if (pulse_us >= max_us) {
    return kThrottleMax;
  }
  // Linear map [idle_us, max_us] -> [kThrottleMin, kThrottleMax].
  const uint32_t span_us   = static_cast<uint32_t>(max_us - idle_us);
  const uint32_t span_thr  = static_cast<uint32_t>(kThrottleMax - kThrottleMin);
  const uint32_t offset_us = static_cast<uint32_t>(pulse_us - idle_us);
  const uint32_t thr =
      kThrottleMin + (offset_us * span_thr + span_us / 2U) / span_us;
  return static_cast<uint16_t>(thr);
}

uint16_t buildFrameBidir(uint16_t value11) {
  const uint16_t value    = static_cast<uint16_t>(value11 & 0x07FFU);
  const uint16_t with_tlm = static_cast<uint16_t>(value << 1);  // tlm bit clear
  // Same CRC as the normal frame, then inverted: the inverted CRC is what
  // requests the eRPM reply.
  const uint16_t crc =
      static_cast<uint16_t>((~(with_tlm ^ (with_tlm >> 4) ^ (with_tlm >> 8))) &
                            0x0FU);
  return static_cast<uint16_t>((with_tlm << 4) | crc);
}

uint32_t gcrDifferentialDecode(uint32_t raw) {
  return raw ^ (raw >> 1);
}

int gcrQuintetToNibble(uint8_t quintet) {
  // The fixed bidirectional-DShot GCR alphabet: each nibble has one valid
  // 5-bit code. Any other quintet is a corrupted reply.
  switch (quintet & 0x1FU) {
    case 0x19: return 0x0;
    case 0x1B: return 0x1;
    case 0x12: return 0x2;
    case 0x13: return 0x3;
    case 0x1D: return 0x4;
    case 0x15: return 0x5;
    case 0x16: return 0x6;
    case 0x17: return 0x7;
    case 0x1A: return 0x8;
    case 0x09: return 0x9;
    case 0x0A: return 0xA;
    case 0x0B: return 0xB;
    case 0x1E: return 0xC;
    case 0x0D: return 0xD;
    case 0x0E: return 0xE;
    case 0x0F: return 0xF;
    default:   return -1;
  }
}

int32_t decodeErpm(uint32_t gcr20) {
  // Four 5-bit quintets, most significant first, to a 16-bit value.
  uint16_t value = 0;
  for (int q = 3; q >= 0; --q) {
    const uint8_t quintet = static_cast<uint8_t>((gcr20 >> (q * 5)) & 0x1FU);
    const int     nibble  = gcrQuintetToNibble(quintet);
    if (nibble < 0) {
      return -1;
    }
    value = static_cast<uint16_t>((value << 4) | static_cast<uint16_t>(nibble));
  }

  // 4-bit checksum: folding all four nibbles together must give 0xF.
  const uint16_t fold = static_cast<uint16_t>(value ^ (value >> 4) ^
                                              (value >> 8) ^ (value >> 12));
  if ((fold & 0x0FU) != 0x0FU) {
    return -1;
  }

  // Top 12 bits are the period field: 3-bit exponent, 9-bit mantissa.
  const uint16_t tel12 = static_cast<uint16_t>(value >> 4);
  if (tel12 == 0x0FFFU) {
    return 0;  // ESC reports the motor stopped.
  }
  const uint32_t period_us = static_cast<uint32_t>(tel12 & 0x01FFU)
                             << (tel12 >> 9);
  if (period_us == 0) {
    return -1;
  }
  // eRPM = 60e6 / period_us, rounded.
  return static_cast<int32_t>((60000000U + period_us / 2U) / period_us);
}

}  // namespace cp::libs::dshot

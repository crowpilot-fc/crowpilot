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

}  // namespace cp::libs::dshot

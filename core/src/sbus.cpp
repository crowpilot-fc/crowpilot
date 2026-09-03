// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
#include "crowpilot/sbus.hpp"

namespace crowpilot {
namespace {

constexpr std::uint8_t kStartByte = 0x0f;
constexpr std::size_t kFlagsIndex = 23;
constexpr std::size_t kEndIndex = 24;

}  // namespace

bool SbusParser::push(const std::uint8_t byte,
                      const std::uint64_t received_at_us,
                      SbusFrame& completed_frame) {
    if (size_ == 0U && byte != kStartByte) {
        return false;
    }

    bytes_[size_] = byte;
    ++size_;

    if (size_ < kSbusFrameSize) {
        return false;
    }

    const bool complete = bytes_[0] == kStartByte &&
                          valid_end_byte(bytes_[kEndIndex]);
    if (complete) {
        decode(bytes_, received_at_us, completed_frame);
    }

    reset();
    return complete;
}

void SbusParser::reset() {
    size_ = 0;
}

bool SbusParser::valid_end_byte(const std::uint8_t byte) {
    return byte == 0x00U || byte == 0x04U || byte == 0x14U ||
           byte == 0x24U || byte == 0x34U;
}

void SbusParser::decode(
    const std::array<std::uint8_t, kSbusFrameSize>& bytes,
    const std::uint64_t received_at_us,
    SbusFrame& frame) {
    for (std::size_t channel = 0; channel < kSbusChannelCount; ++channel) {
        const std::size_t first_bit = channel * 11U;
        std::uint16_t value = 0;
        for (std::size_t bit = 0; bit < 11U; ++bit) {
            const std::size_t payload_bit = first_bit + bit;
            const std::size_t byte_index = 1U + payload_bit / 8U;
            const std::size_t bit_index = payload_bit % 8U;
            const auto bit_value = static_cast<std::uint16_t>(
                (bytes[byte_index] >> bit_index) & 0x01U);
            value = static_cast<std::uint16_t>(value | (bit_value << bit));
        }
        frame.channels[channel] = value;
    }

    const std::uint8_t flags = bytes[kFlagsIndex];
    frame.digital_channel_17 = (flags & 0x01U) != 0U;
    frame.digital_channel_18 = (flags & 0x02U) != 0U;
    frame.frame_lost = (flags & 0x04U) != 0U;
    frame.failsafe = (flags & 0x08U) != 0U;
    frame.received_at_us = received_at_us;
}

bool sbus_frame_healthy(const SbusFrame& frame,
                        const std::uint64_t now_us,
                        const std::uint64_t timeout_us) {
    if (frame.frame_lost || frame.failsafe || now_us < frame.received_at_us) {
        return false;
    }
    return (now_us - frame.received_at_us) <= timeout_us;
}

}  // namespace crowpilot

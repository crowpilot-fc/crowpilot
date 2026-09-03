// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace crowpilot {

inline constexpr std::size_t kSbusChannelCount = 16;
inline constexpr std::size_t kSbusFrameSize = 25;
inline constexpr std::uint16_t kSbusNominalMin = 172;
inline constexpr std::uint16_t kSbusNominalMax = 1811;

struct SbusFrame {
    std::array<std::uint16_t, kSbusChannelCount> channels{};
    bool digital_channel_17 = false;
    bool digital_channel_18 = false;
    bool frame_lost = false;
    bool failsafe = false;
    std::uint64_t received_at_us = 0;
};

class SbusParser {
public:
    [[nodiscard]] bool push(std::uint8_t byte,
                            std::uint64_t received_at_us,
                            SbusFrame& completed_frame);
    void reset();

private:
    [[nodiscard]] static bool valid_end_byte(std::uint8_t byte);
    static void decode(const std::array<std::uint8_t, kSbusFrameSize>& bytes,
                       std::uint64_t received_at_us,
                       SbusFrame& frame);

    std::array<std::uint8_t, kSbusFrameSize> bytes_{};
    std::size_t size_ = 0;
};

[[nodiscard]] bool sbus_frame_healthy(const SbusFrame& frame,
                                      std::uint64_t now_us,
                                      std::uint64_t timeout_us);

}  // namespace crowpilot

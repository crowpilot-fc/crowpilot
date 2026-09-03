// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
#pragma once

#include "crowpilot/sbus.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace crowpilot {

inline constexpr std::size_t kOutputCount = 8;

enum class OutputKind : std::uint8_t {
    motor,
    servo,
};

struct OutputRoute {
    std::uint8_t source_channel = 0;
    OutputKind kind = OutputKind::servo;
    std::uint16_t minimum_us = 1000;
    std::uint16_t maximum_us = 2000;
    std::uint16_t safe_us = 1500;
    bool reversed = false;
};

using OutputRoutes = std::array<OutputRoute, kOutputCount>;
using OutputPulses = std::array<std::uint16_t, kOutputCount>;

[[nodiscard]] constexpr OutputRoutes trainer_output_routes() {
    return {{{2, OutputKind::motor, 1000, 2000, 1000, false},
             {0, OutputKind::servo, 1000, 2000, 1500, false},
             {0, OutputKind::servo, 1000, 2000, 1500, true},
             {1, OutputKind::servo, 1000, 2000, 1500, false},
             {3, OutputKind::servo, 1000, 2000, 1500, false},
             {5, OutputKind::servo, 1000, 2000, 1500, false},
             {5, OutputKind::servo, 1000, 2000, 1500, true},
             {6, OutputKind::servo, 1000, 2000, 1500, false}}};
}

[[nodiscard]] bool validate_output_routes(const OutputRoutes& routes);
[[nodiscard]] std::uint16_t sbus_to_pulse(std::uint16_t raw,
                                          const OutputRoute& route);
[[nodiscard]] OutputPulses safe_output_pulses(const OutputRoutes& routes);
[[nodiscard]] OutputPulses manual_output_pulses(const SbusFrame& frame,
                                                const OutputRoutes& routes);

}  // namespace crowpilot

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
#include "crowpilot/output.hpp"

#include <algorithm>

namespace crowpilot {

bool validate_output_routes(const OutputRoutes& routes) {
    return std::all_of(routes.begin(), routes.end(), [](const OutputRoute& route) {
        return route.source_channel < kSbusChannelCount &&
               route.minimum_us >= 750U && route.maximum_us <= 2250U &&
               route.minimum_us < route.maximum_us &&
               route.safe_us >= route.minimum_us &&
               route.safe_us <= route.maximum_us &&
               (route.kind != OutputKind::motor || !route.reversed);
    });
}

std::uint16_t sbus_to_pulse(const std::uint16_t raw,
                            const OutputRoute& route) {
    const std::uint16_t clamped =
        std::clamp(raw, kSbusNominalMin, kSbusNominalMax);
    const std::uint32_t input_offset =
        static_cast<std::uint32_t>(clamped - kSbusNominalMin);
    const std::uint32_t input_span =
        static_cast<std::uint32_t>(kSbusNominalMax - kSbusNominalMin);
    const std::uint32_t output_span =
        static_cast<std::uint32_t>(route.maximum_us - route.minimum_us);
    const std::uint32_t scaled =
        static_cast<std::uint32_t>(route.minimum_us) +
        (input_offset * output_span + input_span / 2U) / input_span;
    const std::uint16_t pulse = static_cast<std::uint16_t>(scaled);

    if (!route.reversed) {
        return pulse;
    }
    return static_cast<std::uint16_t>(
        route.maximum_us - (pulse - route.minimum_us));
}

OutputPulses safe_output_pulses(const OutputRoutes& routes) {
    OutputPulses pulses{};
    for (std::size_t index = 0; index < routes.size(); ++index) {
        pulses[index] = routes[index].safe_us;
    }
    return pulses;
}

OutputPulses manual_output_pulses(const SbusFrame& frame,
                                  const OutputRoutes& routes) {
    OutputPulses pulses{};
    for (std::size_t index = 0; index < routes.size(); ++index) {
        const auto channel = static_cast<std::size_t>(routes[index].source_channel);
        pulses[index] = sbus_to_pulse(frame.channels[channel], routes[index]);
    }
    return pulses;
}

}  // namespace crowpilot

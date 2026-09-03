// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
#include "crowpilot/board.hpp"
#include "crowpilot/output.hpp"
#include "crowpilot/pwm_outputs.hpp"
#include "crowpilot/safety.hpp"
#include "crowpilot/sbus.hpp"
#include "crowpilot/sbus_uart.hpp"

#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include <cstdint>
#include <cstdio>

namespace {

constexpr std::uint64_t kReceiverTimeoutUs = 100'000U;
constexpr std::uint64_t kLoopPeriodUs = 2'000U;
constexpr std::size_t kThrottleChannel = 2;
constexpr std::size_t kArmChannel = 4;
constexpr std::uint16_t kThrottleLowThreshold = 300;
constexpr std::uint16_t kArmLowThreshold = 800;
constexpr std::uint16_t kArmHighThreshold = 1200;

bool arm_requested(const crowpilot::SbusFrame& frame) {
    return frame.channels[kArmChannel] >= kArmHighThreshold;
}

bool arm_released(const crowpilot::SbusFrame& frame) {
    return frame.channels[kArmChannel] <= kArmLowThreshold;
}

}  // namespace

int main() {
    stdio_init_all();

    gpio_init(crowpilot::board::kCompanionEnablePin);
    gpio_put(crowpilot::board::kCompanionEnablePin, false);
    gpio_set_dir(crowpilot::board::kCompanionEnablePin, GPIO_OUT);

    gpio_init(crowpilot::board::kStatusLedPin);
    gpio_put(crowpilot::board::kStatusLedPin, false);
    gpio_set_dir(crowpilot::board::kStatusLedPin, GPIO_OUT);

    const auto routes = crowpilot::trainer_output_routes();
    const bool routes_valid = crowpilot::validate_output_routes(routes);
    const auto safe_pulses = crowpilot::safe_output_pulses(routes);

    crowpilot::PwmOutputs outputs;
    const bool outputs_ready = outputs.initialize(safe_pulses);

    crowpilot::SbusUart receiver;
    receiver.initialize();

    crowpilot::SafetyMachine safety;
    crowpilot::SbusFrame frame{};
    bool have_frame = false;

    const bool reset_by_watchdog = watchdog_caused_reboot();
    std::printf("CrowPilot board=%s watchdog_reset=%u\n",
                crowpilot::board::kBoardName,
                reset_by_watchdog ? 1U : 0U);

    watchdog_enable(100U, true);

    while (true) {
        const absolute_time_t tick_start = get_absolute_time();
        if (receiver.poll(frame)) {
            have_frame = true;
        }

        const std::uint64_t now_us = time_us_64();
        const bool receiver_healthy =
            have_frame && crowpilot::sbus_frame_healthy(
                              frame, now_us, kReceiverTimeoutUs);
        const bool requested = receiver_healthy && arm_requested(frame);
        const bool released = !receiver_healthy || arm_released(frame);

        crowpilot::SafetyInputs safety_inputs{};
        safety_inputs.self_tests_complete = true;
        safety_inputs.self_tests_ok = routes_valid && outputs_ready;
        safety_inputs.receiver_healthy = receiver_healthy;
        safety_inputs.arm_requested = requested && !released;
        safety_inputs.throttle_low = receiver_healthy &&
                                     frame.channels[kThrottleChannel] <=
                                         kThrottleLowThreshold;
        static_cast<void>(safety.update(safety_inputs));

        if (safety.output_authority_enabled()) {
            outputs.apply(crowpilot::manual_output_pulses(frame, routes));
        } else {
            outputs.apply(safe_pulses);
        }

        gpio_put(crowpilot::board::kStatusLedPin,
                 safety.state() == crowpilot::FlightState::armed);

        watchdog_update();
        busy_wait_until(delayed_by_us(tick_start, kLoopPeriodUs));
    }
}

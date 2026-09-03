// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
#include "crowpilot/pwm_outputs.hpp"

#include "crowpilot/board.hpp"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include <cstdint>

namespace crowpilot {

bool PwmOutputs::initialize(const OutputPulses& initial_pulses) {
    constexpr std::uint16_t kPeriodUs = 20'000U;
    const float divider =
        static_cast<float>(clock_get_hz(clk_sys)) / 1'000'000.0F;
    if (divider < 1.0F || divider >= 256.0F) {
        return false;
    }

    std::uint32_t slice_mask = 0;
    for (std::size_t index = 0; index < board::kPwmOutputPins.size(); ++index) {
        const unsigned pin = board::kPwmOutputPins[index];
        gpio_init(pin);
        gpio_pull_down(pin);
        gpio_set_dir(pin, GPIO_IN);

        const unsigned slice = pwm_gpio_to_slice_num(pin);
        pwm_config config = pwm_get_default_config();
        pwm_config_set_clkdiv(&config, divider);
        pwm_config_set_wrap(&config, kPeriodUs - 1U);
        pwm_init(slice, &config, false);
        pwm_set_gpio_level(pin, initial_pulses[index]);
        slice_mask |= 1UL << slice;
    }

    for (const unsigned pin : board::kPwmOutputPins) {
        gpio_set_function(pin, GPIO_FUNC_PWM);
    }
    pwm_set_mask_enabled(slice_mask);
    initialized_ = true;
    return true;
}

void PwmOutputs::apply(const OutputPulses& pulses) {
    if (!initialized_) {
        return;
    }
    for (std::size_t index = 0; index < board::kPwmOutputPins.size(); ++index) {
        pwm_set_gpio_level(board::kPwmOutputPins[index], pulses[index]);
    }
}

}  // namespace crowpilot

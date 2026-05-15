// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

namespace cp::hal {

// Configure the status LED pin as a digital output and drive it low.
void led_init();

// Update the status LED based on the current monotonic time in microseconds.
// At Phase 1 this is a 1 Hz heartbeat on the onboard LED.
void led_tick(uint32_t t_now_us);

// Drop into a 5 Hz blink on the onboard LED forever. Called from cp::core::init
// when a mandatory subsystem (currently the IMU) fails to come up. Does not
// return. Uses a busy-wait loop and no calls to delay() per CODING_STANDARDS
// §9.1.
[[noreturn]] void haltWithFastBlink();

}  // namespace cp::hal

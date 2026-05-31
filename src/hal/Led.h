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
// return. Calls delay() inside the loop so USB CDC can drain buffered Serial
// output and so the optional repeat_msg is re-printed at ~1 Hz. The
// repeat_msg is what a bench operator attaching a serial monitor later sees:
// the firmware keeps shouting "I halted because X" forever so the
// diagnostic is never lost to a missed window.
[[noreturn]] void haltWithFastBlink(const char* repeat_msg = nullptr);

// Built-in LED flasher output (aircraft lighting). Configures LED_FLASHER_PIN
// as a digital output, and sets its level. The caller (the flasher module)
// has already applied the pattern and the polarity, so `high` is the literal
// pin level to drive. Only called when ENABLE_LED_FLASHER is set. No-ops on
// the simulated HAL.
void led_flasher_init();
void led_flasher_set(bool high);

}  // namespace cp::hal

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// Built-in LED flasher (aircraft lighting). A switched on/off output on one
// GPIO, blinking in a beacon, strobe, or steady pattern, driven from the main
// loop. It is independent of the user hook, and off by default so the pin
// stays free. The output is a logic signal, so it drives a small 2-pin LED
// through a resistor, a high-power LED through a low-side MOSFET, or a 3-pin
// LED module that takes a signal plus separate power. See Config.h for the
// pin, pattern, polarity, and timing, and docs/developer-guide for wiring.

namespace cp::lighting::flasher {

// Configure the output pin. Call once at startup.
void init();

// Update the output for the current time. `now_us` is the loop's monotonic
// microsecond timestamp. Call every loop tick.
void tick(uint32_t now_us);

// Pure pattern helper, exposed for testing. Returns true when the LED should
// be lit at time `t_ms` for the given pattern (one of the LED_FLASH_* values),
// before the active-high polarity is applied.
bool flasherState(uint8_t pattern, uint32_t t_ms);

}  // namespace cp::lighting::flasher

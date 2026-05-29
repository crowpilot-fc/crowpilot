// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/hal/Hal.h"

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_NATIVE

#include <Arduino.h>

// Native analog input. The RP2350 ADC is 12-bit on GP26 to GP29. Used by the
// battery monitor to read a divided pack voltage.

namespace cp::hal {

void adc_init(uint8_t pin) {
  analogReadResolution(12);
  pinMode(pin, INPUT);
}

uint16_t adc_read_raw(uint8_t pin) {
  return static_cast<uint16_t>(analogRead(pin));
}

}  // namespace cp::hal

#endif  // BUILD_TARGET == BUILD_TARGET_NATIVE

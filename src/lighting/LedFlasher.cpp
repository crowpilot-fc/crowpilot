// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/lighting/LedFlasher.h"

#include "src/Config.h"

#if ENABLE_LED_FLASHER

#include "src/hal/Led.h"

namespace cp::lighting::flasher {

bool flasherState(uint8_t pattern, uint32_t t_ms) {
  switch (pattern) {
    case LED_FLASH_STEADY:
      return true;

    case LED_FLASH_BEACON: {
      const uint32_t phase = t_ms % LED_BEACON_PERIOD_MS;
      return phase < LED_BEACON_ON_MS;
    }

    case LED_FLASH_STROBE:
    default: {
      // Two flashes then dark: flash, gap, flash, then dark to the period end.
      const uint32_t phase   = t_ms % LED_STROBE_PERIOD_MS;
      const uint32_t flash2_start = LED_STROBE_FLASH_MS + LED_STROBE_GAP_MS;
      const uint32_t flash2_end   = flash2_start + LED_STROBE_FLASH_MS;
      return phase < LED_STROBE_FLASH_MS ||
             (phase >= flash2_start && phase < flash2_end);
    }
  }
}

void init() {
  cp::hal::led_flasher_init();
}

void tick(uint32_t now_us) {
  const uint32_t now_ms = now_us / 1000U;
  const bool lit = flasherState(LED_FLASHER_PATTERN, now_ms);
  // Apply the drive polarity: active-high lights on a high pin.
  const bool pin_high = LED_FLASHER_ACTIVE_HIGH ? lit : !lit;
  cp::hal::led_flasher_set(pin_high);
}

}  // namespace cp::lighting::flasher

#endif  // ENABLE_LED_FLASHER

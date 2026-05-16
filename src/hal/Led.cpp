// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include <Arduino.h>

#include "src/Config.h"
#include "src/hal/Led.h"

namespace cp::hal {

namespace {

// 1 Hz blink. Toggle every 500 ms.
constexpr uint32_t HEARTBEAT_HALF_PERIOD_US = 500000UL;

uint32_t g_last_toggle_us = 0;
bool     g_state          = false;

}  // namespace

void led_init() {
  pinMode(PIN_LED_ONBOARD, OUTPUT);
  digitalWrite(PIN_LED_ONBOARD, LOW);
  g_state          = false;
  g_last_toggle_us = 0;
}

void led_tick(uint32_t t_now_us) {
  if ((t_now_us - g_last_toggle_us) < HEARTBEAT_HALF_PERIOD_US) {
    return;
  }
  g_state          = !g_state;
  g_last_toggle_us = t_now_us;
  digitalWriteFast(PIN_LED_ONBOARD, g_state ? HIGH : LOW);
}

[[noreturn]] void haltWithFastBlink() {
  pinMode(PIN_LED_ONBOARD, OUTPUT);
  bool     state          = false;
  uint32_t last_toggle_us = micros();
  // 5 Hz blink = 200 ms period = 100 ms half period.
  constexpr uint32_t kPanicHalfPeriodUs = 100000UL;
  while (true) {
    if ((micros() - last_toggle_us) >= kPanicHalfPeriodUs) {
      state          = !state;
      last_toggle_us = micros();
      digitalWriteFast(PIN_LED_ONBOARD, state ? HIGH : LOW);
    }
  }
}

}  // namespace cp::hal

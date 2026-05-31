// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/Config.h"
#include "src/hal/Led.h"

#if BUILD_TARGET == BUILD_TARGET_NATIVE

#include <Arduino.h>

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

void led_flasher_init() {
  pinMode(LED_FLASHER_PIN, OUTPUT);
  digitalWrite(LED_FLASHER_PIN, LOW);
}

void led_flasher_set(bool high) {
  digitalWriteFast(LED_FLASHER_PIN, high ? HIGH : LOW);
}

[[noreturn]] void haltWithFastBlink(const char* repeat_msg) {
  pinMode(PIN_LED_ONBOARD, OUTPUT);
  // Also drive the WeAct V10's second user LED (GP24, LED1) so a bench
  // operator sees TWO LEDs blinking at different rates. Boards without a
  // GP24 LED just toggle a free pin, harmless. The two rates make it
  // unmistakable: fast = panic, slow = "FC is alive". A single LED can be
  // mistaken for a noisy power rail; two clearly different rates cannot.
  constexpr uint8_t kAuxBlinkPin = 24;
  pinMode(kAuxBlinkPin, OUTPUT);

  bool     state           = false;
  uint32_t last_msg_ms     = 0;
  uint32_t last_aux_ms     = 0;
  bool     aux_state       = false;
  // 5 Hz blink = 200 ms period = 100 ms half period.
  constexpr uint32_t kPanicHalfPeriodMs = 100UL;
  constexpr uint32_t kAuxHalfPeriodMs   = 500UL;   // 1 Hz on the aux LED.
  constexpr uint32_t kRepeatIntervalMs  = 1000UL;
  while (true) {
    state = !state;
    // Plain digitalWrite, not digitalWriteFast, to rule out any rp2040-core
    // optimisation quirk on RP2350. Speed in the panic loop does not matter.
    digitalWrite(PIN_LED_ONBOARD, state ? HIGH : LOW);
    if ((millis() - last_aux_ms) >= kAuxHalfPeriodMs) {
      aux_state = !aux_state;
      digitalWrite(kAuxBlinkPin, aux_state ? HIGH : LOW);
      last_aux_ms = millis();
    }
    // Re-print the diagnostic at ~1 Hz so a bench operator who attaches a
    // serial monitor after the firmware has already halted catches the
    // message. Without this repeat, the original print would be lost in the
    // CDC buffer the moment that buffer overflows or the host disconnects.
    if (repeat_msg != nullptr && (millis() - last_msg_ms) >= kRepeatIntervalMs) {
      Serial.print("PANIC: ");
      Serial.println(repeat_msg);
      last_msg_ms = millis();
    }
    // delay() yields control to the rp2040 core, which polls USB CDC.
    // Without this yield, the firmware busy-spins and Serial bytes never
    // leave the FC.
    delay(kPanicHalfPeriodMs);
  }
}

}  // namespace cp::hal

#endif  // BUILD_TARGET == BUILD_TARGET_NATIVE

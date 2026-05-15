// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "user_hook/UserHook.h"

#include <Arduino.h>

#include "Config.h"
#include "user_hook/Budget.h"

// User entry points. Defined in user-sketch.ino at the repo root, which the
// Arduino IDE concatenates into the sketch. Declared here so the dispatcher
// can call them across translation units.
void user_setup();
void user_tick();

namespace cp::user_hook {

namespace {

bool     s_enabled        = false;
uint32_t s_tick_counter   = 0;
uint32_t s_strikes        = 0;
uint32_t s_warnings       = 0;
uint32_t s_calls          = 0;
uint32_t s_last_us        = 0;
uint32_t s_max_us         = 0;
bool     s_disable_logged = false;

}  // anonymous namespace

void init() {
#if ENABLE_USER_HOOK
  s_enabled        = true;
  s_tick_counter   = 0;
  s_strikes        = 0;
  s_warnings       = 0;
  s_calls          = 0;
  s_last_us        = 0;
  s_max_us         = 0;
  s_disable_logged = false;
  user_setup();
  if (Serial) {
    Serial.println("User hook OK (user_setup done).");
  }
#endif
}

void tick() {
#if ENABLE_USER_HOOK
  ++s_tick_counter;
  if (!s_enabled) {
    return;
  }
  if ((s_tick_counter % USER_HOOK_RATE_DIV) != 0) {
    return;
  }

  budget::arm();
  user_tick();
  budget::markEnd();

  ++s_calls;
  s_last_us = budget::lastUs();
  if (s_last_us > s_max_us) {
    s_max_us = s_last_us;
  }

  if (budget::trippedHard()) {
    ++s_strikes;
    if (s_strikes >= 3) {
      // Sticky disable. The hook stays off for the rest of the flight;
      // a reboot is required to re-enable.
      s_enabled = false;
      if (!s_disable_logged) {
        if (Serial) {
          Serial.println("WARN: user hook disabled after 3 overruns.");
        }
        s_disable_logged = true;
      }
    }
  } else if (budget::trippedSoft()) {
    ++s_warnings;
  }
#endif
}

bool enabled() {
  return s_enabled;
}

uint32_t strikeCount() {
  return s_strikes;
}

uint32_t warningCount() {
  return s_warnings;
}

uint32_t lastUs() {
  return s_last_us;
}

uint32_t maxUs() {
  return s_max_us;
}

uint32_t callCount() {
  return s_calls;
}

}  // namespace cp::user_hook

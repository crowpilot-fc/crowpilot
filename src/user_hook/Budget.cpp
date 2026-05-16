// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/user_hook/Budget.h"

#include <Arduino.h>

#include "src/Config.h"

namespace cp::user_hook::budget {

namespace {

uint32_t s_start_us     = 0;
uint32_t s_last_us      = 0;
bool     s_tripped_soft = false;
bool     s_tripped_hard = false;

}  // anonymous namespace

void arm() {
  s_start_us = micros();
}

void markEnd() {
  s_last_us      = micros() - s_start_us;
  s_tripped_soft = s_last_us > USER_HOOK_BUDGET_US;
  s_tripped_hard = s_last_us > USER_HOOK_HARD_LIMIT_US;
}

uint32_t lastUs() {
  return s_last_us;
}

bool trippedSoft() {
  return s_tripped_soft;
}

bool trippedHard() {
  return s_tripped_hard;
}

}  // namespace cp::user_hook::budget

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// Time-budget tracker for the user extension hook. The hook runs inside the
// main loop with no preemption, so the budget is enforced by measuring
// elapsed time across the user_tick() call rather than by a preempting
// interrupt: arm() before the call, markEnd() after, then read the trip
// flags. A soft trip (over USER_HOOK_BUDGET_US) is advisory; a hard trip
// (over the 250 us hard limit) costs the hook a strike.

namespace cp::user_hook::budget {

// Record the start time of a user_tick() call.
void arm();

// Record the end time and latch the soft and hard trip flags.
void markEnd();

// Microseconds the last user_tick() call took.
uint32_t lastUs();

// True if the last call exceeded the soft budget (USER_HOOK_BUDGET_US).
bool trippedSoft();

// True if the last call exceeded the hard limit (USER_HOOK_HARD_LIMIT_US).
bool trippedHard();

}  // namespace cp::user_hook::budget

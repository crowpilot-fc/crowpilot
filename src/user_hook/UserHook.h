// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// User extension hook dispatcher. Lets a pilot add non-flight-critical
// behavior (retracts, flaps, lights, custom sensors) in user-sketch.ino
// without touching the flight stack. The user provides user_setup() and
// user_tick(); the dispatcher rate-limits user_tick(), enforces a time
// budget, and disables the hook after three hard overruns.
//
// The hook is gated by ENABLE_USER_HOOK and defaults off. See
// internal-docs/USER_HOOK_SPEC.md and docs/user-guide/user-sketch.md.

namespace cp::user_hook {

// Run user_setup() once, after the flight stack is initialised. A no-op
// when ENABLE_USER_HOOK is 0.
void init();

// Dispatch user_tick() if this is a hook tick (every USER_HOOK_RATE_DIV
// main loop iterations) and the hook is still enabled. Arms the time
// budget, records strikes, and disables the hook on the third hard
// overrun. A no-op when ENABLE_USER_HOOK is 0.
void tick();

// True while the hook is still allowed to run. Goes false permanently
// after three hard overruns; a reboot is required to re-enable.
bool enabled();

// Hard overruns recorded so far. Three disables the hook.
uint32_t strikeCount();

// Soft-budget overruns recorded so far. Advisory only.
uint32_t warningCount();

// Duration of the most recent user_tick() call, microseconds.
uint32_t lastUs();

// Worst user_tick() duration seen since boot, microseconds.
uint32_t maxUs();

// Number of user_tick() calls dispatched since boot.
uint32_t callCount();

}  // namespace cp::user_hook

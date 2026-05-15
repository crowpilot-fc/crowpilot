// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

// Cross-module shared state. Per ARCHITECTURE.md §5, kept intentionally
// minimal. Most cross-module data flows via per-module accessor functions
// (cp::sensors::imu::latest(), cp::modes::fader(), etc.). Add extern
// declarations here only when a piece of state is genuinely needed by more
// than one module and cannot fit a per-module accessor.

namespace cp::state {

// No shared extern variables in Phase 1.

}  // namespace cp::state

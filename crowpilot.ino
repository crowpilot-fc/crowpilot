// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

// CrowPilot Arduino entry shim.
//
// All firmware logic lives under src/. This file exists only because the
// Arduino build system needs a sketch at the repository root. setup()
// brings up every module in order, loop() runs one cooperative super-loop
// iteration. Both delegate straight to cp::core.

#include "src/core/Loop.h"

void setup() {
  cp::core::init();
}

void loop() {
  cp::core::tick();
}

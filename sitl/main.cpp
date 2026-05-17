// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// SITL host entry point. Stands in for the Arduino setup()/loop()
// driver: it brings the firmware up against the simulated HAL and runs
// the 1 kHz superloop for a bounded number of ticks.
//
// Usage: crowpilot_sitl [tick_count]   (default 3000, three seconds)

#include <cstdio>
#include <cstdlib>

#include "src/core/Loop.h"

int main(int argc, char** argv) {
  unsigned long ticks = 3000;
  if (argc > 1) {
    ticks = strtoul(argv[1], nullptr, 10);
  }

  printf("=== CrowPilot SITL host build ===\n");
  printf("running %lu loop ticks against the simulated HAL\n\n", ticks);

  cp::core::init();
  for (unsigned long i = 0; i < ticks; ++i) {
    cp::core::tick();
  }

  printf("\n=== SITL run complete: %lu ticks ===\n", ticks);
  return 0;
}

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// SITL host entry point. Stands in for the Arduino setup()/loop()
// driver: it brings the firmware up against the simulated HAL and runs
// the 1 kHz superloop for a bounded number of ticks. Once per simulated
// second it prints a SIM line with the rigid-body model's ground truth,
// so the controller's progress can be read against the real attitude.
//
// Usage: crowpilot_sitl [tick_count]   (default 3000, three seconds)

#include <cstdio>
#include <cstdlib>

#include "src/core/Loop.h"
#include "src/hal/sim/SimPhysics.h"

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
    if ((i + 1) % 1000 == 0) {
      float gx = 0.0f;
      float gy = 0.0f;
      float gz = 0.0f;
      cp::sim::physics_gyro_dps(gx, gy, gz);
      printf("SIM  t=%lus  roll=%.1f pitch=%.1f deg  "
             "rates=(%.1f,%.1f,%.1f) dps\n",
             (i + 1) / 1000,
             static_cast<double>(cp::sim::physics_roll_deg()),
             static_cast<double>(cp::sim::physics_pitch_deg()),
             static_cast<double>(gx), static_cast<double>(gy),
             static_cast<double>(gz));
    }
  }

  printf("\n=== SITL run complete: %lu ticks ===\n", ticks);
  return 0;
}

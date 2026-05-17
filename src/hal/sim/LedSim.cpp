// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// Simulated status-LED HAL for the SITL host build. The blink is not
// modelled. A fatal init failure prints and exits the process, the host
// equivalent of the panic blink loop.

#include "src/hal/Led.h"

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_HIL || BUILD_TARGET == BUILD_TARGET_SITL

#include <stdio.h>
#include <stdlib.h>

namespace cp::hal {

void led_init() {}

void led_tick(uint32_t) {}

void haltWithFastBlink() {
  printf("SITL: haltWithFastBlink, fatal init failure, exiting.\n");
  fflush(stdout);
  exit(1);
}

}  // namespace cp::hal

#endif  // SITL or HIL

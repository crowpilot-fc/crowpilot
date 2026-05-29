// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// Simulated analog input for the SITL host build. There is no real battery,
// so adc_read_raw returns a fixed synthetic count standing in for a nominal
// pack so the battery monitor reports a plausible, non-zero voltage.

#include "src/hal/Hal.h"

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_HIL || BUILD_TARGET == BUILD_TARGET_SITL

namespace cp::hal {

void adc_init(uint8_t) {}

uint16_t adc_read_raw(uint8_t) {
  // About 1.41 V at the pin. With the default 11:1 divider that is roughly a
  // mid-charge 4S pack (about 15.5 V). Purely synthetic.
  return 1750;
}

}  // namespace cp::hal

#endif  // SITL or HIL

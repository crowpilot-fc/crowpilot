// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/hal/Hal.h"

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_NATIVE

#include <Arduino.h>

#include "src/hal/I2c.h"

#if   BARO_TYPE == BARO_BMP388
  #include "src/libs/Bmp388.h"
  namespace baro_chip = cp::libs::bmp388;
#elif BARO_TYPE == BARO_BMP280
  #include "src/libs/Bmp280.h"
  namespace baro_chip = cp::libs::bmp280;
#elif BARO_TYPE == BARO_NONE
  // No chip include. baro_present() returns false and reads always fail.
#else
  #error "BARO_TYPE must be BARO_BMP388, BARO_BMP280, or BARO_NONE."
#endif

namespace cp::hal {

namespace {

bool s_present = false;

}  // anonymous namespace

bool baro_init() {
#if BARO_TYPE == BARO_NONE
  s_present = false;
  return true;
#else
  cp::hal::i2c0::ensureInit();
  if (!baro_chip::init(BARO_I2C_ADDR)) {
    s_present = false;
    return false;
  }
  s_present = true;
  return true;
#endif
}

bool baro_present() {
  return s_present;
}

bool baro_read(BaroSample& out) {
#if BARO_TYPE == BARO_NONE
  out.pressure_pa   = 0.0f;
  out.temperature_c = 0.0f;
  return false;
#else
  if (!s_present) {
    return false;
  }
  baro_chip::CompensatedSample raw;
  if (!baro_chip::readAndCompensate(raw)) {
    return false;
  }
  out.pressure_pa   = raw.pressure_pa;
  out.temperature_c = raw.temperature_c;
  return true;
#endif
}

}  // namespace cp::hal

#endif  // BUILD_TARGET == BUILD_TARGET_NATIVE

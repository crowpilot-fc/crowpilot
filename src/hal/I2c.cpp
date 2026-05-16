// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/hal/I2c.h"

#include <Arduino.h>
#include <Wire.h>

#include "src/Config.h"

namespace cp::hal::i2c0 {

namespace {

bool s_initialized = false;

}  // anonymous namespace

void ensureInit() {
  if (s_initialized) {
    return;
  }
  // The Pi Pico Arduino core requires setSDA / setSCL before begin.
  // setClock can run after begin per the documented examples.
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(I2C_BUS_HZ);
  s_initialized = true;
}

}  // namespace cp::hal::i2c0

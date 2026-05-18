// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

namespace cp::sensors::baro {

// One barometer sample. Altitude is relative to a ground-pressure
// reference captured at the first successful read after init. The valid
// flag mirrors the most recent read's outcome. When `BARO_TYPE` is
// `BARO_NONE` the facade never sets valid to true and `is_present`
// returns false; downstream consumers check `is_present` before
// trusting altitude.
struct Sample {
  float    pressure_pa;
  float    temperature_c;
  float    altitude_m;
  uint32_t timestamp_us;
  bool     valid;
};

// Bring up the I2C0 bus (via cp::hal::i2c::ensureInit) and probe the
// configured baro chip. When `BARO_TYPE = BARO_NONE` this returns true
// immediately without touching the bus or claiming a present chip;
// downstream code can call `is_present` to distinguish.
bool init();

// Drive a read. Internally rate-limited via Config.h's
// BARO_READ_INTERVAL_TICKS so that calling every loop tick produces a
// read at the chip's output rate (default ~50 Hz). On the first
// successful read after init the current pressure becomes the
// ground-pressure baseline for altitude.
bool read();

const Sample& latest();

// True if a baro chip is configured AND its init succeeded.
bool is_present();

// True once init has succeeded and no transient I2C failure has been
// seen on the most recent read.
bool is_healthy();

}  // namespace cp::sensors::baro

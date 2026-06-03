// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

namespace cp::hal::i2c {

// Bus state recorded by ensureInit before it hands off to Wire.
enum class RecoveryStatus : uint8_t {
  kClean   = 0,  // bus was idle at power-up, no recovery needed.
  kUnstuck = 1,  // a slave held SDA low; the SCL pulse train released it.
  kStuck   = 2,  // pulses did not release SDA; something is still pulling
                 // the line low (likely faulty hardware on the bus).
};

// Bring up the I2C0 bus on PIN_I2C_SDA / PIN_I2C_SCL at I2C_BUS_HZ. Safe
// to call from any module's init() since the first call configures Wire
// and subsequent calls return immediately. Modules that share the bus
// (IMU and barometer in v1.0) each call this near the top of their own
// init() so neither has to know which lands first.
void ensureInit();

// Status from the most recent ensureInit() bus-recovery pass. Reported
// in the boot trace and exposed by `cp scan` for bench triage.
RecoveryStatus lastRecoveryStatus();

}  // namespace cp::hal::i2c

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

namespace cp::hal::i2c0 {

// Bring up the I2C0 bus on PIN_I2C_SDA / PIN_I2C_SCL at I2C_BUS_HZ. Safe
// to call from any module's init() since the first call configures Wire
// and subsequent calls return immediately. Modules that share the bus
// (IMU and barometer in v1.0) each call this near the top of their own
// init() so neither has to know which lands first.
void ensureInit();

}  // namespace cp::hal::i2c0

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// BMP388 driver. Written from the Bosch Sensortec BMP388 Digital Pressure
// Sensor datasheet rev 1.5. Section 9 covers the compensation formulas
// used here. See ALGORITHMS.md §4.1 for the implementation recipe and
// SPEC.md §16.2 for the citation. No source code from other flight
// controller projects was consulted.

namespace cp::libs::bmp388 {

struct CompensatedSample {
  float pressure_pa;
  float temperature_c;
};

// Probe via CHIP_ID (0x50), soft-reset, load 21-byte NVM calibration,
// configure pressure x8 and temp x4 oversampling, 50 Hz ODR, IIR filter
// coefficient 15, normal mode. Returns false on any I2C NAK or
// unexpected CHIP_ID.
bool init(uint8_t i2c_addr);

// Read the 6-byte data burst at 0x04, apply the floating-point
// compensation per the datasheet, and write the temperature in degrees
// Celsius plus the pressure in pascals to out. Returns false on I2C NAK
// or short read.
bool readAndCompensate(CompensatedSample& out);

}  // namespace cp::libs::bmp388

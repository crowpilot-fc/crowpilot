// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// BMP280 driver. Written from the Bosch Sensortec BMP280 Digital Pressure
// Sensor datasheet v1.20. Section 8.2 covers the compensation formulas
// used here. See ALGORITHMS.md §4.2 for the implementation recipe and
// SPEC.md §16.2 for the citation. No source code from other flight
// controller projects was consulted.

namespace cp::libs::bmp280 {

struct CompensatedSample {
  float pressure_pa;
  float temperature_c;
};

// Probe via CHIP_ID (0xD0 reading 0x58), soft-reset, wait for NVM image
// to copy, load 24-byte calibration, configure pressure x16 and temp x2
// oversampling, IIR filter coefficient 16, normal mode. Returns false on
// any I2C NAK or unexpected CHIP_ID.
bool init(uint8_t i2c_addr);

// Read the 6-byte data burst at 0xF7, apply the 32-bit and 64-bit
// integer compensation per the datasheet, and write the temperature in
// degrees Celsius plus the pressure in pascals to out. Returns false on
// I2C NAK or short read.
bool readAndCompensate(CompensatedSample& out);

}  // namespace cp::libs::bmp280

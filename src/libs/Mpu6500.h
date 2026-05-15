// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// MPU-6500 driver. Written from the TDK InvenSense MPU-6500 Product
// Specification (Revision 1.3) and Register Map (Revision 2.1). See SPEC.md
// §16.2 for the citation. No source code from other flight controller
// projects was consulted.

namespace cp::libs::mpu6500 {

struct RawSample {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};

// Probe the chip at i2c_addr via WHO_AM_I, soft-reset, configure clock,
// sample rate, DLPF, and full-scale ranges. Caches the address and ranges
// for subsequent calls. Returns false on any I2C NAK or unexpected
// WHO_AM_I value.
//
// gyro_fs_sel and accel_fs_sel are the raw FS_SEL / AFS_SEL field values
// (0..3). They map to the GYRO_xxx_DPS / ACCEL_xG selectors in Config.h.
bool init(uint8_t i2c_addr, uint8_t gyro_fs_sel, uint8_t accel_fs_sel);

// Read all 14 sensor bytes (ACCEL x3, TEMP, GYRO x3) in one I2C burst
// starting at ACCEL_XOUT_H (0x3B). Returns false on I2C NAK or short read.
bool readBurst(RawSample& out);

// Sensitivity for the active gyro range, in LSB per deg/s. Datasheet
// table for FS_SEL: 0 -> 131, 1 -> 65.5, 2 -> 32.8, 3 -> 16.4.
float gyroSensitivityLsbPerDps();

// Sensitivity for the active accel range, in LSB per g. Datasheet table
// for AFS_SEL: 0 -> 16384, 1 -> 8192, 2 -> 4096, 3 -> 2048.
float accelSensitivityLsbPerG();

// Per the MPU-6500 datasheet section 11.3 (Temperature in Degrees C):
// temp_c = (raw / 333.87) + 21.0.
float tempRawToCelsius(int16_t raw);

}  // namespace cp::libs::mpu6500

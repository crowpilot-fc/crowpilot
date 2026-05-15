// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// MPU-6050 driver. Written from the TDK InvenSense MPU-6000/MPU-6050 Product
// Specification (Revision 3.4) and Register Map (Revision 4.2). See SPEC.md
// §16.2 for the citation. No source code from other flight controller
// projects was consulted.

namespace cp::libs::mpu6050 {

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
bool init(uint8_t i2c_addr, uint8_t gyro_fs_sel, uint8_t accel_fs_sel);

// Read all 14 sensor bytes (ACCEL x3, TEMP, GYRO x3) in one I2C burst
// starting at ACCEL_XOUT_H (0x3B).
bool readBurst(RawSample& out);

// Same datasheet table as MPU-6500: 131 / 65.5 / 32.8 / 16.4 LSB per deg/s.
float gyroSensitivityLsbPerDps();

// Same datasheet table as MPU-6500: 16384 / 8192 / 4096 / 2048 LSB per g.
float accelSensitivityLsbPerG();

// Per the MPU-6050 datasheet section 4.18 (Register 65 and 66): the
// formula is temp_c = (raw / 340) + 36.53.
float tempRawToCelsius(int16_t raw);

}  // namespace cp::libs::mpu6050

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/libs/Mpu6050.h"

#include <Arduino.h>
#include <Wire.h>

namespace cp::libs::mpu6050 {

namespace {

// Register addresses per MPU-6050 Register Map rev 4.2.
constexpr uint8_t kRegSmplrtDiv    = 0x19;
constexpr uint8_t kRegConfig       = 0x1A;
constexpr uint8_t kRegGyroConfig   = 0x1B;
constexpr uint8_t kRegAccelConfig  = 0x1C;
constexpr uint8_t kRegAccelXoutH   = 0x3B;
constexpr uint8_t kRegPwrMgmt1     = 0x6B;
constexpr uint8_t kRegPwrMgmt2     = 0x6C;
constexpr uint8_t kRegWhoAmI       = 0x75;

// WHO_AM_I expected value per datasheet section 4.32. The register reads back
// the upper 6 bits of the I2C address with the LSB/MSB masked. The default
// AD0=0 address 0b1101000 yields WHO_AM_I = 0x68. AD0=1 reads the same here
// because bit 0 of the address is not part of WHO_AM_I.
constexpr uint8_t kWhoAmIExpected  = 0x68;

constexpr uint8_t kPwrResetBit     = 0x80;
constexpr uint8_t kPwrClkPllXGyro  = 0x01;

// CONFIG (0x1A) DLPF_CFG = 1 -> accel BW 184 Hz, gyro BW 188 Hz, 1.9 ms
// gyro delay, internal sample rate 1 kHz. MPU-6050 shares DLPF across both
// accel and gyro (no separate ACCEL_CONFIG2 register).
constexpr uint8_t kConfigDlpf      = 0x01;

constexpr uint8_t kSmplrtDiv1kHz   = 0x00;

constexpr uint32_t kResetWaitUs    = 100UL * 1000UL;  // 100 ms

uint8_t s_addr           = 0;
uint8_t s_gyro_fs_sel    = 0;
uint8_t s_accel_fs_sel   = 0;

// Busy-wait for the requested microseconds. Used after the soft-reset
// write per CODING_STANDARDS.md §9.1 ("No delay(). Ever."). Init runs
// once at boot so this is acceptable; the hot loop never calls this.
void busyWaitUs(uint32_t us) {
  const uint32_t t0 = micros();
  while ((micros() - t0) < us) {
    // spin
  }
}

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(s_addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool readReg(uint8_t reg, uint8_t& value) {
  Wire.beginTransmission(s_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(s_addr, static_cast<uint8_t>(1), static_cast<uint8_t>(true)) != 1) {
    return false;
  }
  value = static_cast<uint8_t>(Wire.read());
  return true;
}

inline int16_t packInt16Be(uint8_t hi, uint8_t lo) {
  return static_cast<int16_t>(
      (static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo));
}

}  // anonymous namespace

bool init(uint8_t i2c_addr, uint8_t gyro_fs_sel, uint8_t accel_fs_sel) {
  s_addr         = i2c_addr;
  s_gyro_fs_sel  = gyro_fs_sel & 0x03;
  s_accel_fs_sel = accel_fs_sel & 0x03;

  uint8_t who = 0;
  if (!readReg(kRegWhoAmI, who)) {
    return false;
  }
  if (who != kWhoAmIExpected) {
    return false;
  }

  if (!writeReg(kRegPwrMgmt1, kPwrResetBit)) {
    return false;
  }
  busyWaitUs(kResetWaitUs);

  if (!writeReg(kRegPwrMgmt1, kPwrClkPllXGyro)) {
    return false;
  }
  if (!writeReg(kRegPwrMgmt2, 0x00)) {
    return false;
  }
  if (!writeReg(kRegSmplrtDiv, kSmplrtDiv1kHz)) {
    return false;
  }
  if (!writeReg(kRegConfig, kConfigDlpf)) {
    return false;
  }
  if (!writeReg(kRegGyroConfig, static_cast<uint8_t>(s_gyro_fs_sel << 3))) {
    return false;
  }
  if (!writeReg(kRegAccelConfig, static_cast<uint8_t>(s_accel_fs_sel << 3))) {
    return false;
  }
  return true;
}

bool readBurst(RawSample& out) {
  Wire.beginTransmission(s_addr);
  Wire.write(kRegAccelXoutH);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(s_addr, static_cast<uint8_t>(14), static_cast<uint8_t>(true)) != 14) {
    return false;
  }
  uint8_t b[14];
  for (uint8_t i = 0; i < 14; ++i) {
    b[i] = static_cast<uint8_t>(Wire.read());
  }
  out.ax   = packInt16Be(b[0],  b[1]);
  out.ay   = packInt16Be(b[2],  b[3]);
  out.az   = packInt16Be(b[4],  b[5]);
  out.temp = packInt16Be(b[6],  b[7]);
  out.gx   = packInt16Be(b[8],  b[9]);
  out.gy   = packInt16Be(b[10], b[11]);
  out.gz   = packInt16Be(b[12], b[13]);
  return true;
}

float gyroSensitivityLsbPerDps() {
  switch (s_gyro_fs_sel) {
    case 0: return 131.0f;
    case 1: return  65.5f;
    case 2: return  32.8f;
    case 3: return  16.4f;
    default: return 131.0f;
  }
}

float accelSensitivityLsbPerG() {
  switch (s_accel_fs_sel) {
    case 0: return 16384.0f;
    case 1: return  8192.0f;
    case 2: return  4096.0f;
    case 3: return  2048.0f;
    default: return 16384.0f;
  }
}

float tempRawToCelsius(int16_t raw) {
  return static_cast<float>(raw) * (1.0f / 340.0f) + 36.53f;
}

}  // namespace cp::libs::mpu6050

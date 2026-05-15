// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "libs/Bmp280.h"

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

namespace cp::libs::bmp280 {

namespace {

// Register addresses per BMP280 datasheet section 4.
constexpr uint8_t kRegChipId      = 0xD0;
constexpr uint8_t kRegReset       = 0xE0;
constexpr uint8_t kRegStatus      = 0xF3;
constexpr uint8_t kRegCtrlMeas    = 0xF4;
constexpr uint8_t kRegConfig      = 0xF5;
constexpr uint8_t kRegPressMsb    = 0xF7;   // start of 6-byte data burst
constexpr uint8_t kRegCalib       = 0x88;   // start of 24-byte calibration

constexpr uint8_t kChipIdExpected = 0x58;
constexpr uint8_t kResetValue     = 0xB6;

// Init register values per ALGORITHMS.md §4.2.1.
constexpr uint8_t kConfigValue    = 0x10;   // t_sb=0.5ms, filter=16, spi3w_en=0
constexpr uint8_t kCtrlMeasValue  = 0x57;   // osrs_t=x2, osrs_p=x16, mode=normal

constexpr uint8_t kStatusImUpdate = 0x01;

constexpr uint32_t kResetWaitUs   = 5UL * 1000UL;     // 5 ms
constexpr uint32_t kNvmPollUs     = 1UL * 1000UL;     // 1 ms per poll
constexpr uint32_t kNvmTimeoutUs  = 50UL * 1000UL;    // 50 ms guard

// Driver state.
uint8_t s_addr = 0;

// Calibration coefficients per ALGORITHMS.md §4.2.1. Stored at chip-native
// width to match the integer compensation formulas in §4.2.3 and §4.2.4.
uint16_t s_dig_t1 = 0;
int16_t  s_dig_t2 = 0;
int16_t  s_dig_t3 = 0;
uint16_t s_dig_p1 = 0;
int16_t  s_dig_p2 = 0;
int16_t  s_dig_p3 = 0;
int16_t  s_dig_p4 = 0;
int16_t  s_dig_p5 = 0;
int16_t  s_dig_p6 = 0;
int16_t  s_dig_p7 = 0;
int16_t  s_dig_p8 = 0;
int16_t  s_dig_p9 = 0;

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

bool readBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(s_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(s_addr, len, static_cast<uint8_t>(true)) != len) {
    return false;
  }
  for (uint8_t i = 0; i < len; ++i) {
    buf[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

inline uint16_t leU16(const uint8_t* p) {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) |
                               (static_cast<uint16_t>(p[1]) << 8));
}

inline int16_t leI16(const uint8_t* p) {
  return static_cast<int16_t>(leU16(p));
}

bool loadCalibration() {
  uint8_t c[24];
  if (!readBytes(kRegCalib, c, sizeof(c))) {
    return false;
  }
  s_dig_t1 = leU16(c + 0);
  s_dig_t2 = leI16(c + 2);
  s_dig_t3 = leI16(c + 4);
  s_dig_p1 = leU16(c + 6);
  s_dig_p2 = leI16(c + 8);
  s_dig_p3 = leI16(c + 10);
  s_dig_p4 = leI16(c + 12);
  s_dig_p5 = leI16(c + 14);
  s_dig_p6 = leI16(c + 16);
  s_dig_p7 = leI16(c + 18);
  s_dig_p8 = leI16(c + 20);
  s_dig_p9 = leI16(c + 22);
  return true;
}

bool waitForNvmCopy() {
  // Per datasheet 3.10, the NVM image copy completes within ~2 ms after
  // reset. Poll the im_update bit of the status register up to the
  // guard timeout. Returns true once im_update is clear.
  const uint32_t t0 = micros();
  while ((micros() - t0) < kNvmTimeoutUs) {
    uint8_t status = 0;
    if (readReg(kRegStatus, status) && (status & kStatusImUpdate) == 0) {
      return true;
    }
    busyWaitUs(kNvmPollUs);
  }
  return false;
}

// Temperature compensation per ALGORITHMS.md §4.2.3. The intermediate
// `t_fine` is returned via the out param because the pressure
// compensation in §4.2.4 reuses it.
float compensateTemperature(int32_t raw_t, int32_t& t_fine_out) {
  const int32_t var1 = (((raw_t >> 3) - (static_cast<int32_t>(s_dig_t1) << 1))
                       * static_cast<int32_t>(s_dig_t2)) >> 11;
  const int32_t delta = (raw_t >> 4) - static_cast<int32_t>(s_dig_t1);
  const int32_t var2 = (((delta * delta) >> 12) * static_cast<int32_t>(s_dig_t3)) >> 14;
  const int32_t t_fine = var1 + var2;
  t_fine_out = t_fine;
  const int32_t T = (t_fine * 5 + 128) >> 8;
  return static_cast<float>(T) * 0.01f;
}

// Pressure compensation per ALGORITHMS.md §4.2.4. Returns pressure in
// pascals as float. Returns 0.0f if the intermediate divisor is zero,
// which the spec calls out as a divide-by-zero guard.
float compensatePressure(int32_t raw_p, int32_t t_fine) {
  int64_t var1 = static_cast<int64_t>(t_fine) - 128000;
  int64_t var2 = var1 * var1 * static_cast<int64_t>(s_dig_p6);
  var2 = var2 + ((var1 * static_cast<int64_t>(s_dig_p5)) << 17);
  var2 = var2 + (static_cast<int64_t>(s_dig_p4) << 35);
  var1 = ((var1 * var1 * static_cast<int64_t>(s_dig_p3)) >> 8)
       + ((var1 * static_cast<int64_t>(s_dig_p2)) << 12);
  var1 = (((static_cast<int64_t>(1) << 47) + var1) * static_cast<int64_t>(s_dig_p1)) >> 33;

  if (var1 == 0) {
    return 0.0f;
  }

  int64_t p = 1048576 - static_cast<int64_t>(raw_p);
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (static_cast<int64_t>(s_dig_p9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (static_cast<int64_t>(s_dig_p8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (static_cast<int64_t>(s_dig_p7) << 4);

  // p is in Q24.8 fixed-point pascals. Divide by 256 for the float result.
  return static_cast<float>(p) * (1.0f / 256.0f);
}

}  // anonymous namespace

bool init(uint8_t i2c_addr) {
  s_addr = i2c_addr;

  uint8_t id = 0;
  if (!readReg(kRegChipId, id)) {
    return false;
  }
  if (id != kChipIdExpected) {
    return false;
  }

  if (!writeReg(kRegReset, kResetValue)) {
    return false;
  }
  busyWaitUs(kResetWaitUs);

  if (!waitForNvmCopy()) {
    return false;
  }

  if (!loadCalibration()) {
    return false;
  }

  if (!writeReg(kRegConfig, kConfigValue)) {
    return false;
  }
  if (!writeReg(kRegCtrlMeas, kCtrlMeasValue)) {
    return false;
  }
  return true;
}

bool readAndCompensate(CompensatedSample& out) {
  uint8_t b[6];
  if (!readBytes(kRegPressMsb, b, sizeof(b))) {
    return false;
  }
  // 20-bit big-endian pressure and temperature per datasheet 4.2.
  const int32_t raw_p = (static_cast<int32_t>(b[0]) << 12) |
                        (static_cast<int32_t>(b[1]) << 4)  |
                        (static_cast<int32_t>(b[2]) >> 4);
  const int32_t raw_t = (static_cast<int32_t>(b[3]) << 12) |
                        (static_cast<int32_t>(b[4]) << 4)  |
                        (static_cast<int32_t>(b[5]) >> 4);

  int32_t t_fine = 0;
  out.temperature_c = compensateTemperature(raw_t, t_fine);
  out.pressure_pa   = compensatePressure(raw_p, t_fine);
  return true;
}

}  // namespace cp::libs::bmp280

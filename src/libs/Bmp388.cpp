// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "libs/Bmp388.h"

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

namespace cp::libs::bmp388 {

namespace {

// Register addresses per BMP388 datasheet section 5.6.
constexpr uint8_t kRegChipId      = 0x00;
constexpr uint8_t kRegData0       = 0x04;   // start of 6-byte data burst
constexpr uint8_t kRegPwrCtrl     = 0x1B;
constexpr uint8_t kRegOsr         = 0x1C;
constexpr uint8_t kRegOdr         = 0x1D;
constexpr uint8_t kRegConfig      = 0x1F;
constexpr uint8_t kRegNvmParT1    = 0x31;   // start of 21-byte NVM calibration
constexpr uint8_t kRegCmd         = 0x7E;

constexpr uint8_t kChipIdExpected = 0x50;
constexpr uint8_t kCmdSoftReset   = 0xB6;

// Init register values per ALGORITHMS.md §4.1.1.
constexpr uint8_t kPwrCtrlValue   = 0x33;   // press_en=1, temp_en=1, mode=normal
constexpr uint8_t kOsrValue       = 0x13;   // osr_p=x8, osr_t=x4
constexpr uint8_t kOdrValue       = 0x02;   // 50 Hz output data rate
constexpr uint8_t kConfigValue    = 0x08;   // IIR filter coefficient 15

constexpr uint32_t kResetWaitUs   = 10UL * 1000UL;  // 10 ms

// Driver state.
uint8_t s_addr = 0;

// Floating-point calibration coefficients per ALGORITHMS.md §4.1.1.
float s_par_t1  = 0.0f;
float s_par_t2  = 0.0f;
float s_par_t3  = 0.0f;
float s_par_p1  = 0.0f;
float s_par_p2  = 0.0f;
float s_par_p3  = 0.0f;
float s_par_p4  = 0.0f;
float s_par_p5  = 0.0f;
float s_par_p6  = 0.0f;
float s_par_p7  = 0.0f;
float s_par_p8  = 0.0f;
float s_par_p9  = 0.0f;
float s_par_p10 = 0.0f;
float s_par_p11 = 0.0f;

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
  uint8_t nvm[21];
  if (!readBytes(kRegNvmParT1, nvm, sizeof(nvm))) {
    return false;
  }
  // Scaling constants per ALGORITHMS.md §4.1.1 (datasheet Table 7).
  s_par_t1  = static_cast<float>(leU16(nvm + 0)) * 256.0f;
  s_par_t2  = static_cast<float>(leU16(nvm + 2)) / 1073741824.0f;
  s_par_t3  = static_cast<float>(static_cast<int8_t>(nvm[4])) / 281474976710656.0f;
  s_par_p1  = (static_cast<float>(leI16(nvm + 5))  - 16384.0f) / 1048576.0f;
  s_par_p2  = (static_cast<float>(leI16(nvm + 7))  - 16384.0f) / 536870912.0f;
  s_par_p3  = static_cast<float>(static_cast<int8_t>(nvm[9]))  / 4294967296.0f;
  s_par_p4  = static_cast<float>(static_cast<int8_t>(nvm[10])) / 137438953472.0f;
  s_par_p5  = static_cast<float>(leU16(nvm + 11)) * 8.0f;
  s_par_p6  = static_cast<float>(leU16(nvm + 13)) / 64.0f;
  s_par_p7  = static_cast<float>(static_cast<int8_t>(nvm[15])) / 256.0f;
  s_par_p8  = static_cast<float>(static_cast<int8_t>(nvm[16])) / 32768.0f;
  s_par_p9  = static_cast<float>(leI16(nvm + 17)) / 281474976710656.0f;
  s_par_p10 = static_cast<float>(static_cast<int8_t>(nvm[19])) / 281474976710656.0f;
  // Last divisor would not fit a literal float (2^65). Use the closest
  // representable value; the float-32 mantissa gives roughly 7-digit
  // precision here which is well below the chip's resolution.
  s_par_p11 = static_cast<float>(static_cast<int8_t>(nvm[20])) /
              (281474976710656.0f * 131072.0f);
  return true;
}

float compensateTemperature(uint32_t raw_t) {
  const float partial_data1 = static_cast<float>(raw_t) - s_par_t1;
  const float partial_data2 = partial_data1 * s_par_t2;
  const float t_lin         = partial_data2 + (partial_data1 * partial_data1) * s_par_t3;
  return t_lin;
}

float compensatePressure(uint32_t raw_p, float t_lin) {
  const float t_lin2 = t_lin * t_lin;
  const float t_lin3 = t_lin2 * t_lin;

  float partial_data1 = s_par_p6 * t_lin;
  float partial_data2 = s_par_p7 * t_lin2;
  float partial_data3 = s_par_p8 * t_lin3;
  const float partial_out1 = s_par_p5 + partial_data1 + partial_data2 + partial_data3;

  partial_data1 = s_par_p2 * t_lin;
  partial_data2 = s_par_p3 * t_lin2;
  partial_data3 = s_par_p4 * t_lin3;
  const float partial_out2 = static_cast<float>(raw_p) *
      (s_par_p1 + partial_data1 + partial_data2 + partial_data3);

  const float raw_p_f       = static_cast<float>(raw_p);
  const float raw_p_sq      = raw_p_f * raw_p_f;
  const float raw_p_cube    = raw_p_sq * raw_p_f;
  partial_data1             = raw_p_sq;
  partial_data2             = s_par_p9 + s_par_p10 * t_lin;
  partial_data3             = partial_data1 * partial_data2;
  const float partial_data4 = partial_data3 + raw_p_cube * s_par_p11;

  return partial_out1 + partial_out2 + partial_data4;
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

  if (!writeReg(kRegCmd, kCmdSoftReset)) {
    return false;
  }
  busyWaitUs(kResetWaitUs);

  // Confirm the chip came back up after reset.
  if (!readReg(kRegChipId, id) || id != kChipIdExpected) {
    return false;
  }

  if (!loadCalibration()) {
    return false;
  }

  if (!writeReg(kRegPwrCtrl, kPwrCtrlValue)) {
    return false;
  }
  if (!writeReg(kRegOsr, kOsrValue)) {
    return false;
  }
  if (!writeReg(kRegOdr, kOdrValue)) {
    return false;
  }
  if (!writeReg(kRegConfig, kConfigValue)) {
    return false;
  }
  return true;
}

bool readAndCompensate(CompensatedSample& out) {
  uint8_t b[6];
  if (!readBytes(kRegData0, b, sizeof(b))) {
    return false;
  }
  // Pressure and temperature are 24-bit little-endian per the datasheet.
  const uint32_t raw_p = static_cast<uint32_t>(b[0]) |
                         (static_cast<uint32_t>(b[1]) << 8) |
                         (static_cast<uint32_t>(b[2]) << 16);
  const uint32_t raw_t = static_cast<uint32_t>(b[3]) |
                         (static_cast<uint32_t>(b[4]) << 8) |
                         (static_cast<uint32_t>(b[5]) << 16);

  out.temperature_c = compensateTemperature(raw_t);
  out.pressure_pa   = compensatePressure(raw_p, out.temperature_c);
  return true;
}

}  // namespace cp::libs::bmp388

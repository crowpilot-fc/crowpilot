// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/hal/I2c.h"

#include <Arduino.h>
#include <Wire.h>

#include "src/Config.h"

namespace cp::hal::i2c {

namespace {

bool s_initialized = false;

}  // anonymous namespace

void ensureInit() {
  if (s_initialized) {
    return;
  }
  // I2C bus recovery: before bringing Wire up, toggle SCL up to 9 times
  // with SDA released. After a warm reset, a slave device that was in
  // the middle of an ACK or data byte from the previous firmware run
  // can still be pulling SDA low. The master cannot send a START until
  // SDA goes high. Pulsing SCL up to 9 times lets the slave finish its
  // byte (a byte is 9 SCL cycles: 8 data + 1 ACK) and release SDA.
  // After that we generate a manual STOP (SCL high, SDA low->high) to
  // put the bus in idle, then hand off to the Wire driver.
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, OUTPUT);
  digitalWrite(PIN_I2C_SCL, HIGH);
  delayMicroseconds(10);
  for (int i = 0; i < 9; ++i) {
    if (digitalRead(PIN_I2C_SDA) == HIGH) {
      break;  // bus already idle
    }
    digitalWrite(PIN_I2C_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(PIN_I2C_SCL, HIGH);
    delayMicroseconds(10);
  }
  // Manual STOP condition: SDA low while SCL is high, then SDA high.
  pinMode(PIN_I2C_SDA, OUTPUT);
  digitalWrite(PIN_I2C_SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_I2C_SDA, HIGH);
  delayMicroseconds(10);
  // Now hand off to the Wire driver. The Pi Pico Arduino core requires
  // setSDA / setSCL before begin. setClock can run after begin per the
  // documented examples.
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(I2C_BUS_HZ);
  // Cap every Wire transaction so a missing bus device cannot hang the
  // firmware in init. 25 ms is generous for the longest real transaction
  // at 400 kHz (the burst register read is ~14 bytes, ~350 us) and short
  // enough that a bench operator on a bare FC sees the IMU init fail
  // promptly and the panic LED start blinking. The second arg resets the
  // bus on timeout so a partial transaction does not wedge later calls.
  // The rp2040 Arduino core's Wire spells this as setTimeout(ms, reset)
  // rather than the upstream setWireTimeout(us, reset).
  Wire.setTimeout(25UL, true);
  s_initialized = true;
}

}  // namespace cp::hal::i2c

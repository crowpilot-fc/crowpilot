// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "user_hook/PinGuard.h"

#include <Arduino.h>

#include "Config.h"
#include "airframes/Airframe.h"

namespace cp::user_hook::pins {

namespace {

// One bit per GPIO. Set once a claimed-pin write has been warned about, so
// the warning fires at most once per pin per boot.
uint32_t s_warned_mask = 0;

void warnOnce(uint8_t pin) {
  if (pin < 32) {
    const uint32_t bit = 1UL << pin;
    if (s_warned_mask & bit) {
      return;
    }
    s_warned_mask |= bit;
  }
  if (Serial) {
    Serial.print("WARN: user_tick() write to firmware-claimed pin GP");
    Serial.print(pin);
    Serial.println(" ignored. See docs/reference/pin-maps.md for free pins.");
  }
}

}  // anonymous namespace

bool isClaimed(uint8_t pin) {
  if (pin == PIN_I2C_SDA   || pin == PIN_I2C_SCL)    return true;
  if (pin == PIN_MOTOR_RIGHT || pin == PIN_MOTOR_LEFT) return true;
  for (uint8_t i = 0; i < cp::airframes::N_SERVOS; ++i) {
    if (pin == PIN_SERVOS[i]) {
      return true;
    }
  }
  if (pin == PIN_SBUS_RX) return true;
  if (pin == PIN_SD_MOSI || pin == PIN_SD_MISO ||
      pin == PIN_SD_SCK  || pin == PIN_SD_CS)        return true;
  if (pin == PIN_LED_ONBOARD || pin == PIN_LED_STATUS) return true;
  if (pin == PIN_COMPANION_TX || pin == PIN_COMPANION_RX) return true;
  return false;
}

void guardPinMode(uint8_t pin, uint8_t mode) {
  if (isClaimed(pin)) {
    warnOnce(pin);
    return;
  }
  pinMode(pin, mode);
}

void guardDigitalWrite(uint8_t pin, uint8_t value) {
  if (isClaimed(pin)) {
    warnOnce(pin);
    return;
  }
  digitalWrite(pin, value);
}

void guardAnalogWrite(uint8_t pin, int value) {
  if (isClaimed(pin)) {
    warnOnce(pin);
    return;
  }
  analogWrite(pin, value);
}

}  // namespace cp::user_hook::pins

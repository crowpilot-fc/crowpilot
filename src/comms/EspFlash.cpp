// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/comms/EspFlash.h"

#if BOARD_HAS_ESP_FLASH && BUILD_TARGET == BUILD_TARGET_NATIVE

#include <Arduino.h>

// Config.h includes the active board profile, which is what brings
// PIN_ESP_EN and PIN_ESP_IO0 into scope here.
#include "src/Config.h"

namespace cp::comms::esp_flash {

namespace {

// Match the ESP32-C3 ROM bootloader default and the companion UART rate the
// CLI module already brings the link up at.
constexpr unsigned long kEspBaud = 115200;

// Classic esptool reset timing. The chip needs the EN edge to see IO0 already
// asserted, and a small settle before bytes are accepted.
constexpr uint32_t kResetLowMs   = 100;
constexpr uint32_t kBootSettleMs = 50;

void drive_low(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void drive_high(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

}  // namespace

void enter_bootloader() {
  // Hold IO0 low first so the strap is stable before the EN edge rises.
  drive_low(PIN_ESP_IO0);
  drive_low(PIN_ESP_EN);
  delay(kResetLowMs);
  // Release EN. The ROM samples GPIO9 here; with IO0 low it enters the UART
  // bootloader. IO0 stays held low through the brief settle so the strap
  // does not float during the sample window.
  drive_high(PIN_ESP_EN);
  delay(kBootSettleMs);
  // IO0 can be released once the ROM is running. Leaving it driven low is
  // harmless (the ESP only reads it at reset), but releasing matches what
  // esptool does after the reset sequence.
  pinMode(PIN_ESP_IO0, INPUT);
}

void reset_into_app() {
  // Make sure IO0 is high (or released) so the strap reads high on the EN
  // edge and the ROM jumps to the application instead of the bootloader.
  pinMode(PIN_ESP_IO0, INPUT);
  drive_low(PIN_ESP_EN);
  delay(kResetLowMs);
  drive_high(PIN_ESP_EN);
}

void run_bridge(uint32_t inactivity_ms) {
  // Make sure the UART is up at the ROM baud. Cli.cpp brings Serial2 up at
  // the same rate, but a re-begin here is safe and decouples this entry
  // point from the CLI's init order.
  Serial2.begin(kEspBaud);

  uint32_t last_byte_ms = millis();
  // Small scratch buffers so we read what is available in one shot rather
  // than ping-ponging a byte at a time. Keeps the bridge throughput well
  // above the 115200 baud raw rate.
  uint8_t buf[64];

  while ((millis() - last_byte_ms) < inactivity_ms) {
    // Host -> ESP.
    int n = Serial.available();
    if (n > 0) {
      if (n > static_cast<int>(sizeof(buf))) {
        n = sizeof(buf);
      }
      const int got = Serial.readBytes(buf, n);
      if (got > 0) {
        Serial2.write(buf, got);
        last_byte_ms = millis();
      }
    }

    // ESP -> host.
    n = Serial2.available();
    if (n > 0) {
      if (n > static_cast<int>(sizeof(buf))) {
        n = sizeof(buf);
      }
      const int got = Serial2.readBytes(buf, n);
      if (got > 0) {
        Serial.write(buf, got);
        last_byte_ms = millis();
      }
    }
  }
}

}  // namespace cp::comms::esp_flash

#endif  // BOARD_HAS_ESP_FLASH && native

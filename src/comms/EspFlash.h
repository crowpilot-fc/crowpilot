// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// ESP32-C3 companion passthrough flash support.
//
// Wires the FC into the role esptool normally plays over USB-serial: pulses
// the ESP's EN line to reset it, holds GPIO9 (the IO0 strap) low at reset to
// drop the chip into its permanent UART ROM bootloader, then bridges USB CDC
// bytes to the companion UART so a host running esptool can program the ESP
// without a USB cable on the ESP itself.
//
// Requires two extra control jumpers from the FC to the ESP:
//   PIN_ESP_EN  -> ESP EN  (chip reset, active low)
//   PIN_ESP_IO0 -> ESP GPIO9 (boot strap, sampled at the rising EN edge)
//
// Compiled out unless the board profile defines BOARD_HAS_ESP_FLASH 1.

#include "src/Config.h"

// SITL builds inherit the WeAct board profile (which sets BOARD_HAS_ESP_FLASH)
// but have no Arduino Serial2 or GPIO. The bridge is native-only.
#if BOARD_HAS_ESP_FLASH && BUILD_TARGET == BUILD_TARGET_NATIVE

namespace cp::comms::esp_flash {

// Drive EN low to reset the ESP while holding IO0 low so the ROM sees a low
// strap on GPIO9 at the rising EN edge and enters its UART bootloader.
// Leaves IO0 held low (released later by run_bridge once flashing is done).
// Timings match esptool's classic_reset sequence (about 100 ms low/hold).
void enter_bootloader();

// Drive IO0 high (or release it) and pulse EN low to reset the ESP into its
// application image. Safe to call without enter_bootloader() first.
void reset_into_app();

// Bridge USB CDC to the companion UART at the ESP ROM baud (115200) until
// neither side has sent a byte for inactivity_ms. Both directions are
// drained every pass. Returns when the inactivity timer elapses.
void run_bridge(uint32_t inactivity_ms);

}  // namespace cp::comms::esp_flash

#endif  // BOARD_HAS_ESP_FLASH && native

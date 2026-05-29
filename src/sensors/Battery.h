// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// Battery voltage monitor. Reads a divided pack voltage from an ADC pin,
// low-passes it, auto-detects the cell count at power-up (or uses a fixed
// count), and raises a low-voltage warning. The reading feeds telemetry and
// the pre-arm checks. Compiled in only when ENABLE_BATTERY_MONITOR is set.
//
// Low voltage is a warning in v1, not an automatic flight action. A safe
// low-battery response for a plane is return-to-home, which needs GPS (v2).

namespace cp::sensors::battery {

// Configure the ADC pin and reset state. Call once at startup.
void init();

// Read and filter the pack voltage, detect the cell count on the first valid
// read, and update the low-voltage flag. Call periodically from the loop.
void update();

// Filtered pack voltage in volts.
float voltage();

// Detected (or configured) cell count, 0 if no pack is present.
uint8_t cells();

// Pack voltage divided by the cell count, or 0 with no pack.
float perCellVoltage();

// True when a pack is connected (per-cell above CELL_MIN_PRESENT_V). False on
// USB power with no battery, so the low-voltage and arm checks do not trip.
bool present();

// True when a pack is present and below the per-cell warning threshold.
bool low();

// --- Pure helpers, exposed for host testing ---------------------------------

// Convert a 12-bit ADC count to a pack voltage using the configured reference
// and divider ratio.
float rawToVolts(uint16_t raw);

// Smallest cell count whose per-cell voltage is at or below CELL_MAX_V, for a
// given pack voltage. Returns 0 when the pack reads as absent. Assumes a
// reasonably charged pack at detection time.
uint8_t detectCells(float pack_v);

}  // namespace cp::sensors::battery

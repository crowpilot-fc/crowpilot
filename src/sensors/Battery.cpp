// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/sensors/Battery.h"

#include "src/Config.h"

#if ENABLE_BATTERY_MONITOR

#include "src/hal/Hal.h"

namespace cp::sensors::battery {

namespace {

constexpr float    kAdcFullScale = 4095.0f;  // 12-bit
constexpr uint8_t  kMaxCells     = 8;

float   s_voltage = 0.0f;
uint8_t s_cells   = 0;
bool    s_seeded  = false;

}  // anonymous namespace

float rawToVolts(uint16_t raw) {
  return (static_cast<float>(raw) / kAdcFullScale) * BATTERY_ADC_VREF *
         BATTERY_DIVIDER_RATIO;
}

uint8_t detectCells(float pack_v) {
  // No pack, or below one usable cell: report absent.
  if (pack_v < CELL_MIN_PRESENT_V) {
    return 0;
  }
  for (uint8_t c = 1; c <= kMaxCells; ++c) {
    if (pack_v <= static_cast<float>(c) * CELL_MAX_V) {
      return c;
    }
  }
  return kMaxCells;
}

void init() {
  cp::hal::adc_init(BATTERY_ADC_PIN);
  s_voltage = 0.0f;
  s_cells   = 0;
  s_seeded  = false;
}

void update() {
  const float v = rawToVolts(cp::hal::adc_read_raw(BATTERY_ADC_PIN));

  // Seed the filter on the first read so the voltage does not ramp up from 0.
  if (!s_seeded) {
    s_voltage = v;
    s_seeded  = true;
  } else {
    s_voltage += BATTERY_FILTER_ALPHA * (v - s_voltage);
  }

  // Cell count: fixed if configured, otherwise auto-detect once a pack is
  // present and the reading has settled enough to be sensible.
  if (BATTERY_CELLS > 0) {
    s_cells = (s_voltage >= CELL_MIN_PRESENT_V) ? BATTERY_CELLS : 0;
  } else if (s_cells == 0) {
    s_cells = detectCells(s_voltage);
  } else if (s_voltage < CELL_MIN_PRESENT_V) {
    s_cells = 0;  // pack removed
  }
}

float voltage() {
  return s_voltage;
}

uint8_t cells() {
  return s_cells;
}

float perCellVoltage() {
  return s_cells > 0 ? s_voltage / static_cast<float>(s_cells) : 0.0f;
}

bool present() {
  return s_cells > 0;
}

bool low() {
  return present() && perCellVoltage() < CELL_WARN_V;
}

}  // namespace cp::sensors::battery

#endif  // ENABLE_BATTERY_MONITOR

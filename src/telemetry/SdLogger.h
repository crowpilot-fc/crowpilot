// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

// SD card binary telemetry logger per SPEC.md §5.13 and TELEMETRY_FORMAT.md.
//
// Each tick (rate-limited to 100 Hz by TELEMETRY_LOG_INTERVAL_TICKS in
// Config.h) the logger gathers state from every other module, packs it
// into a 109-byte `TelemetryRecord` (TELEMETRY_FORMAT.md §3), and writes
// the record to the open `.BIN` file on the SD card. The Pi Pico
// Arduino core's `SD.h` library buffers internally; the hot path does
// not flush. Trailing buffer is lost on power loss or crash, which is
// the documented trade per CODING_STANDARDS.md §11.
//
// Files auto-increment as `LOG0001.BIN` through `LOG9999.BIN` at the SD
// card root. The logger scans the card at init, finds the highest
// existing index, and opens the next one. After 9999 the logger halts
// logging and reports the condition over USB serial.

namespace cp::telemetry {

// Bring up the SD card via SPI0, scan for the next log index, open the
// file. Sets the module to active on success. On any failure (no card,
// FAT not mounted, file system full, index exhausted) the module stays
// inactive and the rest of the firmware runs without logging.
//
// When ENABLE_TELEMETRY_LOG_ASYNC is set, init() does NOT touch the SD
// card. SD bring-up happens in core1_setup() on core 1. The first
// is_active() call from core 0 reads what core 1 has set so far. This
// keeps every SD library call on a single core, which avoids racing the
// SD library's internal SPI state.
void init();

// Build one record per tick (rate-limited internally). With async
// logging enabled the record is enqueued in the SPSC ring buffer for
// core 1 to write. With async logging disabled the record is written
// directly to SD here. Cheap when the rate-limit gate is closed.
void tick();

// Core 1 entry points. Called from the sketch's setup1/loop1
// dispatcher in crowpilot.ino. No-ops unless
// ENABLE_TELEMETRY_LOG && ENABLE_TELEMETRY_LOG_ASYNC.
void core1_setup();
void core1_tick();

// True if the logger is active (init succeeded and no write error since).
bool is_active();

// Most recently opened log filename, or empty string if not active.
const char* current_filename();

}  // namespace cp::telemetry

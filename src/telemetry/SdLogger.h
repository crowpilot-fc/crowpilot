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
void init();

// Maybe write one record this tick (gated on the rate-limit counter).
// Cheap when the gate is closed. About 30 microseconds of work when it
// fires plus the SD library's amortized write cost.
void tick();

// True if the logger is active (init succeeded and no write error since).
bool is_active();

// Most recently opened log filename, or empty string if not active.
const char* current_filename();

}  // namespace cp::telemetry

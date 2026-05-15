// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stddef.h>

// Persistent key-value style storage on the RP2350's on-board flash via the
// Arduino-Pico LittleFS filesystem. The parameter system is the only client
// in v1.0; the API is deliberately minimal (whole-file read and write).
//
// LittleFS lives in a flash region carved out by the Arduino IDE "Flash
// Size" setting. If that setting allocates no filesystem space, begin()
// fails and the caller falls back to volatile defaults. Storage failure is
// never fatal.

namespace cp::storage {

// Mount the filesystem, formatting it once if the mount fails (a blank
// flash region has no filesystem yet). Returns false if storage is
// unavailable, in which case writeText() and readText() also fail.
bool begin();

// Write `len` bytes of `data` to `path`, replacing any existing file.
// Returns false if storage is unavailable or the write did not complete.
bool writeText(const char* path, const char* data, size_t len);

// Read up to `buf_len - 1` bytes from `path` into `buf` and null-terminate.
// Returns the byte count read, 0 if the file is empty, or -1 if storage is
// unavailable or the file does not exist.
int readText(const char* path, char* buf, size_t buf_len);

}  // namespace cp::storage

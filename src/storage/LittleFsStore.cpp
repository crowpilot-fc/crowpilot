// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/storage/LittleFsStore.h"

#include <Arduino.h>
#include <LittleFS.h>

namespace cp::storage {

namespace {

// Mount state. begin() is idempotent; later calls return the cached
// result rather than re-mounting.
bool s_attempted = false;
bool s_mounted   = false;

}  // namespace

bool begin() {
  if (s_attempted) {
    return s_mounted;
  }
  s_attempted = true;

  if (LittleFS.begin()) {
    s_mounted = true;
    return true;
  }

  // A blank flash region has no filesystem. Format once, then retry.
  if (LittleFS.format() && LittleFS.begin()) {
    s_mounted = true;
    return true;
  }

  s_mounted = false;
  return false;
}

bool writeText(const char* path, const char* data, size_t len) {
  if (!s_mounted) {
    return false;
  }
  File f = LittleFS.open(path, "w");
  if (!f) {
    return false;
  }
  const size_t written = f.write(reinterpret_cast<const uint8_t*>(data), len);
  f.close();
  return written == len;
}

int readText(const char* path, char* buf, size_t buf_len) {
  if (!s_mounted || buf_len == 0) {
    return -1;
  }
  File f = LittleFS.open(path, "r");
  if (!f) {
    return -1;
  }
  const int n = f.read(reinterpret_cast<uint8_t*>(buf), buf_len - 1);
  f.close();
  if (n < 0) {
    return -1;
  }
  buf[n] = '\0';
  return n;
}

}  // namespace cp::storage

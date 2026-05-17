// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// SITL stand-in for the LittleFS parameter store. The host-build
// checkpoint keeps no persistent storage: writes are accepted and
// discarded, reads report nothing saved, so the parameter registry
// always comes up at its compile-time defaults.

#include "src/storage/LittleFsStore.h"

namespace cp::storage {

bool begin() {
  return true;
}

bool writeText(const char*, const char*, size_t) {
  return true;
}

int readText(const char*, char*, size_t) {
  return -1;
}

}  // namespace cp::storage

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// SITL stand-in for the SD-card telemetry logger. The host-build
// checkpoint does not write a log: the logger reports itself inactive.
// Writing the binary log to a host file is a later step.

#include "src/telemetry/SdLogger.h"

namespace cp::telemetry {

void init() {}

void tick() {}

bool is_active() {
  return false;
}

const char* current_filename() {
  return "";
}

}  // namespace cp::telemetry

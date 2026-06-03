// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

// CrowPilot Arduino entry shim.
//
// All firmware logic lives under src/. This file exists only because the
// Arduino build system needs a sketch at the repository root. setup() /
// loop() run on core 0 and delegate to cp::core. setup1() / loop1() run
// on core 1; they dispatch to whatever firmware module needs core 1 work,
// gated by its ENABLE_* flag. Currently:
//
//   - ENABLE_ONBOARD_WIFI (Pico 2 W): the WiFi access point, HTTP server,
//     and WebSocket bridge run on core 1.
//   - ENABLE_TELEMETRY_LOG && ENABLE_TELEMETRY_LOG_ASYNC: the SD writes
//     run on core 1, fed by a SPSC ring buffer the flight loop on core 0
//     pushes records into.
//
// Both modules are non-flight-critical and tolerate the other taking
// time on core 1 because each does its own bounded work per tick.

#include "src/core/Loop.h"
#include "src/telemetry/SdLogger.h"
// Loop.h transitively pulls in src/Config.h, which is where ENABLE_*
// preprocessor flags live. We do NOT #include "src/Config.h" directly
// from this .ino because the Arduino preprocessor double-defines its
// constexpr block when the sketch root is on the include path AND a
// transitive include also pulls it. The transitive path is enough.

#if ENABLE_ONBOARD_WIFI
#include "src/comms/OnboardWifi.h"
#endif

void setup() {
  cp::core::init();
}

void loop() {
  cp::core::tick();
}

void setup1() {
#if ENABLE_ONBOARD_WIFI
  cp::comms::onboard_wifi::init();
#endif
#if ENABLE_TELEMETRY_LOG && ENABLE_TELEMETRY_LOG_ASYNC
  cp::telemetry::core1_setup();
#endif
}

void loop1() {
#if ENABLE_ONBOARD_WIFI
  cp::comms::onboard_wifi::tick();
#endif
#if ENABLE_TELEMETRY_LOG && ENABLE_TELEMETRY_LOG_ASYNC
  cp::telemetry::core1_tick();
#endif
}

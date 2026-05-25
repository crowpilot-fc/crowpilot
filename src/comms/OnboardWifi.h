// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <Arduino.h>  // for the Stream type cli_stream() returns.

// Onboard WiFi companion for boards with an integrated radio (the Pico 2 W's
// CYW43439). The flight controller raises its own WiFi access point and
// serves the companion UI itself, so a separate ESP companion is not needed.
//
// Everything here runs on core1 (Arduino setup1 / loop1), so the network
// stack never steals time from the 1 kHz flight loop on core0. Compiled out
// unless ENABLE_ONBOARD_WIFI is set; see Config.h.

namespace cp::comms::onboard_wifi {

// Bring up the radio, the access point, the web server, and the WebSocket
// bridge. Call once from setup1(), on core1.
void init();

// Service the web server, the WebSocket, and the cp byte bridge. Call
// repeatedly from loop1(), on core1.
void tick();

// The Stream the core0 cp CLI registers as a command channel. Reads pull
// command bytes that arrived over the WebSocket; writes are queued back out to
// the browser. Backed by lock-free cross-core ring buffers.
Stream* cli_stream();

}  // namespace cp::comms::onboard_wifi

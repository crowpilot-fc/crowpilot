// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

// Onboard WiFi companion for boards with an integrated radio (the Pico 2 W's
// CYW43439). The flight controller raises its own WiFi access point and
// serves the companion UI itself, so a separate ESP companion is not needed.
//
// Everything here runs on core1 (Arduino setup1 / loop1), so the network
// stack never steals time from the 1 kHz flight loop on core0. Compiled out
// unless ENABLE_ONBOARD_WIFI is set; see Config.h.

namespace cp::comms::onboard_wifi {

// Bring up the radio, the access point, and the web server. Call once from
// setup1(), on core1.
void init();

// Service the web server and the radio link. Call repeatedly from loop1().
void tick();

}  // namespace cp::comms::onboard_wifi

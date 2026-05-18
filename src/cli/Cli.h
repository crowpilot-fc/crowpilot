// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

// Serial command interface for the browser-based configurator. Speaks a
// line-oriented text protocol: a handshake, a parameter dump, and
// set/save/load/defaults commands against the runtime parameter registry.
// Every reply line is prefixed with "cp " so the host can tell protocol
// output apart from the DEBUG_PRINT_* streams that share the port.
//
// The interface runs on the USB CDC serial and, on native builds with
// ENABLE_COMPANION_CLI set, also on the companion UART, so an ESP WiFi
// module can bridge the same protocol to a phone. Bench tool only,
// compiled out when ENABLE_CONFIG_CLI is 0.

namespace cp::cli {

// Reset the line buffers and open the companion UART. Call once from
// setup(), after the parameter registry is initialised.
void init();

// Drain whatever serial bytes are waiting on each channel and dispatch any
// completed command line. Non-blocking. Call once per loop tick.
void poll();

// True while a channel has live telemetry streaming on (it issued
// "cp stream on"). Always false when ENABLE_CONFIG_CLI is 0.
bool streaming();

// Write one telemetry line to the channel that requested streaming. The
// caller passes the line body without the trailing newline. A no-op when
// no channel is streaming or ENABLE_CONFIG_CLI is 0.
void emit_telemetry(const char* line);

}  // namespace cp::cli

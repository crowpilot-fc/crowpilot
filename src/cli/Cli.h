// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

// Serial command interface for the browser-based configurator. Speaks a
// line-oriented text protocol over the USB CDC serial: a handshake, a
// parameter dump, and set/save/load/defaults commands against the runtime
// parameter registry. Every reply line is prefixed with "cp " so the host
// can tell protocol output apart from the DEBUG_PRINT_* streams that share
// the port. Bench tool only, compiled out when ENABLE_CONFIG_CLI is 0.

namespace cp::cli {

// Reset the line buffer. Call once from setup(), after the parameter
// registry is initialised.
void init();

// Drain whatever serial bytes are waiting and dispatch any completed
// command line. Non-blocking. Call once per loop tick.
void poll();

}  // namespace cp::cli

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

#include "src/estimation/Attitude.h"

// 2-axis pan/tilt camera gimbal stabilization. Reads headtracker channels
// from the receiver, drives two extra servos with a rate-damped correction
// from the IMU. Result: head still controls aim, plane shake is smoothed
// out of the video.
//
// Math (per axis):
//   servo_us = headtracker_us + trim_us - gain * gyro_rate_dps
//
// Where gain is in microseconds-per-degrees-per-second. gain=0 disables
// stabilization (pure passthrough). Typical good values are 1-5.
//
// Entire feature is gated by the GIMBAL_ENABLE runtime param. When off,
// init() does not attach the servo pins, leaving them free for other use.
// When on, init() attaches PIN_GIMBAL_PAN and PIN_GIMBAL_TILT and tick()
// drives them every main loop iteration.
//
// Pin assignment is per-board in the board profile. The Tiny exposes
// PIN_GIMBAL_PAN = GP3 and PIN_GIMBAL_TILT = GP28; the WeAct V10 exposes
// GP27 and GP28. A board that supports the gimbal declares the two pin
// constants and also defines BOARD_HAS_GIMBAL to 1, which is what this
// module tests. Boards that leave BOARD_HAS_GIMBAL at its default of 0
// compile this module to a no-op and keep the pins free.
//
// The pins are constexpr, not macros, so the guard has to be the
// BOARD_HAS_GIMBAL macro. An earlier #ifdef PIN_GIMBAL_PAN was never true
// on any board and silently disabled the whole feature.

namespace cp::control::gimbal {

// Read the enable param. If on, attach the gimbal servos and center them.
// If off, leave the pins untouched. Safe to call multiple times.
void init();

// True if the gimbal is currently active (enabled at init and pins are
// attached). False otherwise.
bool active();

// One stabilization tick. Reads the configured pan and tilt channels from
// `channels` (the failsafe-effective channel snapshot, microseconds, 1-based
// channel indices into 0-based array), reads the body rates from `rates`,
// writes the rate-damped output to the gimbal servos. No-op when not active.
void tick(const uint16_t* channels,
          const cp::estimation::attitude::BodyRates& rates);

}  // namespace cp::control::gimbal

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
// PIN_GIMBAL_PAN = GP3 and PIN_GIMBAL_TILT = GP28. Other boards add their
// own constants if they want to support the gimbal. Boards that omit
// PIN_GIMBAL_PAN cause this module to compile to a no-op.

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

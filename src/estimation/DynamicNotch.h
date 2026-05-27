// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// RPM-tracking dynamic gyro notch. See docs/developer-guide/algorithms.md.
//
// The dominant gyro vibration peak on a powered airframe is the motor
// rotation frequency, which moves with throttle. A fixed notch only catches
// one RPM. This module reads the per-motor electrical RPM that bidirectional
// DShot reports through the HAL, turns it into a mechanical vibration
// frequency, and retunes the estimator's gyro notch to follow it.
//
// The target is the mean of the running motors' frequencies, clamped to
// [DYN_NOTCH_MIN_HZ, DYN_NOTCH_MAX_HZ]. The applied center is slew-rate
// limited so the notch glides rather than jumps. When no motor reports RPM
// (any non-bidirectional protocol, or motors stopped) the module hands the
// notch back to the fixed GYRO_NOTCH_CENTER_HZ. Compiled in only when
// ENABLE_DYNAMIC_NOTCH is set.

namespace cp::estimation::dynnotch {

// Reset the tracker. The notch returns to the fixed center until the first
// update sees running motors.
void init();

// Call once per loop tick. Internally rate-divides by DYN_NOTCH_UPDATE_DIV,
// reads the HAL eRPM, and retunes the gyro notch.
void update();

// Pure helper, exposed for host testing. Mean mechanical vibration frequency
// in Hz over the running motors (eRPM > 0), clamped to the configured band.
// Returns 0 when no motor is running, which means hand the notch back to the
// fixed center.
float computeTargetHz(const uint32_t* erpm, uint8_t count);

}  // namespace cp::estimation::dynnotch

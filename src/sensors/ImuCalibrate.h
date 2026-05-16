// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

// IMU bias calibration. See docs/developer-guide/algorithms.md.
//
// A MEMS gyroscope and accelerometer each have a fixed zero offset. Left
// uncorrected the gyro offset drifts the attitude estimate. This routine
// measures the six offsets so they can be removed from every later
// sample.
//
// It is a one-shot bench routine, gated by ENABLE_IMU_CALIBRATION in
// Config.h, run at boot before any motor output is enabled. With the
// airframe held stationary and level (belly down, so the body z-axis
// reads gravity) it averages a batch of raw samples, removes the one g
// of gravity from the vertical accelerometer axis, prints the six bias
// values formatted for src/Config.h, and halts. The pilot pastes the
// values, sets ENABLE_IMU_CALIBRATION back to 0, and reflashes.

namespace cp::sensors::imu_calibrate {

// Run the calibration and halt. Requires a working IMU. Per the coding
// standard this is one of the few places an endless loop is acceptable,
// because the routine is a deliberate, motors-disabled bench step.
[[noreturn]] void run();

}  // namespace cp::sensors::imu_calibrate

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

namespace cp::core {

// Bring up every module in the documented init order. Called once from
// Arduino setup(). Does not return on a fatal init failure.
void init();

// Execute one superloop iteration and regulate to the configured loop
// period. Called from Arduino loop().
void tick();

// Most recent measured loop period in microseconds. Updated each tick
// by the period-measurement code. Zero on the very first tick before a
// delta has been captured. Used by the telemetry logger so each record
// carries the per-iteration period rather than the 1-second average.
uint32_t last_loop_period_us();

// Count of ticks since boot whose work exceeded the loop period. A
// nonzero and growing value means the 1 kHz control rate is degraded.
uint32_t loop_overrun_count();

// True while the firmware is in the IMU-loss-degraded state: a plane is
// being held in MANUAL passthrough, or a tailsitter / quad is being held
// disarmed, because IMU health dropped after arming. Read by the
// telemetry logger so the post-flight log records when this fired.
bool imu_loss_degraded();

}  // namespace cp::core

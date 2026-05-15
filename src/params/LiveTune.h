// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// In-flight gain tuning over two transmitter channels. When ENABLE_LIVE_TUNE
// is set, the channels named by LIVE_TUNE_CH_KP and LIVE_TUNE_CH_KD scale the
// active Kp and Kd gains by up to +/- LIVE_TUNE_RANGE. The PID controller
// multiplies its blended gains by the multipliers below every tick.
//
// A save gesture (disarmed, yaw stick held hard left for two seconds) bakes
// the current multipliers into the persistent parameters and writes them to
// flash, then recenters the channels so the multipliers return to 1.0.

namespace cp::params::live {

// Reset multipliers to 1.0 and capture the channel neutral points.
void init();

// Read the live-tune channels and update the multipliers. `channels` is the
// raw receiver channel array (microseconds, 1-based channels at 0-based
// indices). `armed` gates the save gesture. Cheap; safe in the hot loop.
// A no-op when ENABLE_LIVE_TUNE is 0.
void update(const uint16_t* channels, bool armed);

// Multiplier applied to every blended Kp gain. 1.0 when live tune is off or
// the channel sits at neutral.
float kpMultiplier();

// Multiplier applied to every blended Kd gain.
float kdMultiplier();

}  // namespace cp::params::live

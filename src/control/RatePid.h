// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include "src/estimation/Attitude.h"

// Rate (acro) controller for the multirotor. Each axis tracks a commanded
// body rate: the stick sets a target rate in degrees per second and the
// controller drives the rate error to zero. There is no attitude reference,
// so the craft holds whatever attitude the pilot leaves it in. This is the
// inner loop; a future angle outer loop would feed rate setpoints into it.
// Reads the gyro-filtered body rates from the estimator. Gains are
// provisional, set by bench tuning.

namespace cp::control::rate {

struct Output {
  float roll;
  float pitch;
  float yaw;
};

void init();

// One step. Setpoints are body rates in degrees per second. `flying` gates
// the integrators so they do not wind up on the ground.
void update(float roll_rate_sp_dps,
            float pitch_rate_sp_dps,
            float yaw_rate_sp_dps,
            const cp::estimation::attitude::BodyRates& rates,
            bool flying,
            float dt_s);

const Output& output();

}  // namespace cp::control::rate

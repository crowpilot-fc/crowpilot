// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// Hand-launch assist for the plane airframes. See
// docs/developer-guide/algorithms.md.
//
// The sequence, gyro and accelerometer only, no GPS: the pilot arms with the
// throttle at idle, holds the plane, and throws it. A forward acceleration
// spike marks the throw. For a fixed window after that, the assist commands a
// wings-level climb-out: it spools the motor to a launch throttle and holds a
// nose-up pitch attitude through the plane stabilizer, so the pilot can let go
// of the sticks during the throw. The window ends on a timeout, on the pilot
// moving a stick, or on disarm, and the assist does not re-trigger until the
// next arm cycle.
//
// Intended for the angle and horizon modes, where the stabilizer holds the
// commanded attitude. Compiled in only when ENABLE_LAUNCH_ASSIST is set.

namespace cp::control::launch {

// The override the assist applies to the pilot's desired state while a launch
// is active. When active is false the fields are unused and the pilot's own
// commands stand.
struct Override {
  bool  active;
  float throttle;   // normalized [0, 1]
  float roll_deg;   // roll attitude setpoint (0 = wings level)
  float pitch_deg;  // pitch attitude setpoint (nose-up climb-out)
};

// Reset the state machine to idle.
void init();

// One step. `armed` is the current arm state, `ax_g` the body-x (forward)
// acceleration in g, `throttle` the pilot's normalized throttle, `roll_stick`
// and `pitch_stick` the normalized [-1, 1] sticks (used to detect the pilot
// taking over), and `dt_s` the loop period. Returns the override to apply.
Override update(bool armed, float ax_g, float throttle,
                float roll_stick, float pitch_stick, float dt_s);

}  // namespace cp::control::launch

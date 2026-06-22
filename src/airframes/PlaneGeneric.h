// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include "src/airframes/Airframe.h"

// Generic plane mixer. Composes the right behaviour at runtime from three
// integer params: PLANE_MOTORS_COUNT (1 or 2), PLANE_TAIL_STYLE (traditional /
// V-tail / none), and PLANE_AILERON_COUNT (0 / 1 / 2). Twelve valid plane
// combinations from those primitives. Pilot picks the combination via the
// Crowpilot Companion app; no re-flash needed.
//
// Builds that set AIRFRAME = AIRFRAME_PLANE_GENERIC compile this mixer and
// use it as the airframe-level update() driven by the plane stabilizer.

namespace cp::airframes::plane_generic {

// Pure mix function. Takes the plane-stabilizer outputs in the normalized
// [-1, +1] axis-command range (with the stab authority blend already
// applied by PlaneStab). Reads the runtime params for shape and writes the
// output slots that are active for the chosen shape. Slots that are not
// active are left at neutral (motors at 0, servos at 0.5).
void mix(float throttle, float roll, float pitch, float yaw,
         float flap, float brake, cp::airframes::Output& out);

}  // namespace cp::airframes::plane_generic

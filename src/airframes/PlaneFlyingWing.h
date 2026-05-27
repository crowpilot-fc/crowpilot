// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include "src/airframes/Airframe.h"

// Flying-wing (delta) mixer. One engine plus two elevons, no rudder. Each
// elevon carries pitch (both surfaces move together) and roll (they move
// opposite), the same allocation the tailsitter uses in forward flight, here
// driven by the plane stabilizer. The cp::airframes facade for this airframe
// is implemented in PlaneFlyingWing.cpp and reads the plane stabilizer output.

namespace cp::airframes::plane_flying_wing {

// Map stabilized commands to the flying-wing actuator outputs.
//   throttle - normalized 0..1, applied to the engine.
//   roll     - normalized -1..1, differential elevon.
//   pitch    - normalized -1..1, common-mode elevon.
//   yaw      - normalized -1..1, ignored (a pure flying wing has no rudder).
// Every output is clamped to its valid range before return.
void mix(float throttle, float roll, float pitch, float yaw,
         cp::airframes::Output& out);

}  // namespace cp::airframes::plane_flying_wing

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include "src/airframes/Airframe.h"

// Twin-engine cargo plane (Caribou) mixer. Two motors plus four control
// surfaces: left and right ailerons, elevator, rudder. The cp::airframes
// facade for this airframe is implemented in PlaneTwinCargo.cpp and reads
// the plane stabilizer output; the pure mixer below is split out so the
// surface mapping can be reasoned about on its own.

namespace cp::airframes::plane_twin_cargo {

// Map stabilized commands to the twin-cargo actuator outputs.
//   throttle - normalized 0..1, applied to both motors.
//   roll     - normalized -1..1, drives the ailerons differentially.
//   pitch    - normalized -1..1, drives the elevator.
//   yaw      - normalized -1..1, drives the rudder, and the motors
//              differentially when ENABLE_DIFF_THRUST_YAW is set.
// Every output is clamped to its valid range before return.
void mix(float throttle, float roll, float pitch, float yaw,
         cp::airframes::Output& out);

}  // namespace cp::airframes::plane_twin_cargo

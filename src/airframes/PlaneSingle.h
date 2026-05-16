// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include "src/airframes/Airframe.h"

// Single-engine plane (Gee Bee) mixer. One engine plus four control
// surfaces: left and right ailerons, elevator, rudder. Simpler than the
// twin-cargo plane: no differential thrust, one throttle command. The
// cp::airframes facade for this airframe is implemented in
// PlaneSingle.cpp and reads the plane stabilizer output.

namespace cp::airframes::plane_single {

// Map stabilized commands to the single-engine actuator outputs.
//   throttle - normalized 0..1, applied to the engine.
//   roll     - normalized -1..1, drives the ailerons differentially.
//   pitch    - normalized -1..1, drives the elevator.
//   yaw      - normalized -1..1, drives the rudder.
// Every output is clamped to its valid range before return.
void mix(float throttle, float roll, float pitch, float yaw,
         cp::airframes::Output& out);

}  // namespace cp::airframes::plane_single

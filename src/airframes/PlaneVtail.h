// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include "src/airframes/Airframe.h"

// V-tail plane mixer. One engine, two ailerons, two ruddervators. The ailerons
// are differential for roll. The ruddervators mix pitch (both surfaces move
// together) and yaw (they move opposite). Driven by the plane stabilizer. The
// cp::airframes facade for this airframe is implemented in PlaneVtail.cpp and
// reads the plane stabilizer output.

namespace cp::airframes::plane_vtail {

// Map stabilized commands to the V-tail actuator outputs.
//   throttle - normalized 0..1, applied to the engine.
//   roll     - normalized -1..1, differential ailerons.
//   pitch    - normalized -1..1, common-mode ruddervator.
//   yaw      - normalized -1..1, differential ruddervator.
// Every output is clamped to its valid range before return.
void mix(float throttle, float roll, float pitch, float yaw,
         float flap, float brake, cp::airframes::Output& out);

}  // namespace cp::airframes::plane_vtail

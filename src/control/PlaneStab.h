// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

#include "src/control/DesiredState.h"
#include "src/estimation/Attitude.h"

// Plane stabilization subsystem. A wing leveler, pitch-attitude hold, yaw
// damper, and optional barometric altitude hold, shared by every fixed-wing
// airframe. It is independent of the tailsitter angle PID: a build selects
// one or the other by airframe. Both compile in; only one feeds the mixer
// at runtime.
//
// The flight-mode switch (CHANNEL_STAB) is read as three positions and mapped
// to MANUAL, RATE, ANGLE, or HORIZON per the PLANE_MODE_SW_* config. MANUAL is
// full passthrough, every stabilization term drops out. RATE damps to a
// commanded body rate without self-leveling. ANGLE self-levels to a commanded
// attitude. HORIZON blends ANGLE near center stick to RATE at full stick.
// Altitude hold is a second switch (CHANNEL_ALT_HOLD) and only engages when a
// healthy barometer is present.

namespace cp::control::plane_stab {

struct Output {
  float   roll;      // normalized aileron command, [-1, +1]
  float   pitch;     // normalized elevator command, [-1, +1]
  float   yaw;       // normalized rudder command, [-1, +1]
  float   throttle;  // normalized throttle, [0, 1]
  uint8_t mode;      // active PLANE_MODE_* this tick
  bool    passthrough_active;
  bool    alt_hold_active;
};

// Coordinated-turn feedforward for a given bank angle in degrees. Returns the
// rudder feedforward (auto-rudder, proportional to sin(bank)) and the pitch
// up-elevator compensation (about 1/cos(bank) - 1, with the bank clamped to
// TURN_COMP_MAX_BANK_DEG). Pure and side-effect-free, exposed for testing.
struct TurnFeedforward {
  float rudder;
  float pitch;
};
TurnFeedforward turnCoordination(float bank_deg);

// Reset the stabilizer state (integrators, altitude target, climb-rate
// filter).
void init();

// One stabilization step. `euler` and `rates` are the forward-flight
// attitude estimate and body rates. `desired` carries the pilot's angle
// setpoints, throttle, and passthrough stick values. `baro_altitude_m` and
// `baro_valid` feed altitude hold. `channels` is the failsafe-effective
// channel array (microseconds, 1-based channels at 0-based indices).
void update(const cp::estimation::attitude::Euler& euler,
            const cp::estimation::attitude::BodyRates& rates,
            const cp::control::desired::State& desired,
            float baro_altitude_m,
            bool baro_valid,
            const uint16_t* channels,
            float dt_s);

// Latest stabilizer output. Always safe to call after init.
const Output& output();

}  // namespace cp::control::plane_stab

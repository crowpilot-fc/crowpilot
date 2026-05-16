// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include "src/estimation/Attitude.h"

// Per-axis PID stabilizer. See docs/developer-guide/algorithms.md.
//
// One controller per axis. Roll and pitch stabilize on the attitude
// error from cp::estimation::attitude::errorToReference(). Yaw has no
// absolute reference without a magnetometer, so it stabilizes on a
// rate error, the pilot's yaw-rate setpoint minus the measured yaw
// rate.
//
// All three axes share one PID form:
//
//   demand = Kp * error + Ki * integral(error) - Kd * body_rate
//
// The derivative term is taken from the measured body rate rather than
// from the error, so a stepped setpoint does not produce a derivative
// kick. For roll and pitch the body rate is the time derivative of the
// controlled angle, the textbook derivative-on-measurement. For yaw,
// which is a rate loop, the same term adds rate damping.
//
// Gains are regime-scheduled. Hover and forward flight have different
// dynamics, so each axis interpolates continuously between a hover gain
// set and a forward-flight gain set by the transition fader. The gains
// are read from the runtime parameter registry, so transmitter live
// tuning takes effect immediately.
//
// The integrator is clamped for anti-windup and is held at zero while
// the aircraft is not flying, so it does not wind up on the bench.
//
// Each axis demand is clamped to the normalized range [-1, 1], ready
// for the mixer.

namespace cp::control::pid {

struct Output {
  float roll;   // normalized roll demand, [-1, 1]
  float pitch;  // normalized pitch demand, [-1, 1]
  float yaw;    // normalized yaw demand, [-1, 1]
};

// Reset the controller. Clears the three integrators and the output.
void init();

// One stabilizer step.
//   attitude_error: roll and pitch errors in degrees, from
//     cp::estimation::attitude::errorToReference(). The yaw component
//     is unused.
//   body_rates: measured body rates in degrees per second, for the
//     derivative term.
//   yaw_rate_setpoint_dps: the pilot's commanded yaw rate.
//   fader: transition value, 1.0 hover to 0.0 forward, schedules the
//     gains.
//   flying: true when the aircraft is making thrust. While false the
//     integrators are held at zero.
//   dt_s: elapsed loop period in seconds.
void update(const cp::estimation::attitude::Euler& attitude_error,
            const cp::estimation::attitude::BodyRates& body_rates,
            float yaw_rate_setpoint_dps,
            float fader,
            bool flying,
            float dt_s);

// The latest per-axis demands. Always safe to call after init.
const Output& output();

}  // namespace cp::control::pid

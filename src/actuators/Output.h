// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

#include "airframes/Airframe.h"

// Actuator output stage. Reads the mixer's per-actuator commands, applies
// the arm/throttle-cut state machine to the motor pulses, and drives
// OneShot125 ESC pulses on the motor pins plus standard 1000-2000 us
// servo PWM on the servo pins.
//
// Safety. The boot state is NOT_ARMED and motor pulses go out at the
// sub-valid `ESC_DISARM_PULSE_US` (120 us) so ESCs see "no signal" and
// stay silent. Arming requires both ch5 LOW (< 1500 us) AND the
// throttle stick at idle (raw <= ARM_THROTTLE_MAX_US, default 1050 us).
// ch5 HIGH always disarms regardless of throttle.
//
// See SPEC.md §5.12 and ALGORITHMS.md §7, §10.

namespace cp::actuators {

enum class ArmState {
  NOT_ARMED,
  ARMED,
};

// Latest pulse widths actually written to the motor and servo hardware
// this tick. Used by the telemetry logger so the log carries the
// commanded value rather than the upstream mixer value. The motor side
// reflects the arm gate (motors at the disarm pulse when NOT_ARMED).
struct LatestPulses {
  uint16_t motor_us[cp::airframes::N_MOTORS];
  uint16_t servo_us[cp::airframes::N_SERVOS];
};

// Configure motor pins as outputs (driven LOW) and attach the servo
// instances. Sets the arm state to NOT_ARMED.
void init();

// One actuator step. Updates the arm-state machine from ch5 and the
// throttle stick, computes the motor and servo pulse widths from the
// mixer output, emits a synchronous OneShot125 burst (all motor pins
// HIGH together, each falls at its scheduled time) and writes the
// servo microsecond values. Motors emit the sub-valid disarm pulse
// when NOT_ARMED. Servos respond regardless of arm state.
//
// throttle_norm is from cp::control::desired::current().throttle.
// ch5_us is the failsafe-effective ch5 microseconds.
void update(const cp::airframes::Output& mix,
            float throttle_norm,
            uint16_t ch5_us);

ArmState arm_state();

const LatestPulses& latest_pulses();

}  // namespace cp::actuators

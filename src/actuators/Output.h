// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

#include "src/airframes/Airframe.h"

// Actuator output stage. Reads the mixer's per-actuator commands, applies
// the arm/throttle-cut state machine to the motor pulses, and drives the
// motor pins via the configured MOTOR_PROTOCOL plus standard 1000-2000 us
// servo PWM on the servo pins.
//
// Safety. The boot state is NOT_ARMED and motor pulses go out at
// `ESC_DISARM_PULSE_US`, the motor-stopped width, so the motors do not
// spin. Arming requires ch5 LOW (< 1500 us), the throttle stick at idle
// (<= ARM_THROTTLE_MAX_US, default 1050 us), and the arm switch to have
// been seen in the disarm position at least once since boot. ch5 HIGH
// always disarms regardless of throttle.
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
  uint16_t servo_us[cp::airframes::N_SERVO_SLOTS];
};

// Configure motor pins as outputs (driven LOW) and attach the servo
// instances. Sets the arm state to NOT_ARMED.
void init();

// One actuator step. Updates the arm-state machine from the arm channel and
// the throttle stick, computes the motor and servo pulse widths from the
// mixer output, emits the motor pulses via the configured protocol, and
// writes the servo microsecond values. Motors emit the motor-stopped
// disarm pulse when NOT_ARMED. Servos respond regardless of arm state.
//
// throttle_norm is from cp::control::desired::current().throttle.
// arm_us is the failsafe-effective arm-channel microseconds.
// prearm_ok gates the disarmed-to-armed transition: arming is refused while
// it is false (a failed pre-arm check). Disarm is never gated. Pass true to
// keep the prior behavior.
void update(const cp::airframes::Output& mix,
            float throttle_norm,
            uint16_t arm_us,
            bool prearm_ok);

ArmState arm_state();

const LatestPulses& latest_pulses();

}  // namespace cp::actuators

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
#include "crowpilot/safety.hpp"

namespace crowpilot {

FlightState SafetyMachine::update(const SafetyInputs& inputs) {
    switch (state_) {
    case FlightState::boot:
        state_ = FlightState::self_test;
        break;

    case FlightState::self_test:
        if (inputs.self_tests_complete) {
            state_ = inputs.self_tests_ok ? FlightState::disarmed
                                          : FlightState::fault;
        }
        break;

    case FlightState::disarmed:
        if (inputs.receiver_healthy && !inputs.arm_requested) {
            observed_arm_low_ = true;
        }
        if (inputs.receiver_healthy && observed_arm_low_ &&
            inputs.arm_requested && inputs.throttle_low) {
            state_ = FlightState::armed;
        }
        break;

    case FlightState::armed:
        if (!inputs.receiver_healthy) {
            state_ = FlightState::failsafe;
        } else if (!inputs.arm_requested) {
            observed_arm_low_ = true;
            state_ = FlightState::disarmed;
        }
        break;

    case FlightState::failsafe:
        if (inputs.receiver_healthy && !inputs.arm_requested) {
            observed_arm_low_ = true;
            state_ = FlightState::disarmed;
        }
        break;

    case FlightState::fault:
        break;
    }

    return state_;
}

void SafetyMachine::force_fault() {
    state_ = FlightState::fault;
}

FlightState SafetyMachine::state() const {
    return state_;
}

bool SafetyMachine::output_authority_enabled() const {
    return state_ == FlightState::armed;
}

}  // namespace crowpilot

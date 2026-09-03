// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
#pragma once

#include <cstdint>

namespace crowpilot {

enum class FlightState : std::uint8_t {
    boot,
    self_test,
    disarmed,
    armed,
    failsafe,
    fault,
};

struct SafetyInputs {
    bool self_tests_complete = false;
    bool self_tests_ok = false;
    bool receiver_healthy = false;
    bool arm_requested = false;
    bool throttle_low = false;
};

class SafetyMachine {
public:
    [[nodiscard]] FlightState update(const SafetyInputs& inputs);
    void force_fault();

    [[nodiscard]] FlightState state() const;
    [[nodiscard]] bool output_authority_enabled() const;

private:
    FlightState state_ = FlightState::boot;
    bool observed_arm_low_ = false;
};

}  // namespace crowpilot

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
#pragma once

#include "crowpilot/output.hpp"

namespace crowpilot {

class PwmOutputs {
public:
    [[nodiscard]] bool initialize(const OutputPulses& initial_pulses);
    void apply(const OutputPulses& pulses);

private:
    bool initialized_ = false;
};

}  // namespace crowpilot

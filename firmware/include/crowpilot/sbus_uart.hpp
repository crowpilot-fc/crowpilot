// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
#pragma once

#include "crowpilot/sbus.hpp"

namespace crowpilot {

class SbusUart {
public:
    void initialize();
    [[nodiscard]] bool poll(SbusFrame& latest_frame);

private:
    SbusParser parser_{};
};

}  // namespace crowpilot

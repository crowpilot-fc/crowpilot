// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

namespace cp::actuators::esc_calibrate {

// One-shot ESC calibration routine per ALGORITHMS.md §7.6 and
// SPEC.md §4.6.
//
// Procedure when invoked at boot (requires ENABLE_ESC_CALIBRATION = 1
// in Config.h and the ESC battery DISCONNECTED at flash time):
//
//   1. Firmware prints the procedure on USB serial.
//   2. Firmware emits the OneShot125 MAX pulse (250 us) for 6 seconds.
//      Pilot reconnects the ESC battery during this window. The ESC
//      sees max-on-power-up and enters calibration mode.
//   3. Firmware switches to the IDLE pulse (125 us) for 4 seconds.
//      The ESC stores the range.
//   4. Firmware halts. Pilot sets ENABLE_ESC_CALIBRATION back to 0,
//      reflashes, and the ESC is calibrated for the configured range.
//
// Most BLHeli ESCs (including the Cyclone 45A in the v1 reference BOM)
// do not need this. The routine is here for ESCs that require manual
// range learning. Per CODING_STANDARDS.md §7.1 this is the only place
// in normal flight code besides the IMU bias calibration where an
// infinite loop is acceptable.
[[noreturn]] void run();

}  // namespace cp::actuators::esc_calibrate

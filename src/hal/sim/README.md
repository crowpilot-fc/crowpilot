<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Sim HAL implementations (placeholder)

This directory will host the SITL and HIL implementations of the
`cp::hal::*` API declared in `src/hal/Hal.h`. They are scaffolded but
empty as of the Phase 12.5 HAL boundary refactor.

The native-target implementations live in `src/hal/native/` and provide
the same API surface against real hardware (Wire for I2C, Servo for
PWM, PIO for SBUS receive, bit-banged OneShot125 for motor pulses).

Sim implementations land in Phase 12.7 per `internal-docs/DEVELOPMENT_PLAN.md`.
Files planned:

- `ImuSim.cpp`. Reads SCRC `SENSOR_FRAME` and converts SI units (rad/s,
  m/s²) to firmware-internal units (deg/s, g).
- `BaroSim.cpp`. Reads SCRC `SENSOR_FRAME` and pulls pressure (Pa) and
  temperature (°C).
- `RxSim.cpp`. Reads SCRC `SENSOR_FRAME`'s RC channel values plus the
  rc-loss flag (per `crowtalk/decisions/0004-rc-loss-flag-v2.md`).
- `OutSim.cpp`. Accumulates motor and servo widths into an
  `ACTUATOR_FRAME` sent back to Scarecrow each tick.

Protocol authority is `Scarecrow's docs/protocol.md`; the mirror in
`crowtalk/reference/PROTOCOL.md` is kept current by the Scarecrow side.
See `internal-docs/SITL_INTEGRATION.md` for the CrowPilot-side spec.

Build-target dispatch happens via `BUILD_TARGET` in `Config.h`. The
native `.cpp` files wrap their bodies in
`#if BUILD_TARGET == BUILD_TARGET_NATIVE`; the sim files (when they
land) will wrap in `#if BUILD_TARGET == BUILD_TARGET_HIL ||
BUILD_TARGET == BUILD_TARGET_SITL` so only one implementation links per
build.

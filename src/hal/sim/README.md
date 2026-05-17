<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Sim HAL implementations

The simulated implementations of the `cp::hal::*` API declared in
`src/hal/Hal.h` and `src/hal/Led.h`. They are selected when
`BUILD_TARGET` is `BUILD_TARGET_SITL` (or `BUILD_TARGET_HIL`); the
native-hardware implementations in `src/hal/native/` are selected for
`BUILD_TARGET_NATIVE`.

Each file wraps its body in
`#if BUILD_TARGET == BUILD_TARGET_HIL || BUILD_TARGET == BUILD_TARGET_SITL`,
so the normal Arduino build compiles them to nothing.

Files:

- `ImuSim.cpp`. Scripted IMU sample.
- `BaroSim.cpp`. Scripted barometer sample.
- `RxSim.cpp`. Scripted receiver channels.
- `OutSim.cpp`. Discards the actuator output.
- `LedSim.cpp`. No-op status LED; a fatal init failure exits the process.

The host build that compiles these into a runnable executable lives in
`sitl/`. See `sitl/README.md`.

## Current scope and the SCRC path

These files presently feed **scripted, static sensor data** so the
firmware can be compiled and run on a host. They do not yet model
airframe dynamics.

The originally planned design exchanged SCRC protocol frames over UDP
with the external Scarecrow simulator. That path depends on the SCRC
protocol spec and the Scarecrow runtime, which are not part of this
repository. The self-contained host build was implemented instead. A
physics model, and later the SCRC transport, can be added behind the
same HAL API without touching the firmware logic.

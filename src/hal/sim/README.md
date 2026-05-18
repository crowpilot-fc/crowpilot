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

- `ImuSim.cpp`. IMU sample read back from the rigid-body model.
- `BaroSim.cpp`. Fixed sea-level barometer sample.
- `RxSim.cpp`. Scripted receiver channels: arm and hold the sticks level.
- `OutSim.cpp`. Feeds the actuator commands to the model and steps it.
- `LedSim.cpp`. No-op status LED. A fatal init failure exits the process.
- `SimPhysics.h`. The rigid-body model interface.
- `SimPhysicsPlane.cpp`. Fixed-wing rotational model, used for the plane
  airframes.
- `SimPhysicsTailsitter.cpp`. Tailsitter rotational model.

The host build that compiles these into a runnable executable lives in
`sitl/`. See `sitl/README.md`.

## Current scope and the SCRC path

The sim HAL is closed-loop. `OutSim` feeds the controller's actuator
output into the rigid-body model, which integrates the aircraft's
rotational dynamics, and `ImuSim` reads the resulting attitude back, so
the firmware flies a simulated aircraft. The model is selected by
`AIRFRAME`: `SimPhysicsPlane` for the fixed-wing airframes,
`SimPhysicsTailsitter` for the tailsitter. Both model attitude dynamics
only, with plausible-estimate airframe constants. See `sitl/README.md`
for what the models do and do not validate.

The originally planned design exchanged SCRC protocol frames over UDP
with the external Scarecrow simulator. That path depends on the SCRC
protocol spec and the Scarecrow runtime, which are not part of this
repository. The self-contained host build was implemented instead. The
SCRC transport can later be added behind the same HAL API without
touching the firmware logic.

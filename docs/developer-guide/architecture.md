<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Architecture

CrowPilot is a single Arduino sketch for the RP2350. This page describes how
the firmware is structured and how state flows through it.

## The super-loop

The firmware is a 1 kHz cooperative super-loop. There is no RTOS and no
preemption. `crowpilot.ino` is a thin shim: `setup` calls `cp::core::init`
and `loop` calls `cp::core::tick`.

`init` brings every module up in dependency order. A missing mandatory sensor,
the IMU or the receiver, prints a clear message and halts with a fast LED
blink before any motor output is possible. The barometer is optional. The
one-shot ESC and IMU calibration routines run here when their compile-time
flags are set.

`tick` runs one iteration in a fixed order:

```
measure loop period
receiver poll
failsafe update
transition fader update
live-tune update
IMU read
barometer read
attitude update
desired-state update
stabilizer update
mixer update
actuator update
user hook
telemetry tick
debug prints
configurator poll
LED tick
regulate to the loop period
```

State flows forward through the chain. The only back-edge is the failsafe
override, which replaces the receiver channels before any consumer sees
them. The barometer read, the telemetry tick, and the user hook are
internally rate-limited and are cheap on the ticks they do not fire.

The tick measures its own period and busy-waits at the end until the next
tick is due. An overrun simply starts the next tick late and is recorded in
the measured period.

## Module layers

```
crowpilot.ino          entry shim
src/core/              the main loop
src/control/           desired-state, PID, plane stabilization
src/estimation/        attitude
src/modes/             transition fader and flight-mode enum
src/airframes/         per-airframe mixers
src/actuators/         output stage, arm logic, ESC calibration
src/sensors/           IMU and barometer facades, IMU calibration
src/radio/             receiver facade
src/failsafe/          lost-link override
src/telemetry/         SD card logger
src/params/            parameter registry and live tuning
src/storage/           flash persistence
src/user_hook/         user extension hook
src/cli/               configurator serial command interface
src/hal/               hardware abstraction layer
src/libs/              datasheet-derived chip and protocol drivers
src/boards/            per-board pin maps
```

Every module lives in a `cp::module_name` namespace. Cross-module state flows
through per-module accessor functions rather than shared globals.

## The HAL boundary

The hardware abstraction layer separates the firmware logic from per-target
input and output. `src/hal/Hal.h` is a transport-agnostic API. The native
implementation under `src/hal/native/` drives real hardware: the I2C bus, the
PIO state machine for SBUS, the OneShot125 emit, and the servo PWM. A simulated
implementation under `src/hal/sim/` backs the SITL build target, which compiles
the firmware logic as a host executable. That host build lives in `sitl/`. The
build target is a compile-time selector in `Config.h`.

The chip drivers in `src/libs/` are written from datasheets. The sensor
facades in `src/sensors/` wrap the HAL with bias subtraction and health flags.

## Compile-time configuration

`src/Config.h` is the single place a builder selects hardware and sets
tunable constants. It holds the preprocessor selectors that gate compilation,
the board, build target, airframe, sensor types, and receiver protocol, and
the typed tunable constants for every subsystem.

The airframe is a compile-time selector. Exactly one mixer compiles in. The
main loop branches on the airframe: the tailsitter runs the quaternion-error
PID and the bicopter mixer, a plane build runs the fixed-wing stabilizer and
the plane mixer. The rest of the stack is shared.

## Error handling

A missing mandatory sensor halts in `init`, before motor output is enabled.
A mid-flight sensor glitch is non-fatal: the affected reading is marked stale
and the flight continues, with the attitude estimate coasting on the previous
tick. Configuration errors are caught at compile time with `#error` guards.

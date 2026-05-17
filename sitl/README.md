<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# CrowPilot SITL host build

Builds the CrowPilot firmware as a native host executable, so the
control loop can run and be inspected on a development machine instead
of an RP2350.

The firmware logic is compiled unchanged from the Arduino build. Only
the platform layer differs: the real hardware HAL is swapped for the
simulated HAL in `src/hal/sim/`, and `<Arduino.h>` is swapped for the
host shim in `arduino_shim/`.

## Status

The SITL build is **closed-loop**. A rigid-body model (`SimPhysics`)
integrates the tailsitter's rotational dynamics from the controller's
actuator output and feeds the resulting motion back as IMU samples, so
the firmware flies a simulated aircraft.

The scripted scenario arms the aircraft at hover throttle with a 15
degree pitch tilt. The controller rights it: the tilt converges to level
within one to two seconds and holds there, with the body rates settling
to zero. Each simulated second the run prints a `SIM` line with the
model's ground-truth tilt and body rates.

What it does and does not show. The model demonstrates that the control
loop is structurally sound and stabilizes a plausible plant: the
estimator tracks, the PID responds without saturating, and the loop
converges. It does **not** validate the tuning against the real
aircraft. The inertia and actuator-effectiveness constants in
`SimPhysics` are plausible estimates for a 1 kg tailsitter, not measured
values, and the actuator sign conventions were set empirically to close
the loop. The model covers rotational dynamics only, not translation or
altitude.

## Building and running

Requires CMake and a C++17 host compiler.

```
cd sitl
cmake -B build -S .
cmake --build build
./build/crowpilot_sitl [tick_count]
```

`tick_count` defaults to 3000 (three seconds at the 1 kHz loop rate).
The firmware's boot banner and `DEV` debug lines print to stdout.

## Layout

```
sitl/
  CMakeLists.txt      Host build: source list, include paths, BUILD_TARGET
  main.cpp            Host entry point (replaces Arduino setup/loop)
  arduino_shim/
    Arduino.h         Host stand-in for the Arduino-Pico core
  SimStorage.cpp      Stand-in for the LittleFS parameter store
  SimTelemetry.cpp    Stand-in for the SD-card logger
src/hal/sim/
  ImuSim.cpp BaroSim.cpp RxSim.cpp OutSim.cpp LedSim.cpp
                      The simulated HAL, selected by BUILD_TARGET_SITL
  SimPhysics.cpp      Rigid-body model the sim HAL closes the loop through
```

The build is selected by `-DBUILD_TARGET=BUILD_TARGET_SITL`, which
`Config.h` and the HAL source guards honour. The normal Arduino build is
unaffected: it leaves `BUILD_TARGET` at the native default and never
compiles `sitl/` or activates the `src/hal/sim/` guards.

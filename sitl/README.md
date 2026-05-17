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

This is the **host-build checkpoint**. The firmware compiles and runs on
the host, the 1 kHz superloop ticks, every module initialises and runs,
and the attitude estimator converges. The simulated HAL feeds **scripted
sensor data**: a static aircraft in the nose-up hover attitude, sticks
disarmed and centred.

It is not yet a closed-loop simulation. A physics model that integrates
the airframe dynamics from the controller's actuator output, and feeds
the result back as sensor data, is the next step.

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
```

The build is selected by `-DBUILD_TARGET=BUILD_TARGET_SITL`, which
`Config.h` and the HAL source guards honour. The normal Arduino build is
unaffected: it leaves `BUILD_TARGET` at the native default and never
compiles `sitl/` or activates the `src/hal/sim/` guards.

<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Adding an airframe

An airframe in CrowPilot is a mixer: the function that turns the controller's roll, pitch, yaw, and throttle demands into motor and servo pulse widths. Adding a new airframe is a self-contained module plus a selector.

## The airframe interface

The airframe contract is in `src/airframes/Airframe.h`. The rest of the firmware calls `cp::airframes::update(...)` once per tick and does not know or care which airframe is compiled in. A new airframe is a new `.cpp` that implements the mixer for one frame type, selected at compile time.

## The mixer function

The mixer takes:

- The controller output: normalized roll, pitch, yaw demands in `[-1, +1]`.
- Throttle: the pilot's throttle demand in `[0, 1]`.
- The transition fader, if the airframe has a hover and a forward regime.

It produces:

- A normalized thrust command in `[0, 1]` for each motor.
- A normalized deflection command in `[0, 1]` for each control surface.

The HAL converts those normalized commands into OneShot125 motor pulses and standard servo PWM. The mixer never touches microseconds.

The mixer must be pure arithmetic. No I/O, no blocking, no dynamic allocation. It runs every tick in the hot path. Clamp every output to its valid range before returning.

## Worked example: the tailsitter mixer

`src/airframes/Airframe.cpp` is the reference tailsitter mixer. It is not two separate blocks. The effector-to-moment map is fixed by geometry and is the same in both regimes: common motor thrust is body-x force, differential thrust is yaw, symmetric elevon is pitch, differential elevon is roll. The mixer computes one allocation. The only term that changes with regime is a direct-stick feedforward added to the elevons in forward flight, blended out toward hover by the fader. The full math is in [algorithms.md](algorithms.md). Read it before writing a new mixer; it shows the expected structure.

## Adding the build target

1. Write `src/airframes/<Airframe>.cpp` implementing the mixer.
2. Add an `AIRFRAME_*` selector macro to `src/Config.h`.
3. Add the dispatch (the `#if` block that compiles your mixer when its selector is active).
4. Add any airframe-specific tunables (mix coefficients, servo trims) to `src/Config.h`.
5. Document the airframe under `docs/airframes/`.

## Testing a new airframe

A new mixer must be bench-verified before flight:

1. Run the [first bench test](../getting-started/first-bench-test.md) through Stage 7 (the bench mixer check). Confirm every stick deflection produces the correct motor differential or servo deflection, in both hover and forward regimes if the airframe has both.
2. Verify the mixer outputs stay within range under full stick deflection plus full throttle. A mixer that saturates under combined demand needs its coefficients rebalanced.
3. Only then proceed to motors-connected and tethered testing.

Do not fly a new airframe until the bench mixer check passes cleanly.

<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Transmitter setup

CrowPilot reads six channels over SBUS. This page covers how to map them on your transmitter. Verify every channel on the bench per [first-bench-test.md](../getting-started/first-bench-test.md) Stage 3 before flying.

## Channel mapping

Map your transmitter to these six channels, in this order:

| Channel | Function | Control | Range |
|---|---|---|---|
| ch1 | Throttle | Throttle stick | 1000 (min) to 2000 (max) us |
| ch2 | Roll | Aileron stick | 1000 to 2000 us, centered 1500 |
| ch3 | Pitch | Elevator stick | 1000 to 2000 us, centered 1500 |
| ch4 | Yaw | Rudder stick | 1000 to 2000 us, centered 1500 |
| ch5 | Arm / cut | Two-position switch | 1000 (LOW, armed) / 2000 (HIGH, disarmed) |
| ch6 | Transition | Two- or three-position switch or slider | 1100 (forward) to 1900 (hover) |

The arm/cut switch on ch5 is mandatory. The transition switch on ch6 is only meaningful for VTOL airframes; on a pure plane it can be left at the hover end and ignored.

## Throws, deadband, expo

- **Endpoints.** Set channel endpoints so the stick travels the full 1000 to 2000 us range. A stick that only reaches 1100 to 1900 costs you control authority.
- **Centering.** Roll, pitch, and yaw must read 1500 us, plus or minus a few, with the sticks centered. Trim them on the transmitter.
- **Deadband.** A small deadband on roll, pitch, and yaw is fine and reduces hand-jitter. The FC does not add its own deadband.
- **Expo.** Expo is a pilot preference. Start with none, add it later if the aircraft feels twitchy around center. Expo is applied on the transmitter, not in `Config.h`.

## Switch assignments

- **ch5 (arm/cut)** goes to a two-position switch. Pick one with a physical guard if your transmitter has one, or at least a switch you will not knock by accident. HIGH must mean disarmed.
- **ch6 (transition)** goes to a two-position switch, a three-position switch, or a slider. A three-position switch gives you hover, mid-transition, and forward as discrete detents. A slider gives you continuous control. Either works; the FC slews smoothly regardless.

## Channel direction

If a stick or switch reads inverted on the bench (throttle up reads 1000 instead of 2000, ch5 HIGH reads armed instead of disarmed), fix it on the transmitter by reversing the channel. Do not invert it in `Config.h`. Keeping all inversion on the transmitter side keeps one source of truth.

## Verifying the setup

Set `DEBUG_PRINT_RX = 1` in `Config.h`, reflash, and open the serial monitor. Move each stick and switch in turn and confirm the channel values move as the table above describes. Full procedure in [first-bench-test.md](../getting-started/first-bench-test.md) Stage 3.

## Transmitter-specific notes

CrowPilot does not care which transmitter you use, only that it outputs SBUS (directly, or via an SBUS-capable receiver). RadioMaster, Jumper, FrSky, and Flysky transmitters paired with an SBUS or ELRS receiver all work. For ELRS receivers that output non-inverted SBUS, set `RX_SBUS_INVERTED = 0` in `Config.h`.

Transmitter-by-transmitter walkthroughs are a good contribution target. If you set up a specific model, consider opening a pull request adding it here.

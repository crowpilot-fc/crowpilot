<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# CrowPilot

An open-source flight controller firmware for the Raspberry Pi RP2350. It supports a 3D-printed tailsitter VTOL bicopter and fixed-wing planes, with the DHC-4 Caribou twin-engine plane as the current first-flight bring-up airframe. It is built to grow into GPS-aware flight.

## Status

**Pre-alpha.** CrowPilot is under active single-developer development and has not completed acceptance flight testing. Do not fly an aircraft running CrowPilot without reading [Safety](safety.md) and [Disclaimer](disclaimer.md) in full. Propellers can cause serious injury or death.

## Get started

New here? Work through the getting-started path in order:

1. [What is CrowPilot](getting-started/what-is-crowpilot.md) - the 30-second pitch and who it is for.
2. [Hardware BOM](getting-started/hardware-bom.md) - the shopping list.
3. [Wiring](getting-started/wiring.md) - how to connect everything.
4. [Flashing](getting-started/flashing.md) - getting the firmware onto the board.
5. [First Bench Test](getting-started/first-bench-test.md) - the smoke test before propellers go on.

## What is inside

- **Getting Started** takes you from an empty bench to a flashed, bench-verified flight controller.
- **User Guide** covers arming, failsafe, transmitter setup, flight modes, tuning, calibration, and reading telemetry logs.
- **Airframes** documents each supported airframe and how to build it.
- **Developer Guide** explains the architecture, the algorithms, and how to add a board or an airframe.
- **Reference** is the lookup material: every config option, every pin map, the telemetry format.

## License

CrowPilot firmware is licensed GPL-3.0-or-later. Documentation is licensed CC-BY-SA-4.0. See the [LICENSE](https://github.com/crowpilot-fc/crowpilot/blob/main/LICENSE) file in the repository for the full text.

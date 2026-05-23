<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# What is CrowPilot

## The 30-second pitch

CrowPilot is a flight controller firmware that runs on the Raspberry Pi RP2350 microcontroller. It is a single Arduino sketch: clone the repository, open it in the Arduino IDE, flash it to a supported RP2350 board, and you have a working flight controller.

It is a clean-build project. Every driver and every algorithm is written from datasheets and published references, not lifted from another flight-controller codebase. The firmware is GPL-3.0-or-later, the documentation is CC-BY-SA-4.0, and the whole thing is built to be read, understood, and modified by one hobbyist with a soldering iron.

## What it flies

CrowPilot is pre-alpha and has not flown yet. The firmware supports two airframe families and ships configured for the bring-up target.

- **Fixed-wing plane** (the current target): a conventional plane with ailerons, elevator, rudder, throttle, and a wing leveler. The first-flight aircraft is the DHC-4 Caribou twin-engine cargo plane, and the default build is configured for it.
- **Tailsitter VTOL bicopter**: two motors, two elevon servos, a continuous hover-to-forward transition controlled by a transmitter switch. This is the carried alternate airframe.

The flight controller itself is not tied to one airframe. It can be installed on any inexpensive foam-based plane or bicopter. The airframe, motors, and propulsion are yours to choose, CrowPilot only cares about the flight electronics.

## What is coming

- **v1.2** adds a single-engine plane airframe.
- **v2** adds GPS: position hold, return-to-home, and waypoint following.

See the [Roadmap](../roadmap.md) for the full picture.

## Who this is for

CrowPilot is for the hobbyist who owns a soldering iron and a 3D printer, is comfortable reading a wiring table, and wants a flight controller they can fully understand and modify. If you have flashed an ESP32, wired an I2C sensor, or built an RC aircraft before, you have the background.

## Who this is not for

CrowPilot is not certified avionics. It is not for any aircraft carrying people, for commercial operations that require certified equipment, or for anyone who needs a guarantee that the firmware will not fail. It is a hobbyist project under active development. Read the [Disclaimer](../disclaimer.md).

## Cost in parts

The flight electronics (the RP2350 board, IMU, barometer, SD card module, and supporting parts) come to roughly USD 20. A full per-item breakdown is in the [Hardware BOM](hardware-bom.md). Airframe, motors, ESCs, battery, and transmitter are separate and depend entirely on what you choose to fly.

## What is next

Read the [Hardware BOM](hardware-bom.md), gather the parts, then move on to [Wiring](wiring.md).

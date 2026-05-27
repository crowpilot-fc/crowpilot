<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Roadmap

CrowPilot ships in versioned releases. Each release has a defined acceptance airframe and a published bench-verification queue. v1.0 is the DHC-4 Caribou first flight. v1.1 brings up the single-engine Gee Bee. v2 adds GPS-aware features.

This page tracks scope, not dates. Items in **complete** have landed on main. Items in **in progress** are actively under work. Items in **next** are queued for the current release. Items in **later** are scoped but unscheduled.

The tailsitter VTOL bicopter, CrowPilot's origin airframe, stays fully supported as a carried airframe and shares the same control core as the planes. It is no longer the first-flight target.

## v1.0: DHC-4 Caribou cargo plane (first flight)

The initial release. The first-flight aircraft is the DHC-4 Caribou twin-engine cargo plane, based on the open-source Caribou STL set, and the firmware ships configured for it.

**Complete.**

- Repo scaffolding, GPL-3.0-or-later code license, CC-BY-SA-4.0 docs license.
- 1 kHz cooperative superloop with regulator and busy-wait period pinning.
- MPU-6500 and MPU-6050 IMU drivers (datasheet-derived, clean-build) with an IMU facade and gyro/accel bias calibration.
- BMP388 and BMP280 barometer drivers with relative altitude. The barometer is optional.
- Madgwick 6-DOF AHRS.
- Gyro filtering in the estimator: a per-axis low-pass and a notch, with an optional RPM-tracking dynamic notch fed by bidirectional-DShot eRPM, plus a D-term low-pass on the PID derivative path.
- SBUS receiver via PIO inverted UART (no external inverter) and a CRSF receiver over UART (Crossfire / ELRS), selected by RX_PROTOCOL.
- Failsafe controlled-fall override on link loss.
- Desired-state generation and angle PID controller (D on measurement, anti-windup by saturation, gain blend).
- Fixed-wing stabilization subsystem: wing leveler, pitch hold, and yaw damper.
- Twin-engine cargo plane mixer (Caribou), single-engine plane mixer, the tailsitter bicopter mixer with its transition fader and flight-mode enum, and the Quad X mixer with self-leveling angle and acro rate modes.
- PWM, OneShot125, and DShot300/600 ESC output, with optional bidirectional DShot eRPM telemetry, 50 Hz servo PWM, arm logic, and ESC calibration.
- User extension hook (pin-claim, read-only sensor API, three-strike disable, per-tick time budget) and the Caribou aux user sketch.
- SD card binary telemetry logger plus host-side decoder and log analyzer. Logging is optional.
- Runtime parameter registry, transmitter live tuning, and LittleFS persistence. All optional.
- Board profiles for the WeAct RP2350A_V10 (default), the Waveshare RP2350-Tiny, and the Raspberry Pi Pico 2 W.
- A minimal IMU-only build profile (no SD card, barometer, or WiFi) for a simple plane on a small board.
- The CrowPilot Configurator (parameter editor, live telemetry, log viewer, firmware flashing) and the `cp` serial command interface it speaks.
- The ESP32-C3 wireless companion and the Pico 2 W onboard-WiFi companion, both serving a phone web UI with a cockpit instrument panel over the `cp` protocol.
- HAL boundary with a native RP2350 implementation and a closed-loop SITL implementation carrying plane and tailsitter rigid-body models.
- Public documentation set (`docs/` tree) and a monochrome brand identity with the Crow Teal accent and a vectorized logo.

**Next.**

- Reference user-sketch examples (e.g. a payload-drop hook).
- Caribou bench verification, per [docs/getting-started/caribou-bench-test.md](getting-started/caribou-bench-test.md).
- Caribou first flight (v1.0 acceptance).

**Later.**

- Tailsitter tethered hover, the carried airframe's acceptance, deprioritized behind the Caribou maiden.

Control-core refinements, scoped for after bench verification and added one
at a time against a proven baseline. Each is standard practice, derived from
public control-theory and signal-processing references. The gyro low-pass,
the gyro notch (static and RPM-tracking dynamic), and the D-term low-pass
have already landed and moved to the complete list above.

- Back-calculation, or tracking, anti-windup in place of the present
  integral-term clamp, for better behavior at the saturation limit.
  Reference: Astrom and Hagglund, PID Controllers (1995).
- Setpoint weighting on the proportional term, to trade setpoint
  tracking against disturbance rejection independently. Reference:
  Astrom and Hagglund, PID Controllers (1995).

## v1.1: Gee Bee single-engine plane

A short, fat, racing-style single-engine plane based on the open-source Gee Bee STL set. The single-engine plane mixer ships in v1.0. v1.1 brings the airframe up.

**Later.**

- EDF-specific tuning notes, an option for the same airframe's power plant.
- Gee Bee bench verification.
- Gee Bee first flight (v1.1 acceptance).

## v2: GPS-aware

**Later.**

- GPS receiver integration on the companion UART.
- Position hold and return-to-home.
- Waypoint following.
- On-board classical auto-tune (across all v1 airframes).
- Magnetometer fusion to bound yaw drift.

## What this roadmap is not

- It is not a calendar. CrowPilot has one developer and ships when the work is done, not when a date is.
- It is not a contract. Features can be added, dropped, or resequenced if the bench data says so.
- It is not the full plan. This page is the public summary of scope, not a granular development schedule.

For the v1 disclaimer and the legal contour of "ship when ready," see [DISCLAIMER.md](../DISCLAIMER.md).

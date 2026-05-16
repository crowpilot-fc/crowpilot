<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Roadmap

CrowPilot ships in versioned releases. Each release has a defined acceptance airframe and a published bench-verification queue. v1.0 is the tailsitter bicopter target; v1.1 and v1.2 broaden the airframe support; v2 adds GPS-aware features.

This page tracks scope, not dates. Items in **complete** have landed on main. Items in **in progress** are actively under work. Items in **next** are queued for the current release. Items in **later** are scoped but unscheduled.

## v1.0: Tailsitter bicopter (Eclipson E-VTOL-1 reference)

The first acceptance airframe is a 3D-printed tailsitter bicopter sized for two small brushless motors and two elevon servos.

**Complete.**

- Repo scaffolding, GPL-3.0-or-later code license, CC-BY-SA-4.0 docs license.
- 1 kHz cooperative superloop with regulator and busy-wait period pinning.
- MPU-6500 and MPU-6050 IMU drivers (datasheet-derived, clean-build) with an IMU facade and gyro/accel bias calibration.
- Madgwick 6-DOF AHRS with hover and forward Euler conventions.
- SBUS receiver via PIO inverted UART. No external inverter.
- Failsafe controlled-fall override on link loss.
- Transition fader and FlightMode enum.
- BMP388 and BMP280 barometer drivers with relative altitude.
- Desired-state generation and angle PID controller (D on measurement, anti-windup by saturation, hover and forward gain blend).
- Tailsitter bicopter mixer.
- OneShot125 ESC bit-bang, 50 Hz servo PWM, arm logic, throttle cut, ESC calibration.
- SD card binary telemetry logger plus host-side CSV decoder (`tools/decode_log.py`).
- HAL boundary (native and sim placeholders for SITL/HIL).
- Public documentation pass (`docs/` tree).

**In progress.**

- Runtime parameter system with LittleFS persistence. Two TX channels scale Kp and Kd by ±50 percent in real time without reflashing.
- Log analyzer tool (`tools/log_analyzer/`).

**Next.**

- Bench verification (one-shot, all phases) on the reference build per [docs/getting-started/first-bench-test.md](getting-started/first-bench-test.md).
- SITL hover validation, then HIL hover validation. Both depend on the SCRC protocol being finalized in the cross-project coordination repo.
- Tethered hover (v1.0 acceptance flight).

**Later.**

Control-core refinements, scoped for after v1.0 bench verification and
added one at a time against a proven baseline. Each is standard
practice, derived from public control-theory and signal-processing
references.

- A low-pass or biquad filter on the PID derivative path, so gyro noise
  is not amplified into the actuators. Reference: Astrom and Murray,
  Feedback Systems.
- Back-calculation, or tracking, anti-windup in place of the present
  integral-term clamp, for better behavior at the saturation limit.
  Reference: Astrom and Hagglund, PID Controllers (1995).
- Setpoint weighting on the proportional term, to trade setpoint
  tracking against disturbance rejection independently. Reference:
  Astrom and Hagglund, PID Controllers (1995).
- A gyro notch filter at the airframe's dominant propeller and motor
  vibration frequency, identified from flight-log FFT analysis.
  Reference: Bristow-Johnson, Audio EQ Cookbook, or Oppenheim and
  Schafer, Discrete-Time Signal Processing.

## v1.1: Caribou cargo plane

A two-engine cargo plane based on the open-source Caribou STL set.

**Next.**

- Plane stabilization subsystem (replaces the tailsitter PID for plane airframes).
- Twin-engine cargo plane mixer.
- User extension hook with pin-claim, sensor read-only API, three-strike disable, and per-tick time budget.
- Reference user-sketch examples (e.g. a payload-drop hook).
- Caribou bench verification.
- ESP companion firmware (Wi-Fi telemetry, simple phone UI).
- Phone UI (browser-based, served by the ESP).
- Caribou first flight (v1.1 acceptance).

## v1.2: Gee Bee single-engine plane

A short, fat, racing-style single-engine plane based on the open-source Gee Bee STL set.

**Later.**

- Single-engine plane mixer.
- EDF-specific tuning notes (an option for power plants for the same airframe).
- Gee Bee bench verification.
- Gee Bee first flight (v1.2 acceptance).

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
- It is not the full plan. Internal `internal-docs/DEVELOPMENT_PLAN.md` is the granular phase plan; this page is the public summary.

For the v1 disclaimer and the legal contour of "ship when ready," see [DISCLAIMER.md](../DISCLAIMER.md). For the cross-project coordination with the SITL host runtime (Scarecrow), see the crowtalk coordination repo.

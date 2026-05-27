<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Changelog

All notable changes to CrowPilot are recorded here. The format follows
Keep a Changelog. Versions follow semantic versioning once a first release
is cut.

## Unreleased

Work toward **v1.0.0**, the initial release. v1.0.0 targets the DHC-4 Caribou
twin-engine plane for its first flight, and the firmware ships configured for
it. The tailsitter VTOL bicopter is supported as a carried airframe. Pre-alpha:
the firmware has not completed acceptance flight testing and every
control-core tuning constant is provisional.

### Added

- The 1 kHz cooperative super-loop and the Arduino entry shim.
- Hardware abstraction layer with a native implementation for the RP2350.
- Datasheet-derived drivers: MPU-6500 and MPU-6050 IMUs, BMP388 and BMP280
  barometers, the SBUS frame decoder with a PIO program for the inverted
  UART, and a CRSF (Crossfire / ELRS) frame decoder over UART.
- IMU and barometer sensor facades and the SBUS and CRSF receiver paths,
  selected by RX_PROTOCOL.
- Lost-link failsafe that holds a level, powered descent.
- Madgwick six-degree-of-freedom attitude estimation, with a quaternion
  attitude error that stays continuous across the hover-to-forward
  transition.
- IMU gyro and accelerometer bias calibration.
- The transition fader and the flight-mode enumeration.
- Pilot desired-state generation.
- The per-axis PID stabilizer with regime gain scheduling and anti-windup.
- The tailsitter bicopter mixer.
- The actuator output stage with arm and disarm safety logic, driving PWM
  (default), OneShot125, or DShot300/600 ESCs and 50 Hz servos. DShot
  clocks a 16-bit CRC-checked frame to each motor from a PIO state machine.
- SD card binary telemetry and a host-side log decoder and analyzer.
- The runtime parameter registry, transmitter live tuning, and flash
  persistence.
- The fixed-wing stabilization subsystem and the twin-engine and
  single-engine plane mixers.
- The user extension hook.
- The CrowPilot Configurator, a browser-based setup tool that edits the
  parameter registry, shows live telemetry, decodes binary flight logs,
  and flashes firmware over WebUSB.
- The cp serial command interface that the configurator speaks, with a
  live telemetry stream and a reboot-to-bootloader command.
- The SITL build target: the firmware logic compiled as a host
  executable against a simulated HAL, with a closed-loop plane and
  tailsitter rigid-body model so the control loop flies a simulated aircraft.
- Board profiles for the WeAct RP2350A_V10 (default), the Waveshare
  RP2350-Tiny, and the Raspberry Pi Pico 2 W.
- A minimal IMU-only build profile (no SD card, barometer, or WiFi) for a
  simple fixed-wing plane on a small board.
- The DHC-4 Caribou as the first-flight airframe, with its aux user sketch
  (retracts, flaps, bay doors, navigation lights). The nose wheel is steered
  mechanically off the rudder.
- The ESP32-C3 wireless companion: a WiFi access point that serves a phone
  web UI and bridges the cp protocol to the flight controller's companion UART.
- Onboard WiFi for the Raspberry Pi Pico 2 W: the flight controller raises
  its own access point and serves the same UI and cp bridge from its second
  core, with no separate companion board.
- A cockpit instrument panel in the phone UI: artificial horizon, altimeter,
  vertical-speed indicator, turn-and-slip, and G-load, fed by an extended cp
  telemetry line.
- A brand identity: a monochrome palette with the Crow Teal accent and a
  vectorized logo, applied to the configurator and the companion UI.
- The documentation set under docs/.

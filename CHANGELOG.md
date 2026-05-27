<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Changelog

All notable changes to CrowPilot are recorded here. The format follows
Keep a Changelog. Versions follow semantic versioning once a first release
is cut.

## Unreleased

CrowPilot v1.0, the initial implementation. Pre-alpha: the firmware has not
completed acceptance flight testing and every control-core tuning constant
is provisional.

### Added

- The 1 kHz cooperative super-loop and the Arduino entry shim.
- Hardware abstraction layer with a native implementation for the RP2350.
- Datasheet-derived drivers: MPU-6500 and MPU-6050 IMUs, BMP388 and BMP280
  barometers, and the SBUS frame decoder with a PIO program for the inverted
  UART.
- IMU and barometer sensor facades and the SBUS receiver facade.
- Lost-link failsafe that holds a level, powered descent.
- Madgwick six-degree-of-freedom attitude estimation, with a quaternion
  attitude error that stays continuous across the hover-to-forward
  transition.
- IMU gyro and accelerometer bias calibration.
- The transition fader and the flight-mode enumeration.
- Pilot desired-state generation.
- The per-axis PID stabilizer with regime gain scheduling and anti-windup.
- The tailsitter bicopter mixer.
- The actuator output stage with arm and disarm safety logic, the
  synchronous OneShot125 emit, and a PIO-clocked DShot300/600 path.
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
  executable against a simulated HAL.
- The documentation set under docs/.

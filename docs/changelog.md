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
- Gyro filtering in the estimator: a per-axis low-pass and a notch, with an
  optional dynamic notch that tracks the motor frequency from bidirectional
  DShot eRPM.
- IMU gyro and accelerometer bias calibration.
- The transition fader and the flight-mode enumeration.
- Pilot desired-state generation.
- The per-axis PID stabilizer with regime gain scheduling and anti-windup.
- The tailsitter bicopter mixer.
- The actuator output stage with arm and disarm safety logic, driving PWM
  (default), OneShot125, or DShot300/600 ESCs and 50 Hz servos. DShot
  clocks a 16-bit CRC-checked frame to each motor from a PIO state machine,
  with an optional bidirectional mode (bench-tuned) that reads ESC eRPM back.
- SD card binary telemetry and a host-side log decoder and analyzer.
- The runtime parameter registry, transmitter live tuning, and flash
  persistence.
- The fixed-wing stabilization subsystem and the twin-engine and
  single-engine plane mixers.
- Plane flight modes on the ch14 switch: manual passthrough, rate
  (gyro-damped), angle (self-level), and horizon (angle near center stick,
  rate at full stick), with the three switch positions mapped in Config.h.
- Optional coordinated-turn assist for the angle and horizon plane modes:
  auto-rudder proportional to bank and a bank-compensated up-elevator to hold
  altitude through a turn. Off by default.
- Optional hand-launch assist for planes: a forward-acceleration throw detector
  that commands a wings-level climb-out (launch throttle and a nose-up hold) for
  a short window, gyro and accelerometer only. Off by default.
- The plane stabilizer gains are now runtime parameters, so the configurator
  and phone sliders set them and the transmitter live-tune knobs trim them in
  flight, the same way they already do the tailsitter PID. The knob is a
  multiplier on the slider-or-default base value.
- A built-in LED flasher for aircraft lighting: a switched on/off output on one
  configurable GPIO, with strobe, beacon, and steady patterns, driven from the
  main loop with no user hook. It drives a small 2-pin LED directly, a 1 W LED
  through a low-side MOSFET, or a 3-pin signal-plus-power module. Off by default
  so the pin stays free.
- Optional battery voltage monitoring: a divided pack voltage on an ADC pin,
  with cell-count auto-detection, a per-cell low-voltage warning surfaced in
  telemetry and the configurator, and a pre-arm minimum. Off by default. Low
  voltage is a warning in v1, not an automatic flight action.
- Pre-arm checks: arming is refused unless the IMU is healthy, the receiver
  link is up with valid channels, and (when monitored) the pack is above the
  arm threshold, on top of the existing throttle-idle and arm-cycle gates.
  Disarming is never gated. The blocking reason is shown in the DEV line and
  the configurator (IMU, RX, or battery).
- Optional per-servo output config: reverse, subtrim, and travel endpoints per
  surface in the output stage, so servo direction and trim are set in the
  firmware instead of mechanically. Off by default. Pilot expo stays on the
  transmitter.
- The Quad X airframe and mixer, with a self-leveling angle mode and an
  acro rate mode (a per-axis rate controller) selected by the stabilizer
  switch, validated against a SITL quad rigid-body model.
- The flying-wing (delta) airframe and elevon mixer: one motor, two elevons,
  no rudder, driven by the fixed-wing stabilizer. Reuses the tailsitter's
  forward-flight elevon allocation and is validated in SITL.
- The V-tail airframe and mixer: one motor, two ailerons, and two ruddervators
  that carry pitch as their common mode and yaw as their differential, driven
  by the fixed-wing stabilizer and validated in SITL.
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

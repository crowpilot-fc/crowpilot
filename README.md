<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# CrowPilot

CrowPilot is an open-source flight controller firmware for the Raspberry Pi
RP2350 microcontroller. It is a single Arduino sketch. The reference aircraft
is a 3D-printed tailsitter VTOL bicopter. The firmware also targets fixed-wing
planes.

## Status

Pre-alpha. CrowPilot has not completed acceptance flight testing. Every gain,
limit, and trim in the control core ships as a provisional value to be
replaced by bench tuning. Read [SAFETY.md](SAFETY.md) and
[DISCLAIMER.md](DISCLAIMER.md) in full before powering an aircraft. Propellers
can cause serious injury.

## Hardware

- MCU: RP2350, dual Cortex-M33 at 150 MHz with a hardware FPU. The RP2040 is
  not supported.
- Reference board: Waveshare RP2350-Tiny. Alternate: WeAct RP2350A_V10.
- Sensors: MPU-6500 or MPU-6050 IMU, optional BMP388 or BMP280 barometer, all
  on I2C0. SBUS receiver decoded by an RP2350 PIO state machine.
- Actuators: OneShot125 ESCs and standard servo PWM. SD card telemetry.

## Building

Build with the Arduino IDE 2.x and the Raspberry Pi Pico Arduino core by
Earle F. Philhower. Open `crowpilot.ino`, select an RP2350 board, and flash.
The only libraries used are those that ship with that core plus the standard
Arduino libraries. All chip drivers are written from datasheets.

Hardware selection and every tunable constant live in
[`src/Config.h`](src/Config.h). See
[docs/reference/config-options.md](docs/reference/config-options.md).

## Repository layout

```
crowpilot.ino     Arduino entry shim
sbus_rx.pio       PIO program for the inverted SBUS UART
src/
  Config.h        Compile-time selectors and tunables
  core/           The 1 kHz main loop
  hal/            Hardware abstraction layer
  libs/           Datasheet-derived chip and protocol drivers
  sensors/        IMU and barometer facades, IMU calibration
  estimation/     Attitude estimation
  radio/          Receiver facade
  failsafe/       Lost-link override
  modes/          Transition fader and flight-mode enum
  control/        Desired-state, PID, plane stabilization
  airframes/      Per-airframe mixers
  actuators/      Output stage, arm logic, ESC calibration
  params/         Runtime parameter registry and live tuning
  storage/        Flash persistence
  telemetry/      SD card binary logger
  user_hook/      User extension hook
  cli/            Configurator serial command interface
docs/             Documentation
tools/            Host-side log decoder and analyzer
configurator/     Browser-based setup tool (Web Serial)
```

## Airframes

The airframe is a compile-time selector. The mixer is the per-airframe piece.
The rest of the stack is shared.

- Tailsitter bicopter. Two motors, two elevons. Hovers nose-up and
  transitions to forward flight. This is the v1.0 reference airframe.
- Twin-engine cargo plane and single-engine plane, driven by the fixed-wing
  stabilization subsystem.

## Documentation

The documentation lives in `docs/`. Start at
[docs/index.md](docs/index.md). The developer guide covers the
[architecture](docs/developer-guide/architecture.md) and the
[algorithms](docs/developer-guide/algorithms.md).

## License

Firmware is licensed GPL-3.0-or-later. Documentation is licensed
CC-BY-SA-4.0. See [LICENSE](LICENSE) and [LICENSE-docs](LICENSE-docs).

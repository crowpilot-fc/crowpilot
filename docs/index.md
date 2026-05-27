<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# CrowPilot

CrowPilot is an open-source flight controller firmware for the Raspberry Pi RP2350. It is a single Arduino sketch, written clean from datasheets, running a 1 kHz control loop. It flies a 3D-printed tailsitter VTOL bicopter and fixed-wing planes, with the DHC-4 Caribou twin-engine plane as the current first-flight bring-up airframe, and it scales from a minimal IMU-only build up to a full build with SD logging and a WiFi companion. The firmware is GPL-3.0-or-later and the documentation is CC-BY-SA-4.0.

## Status

**Pre-alpha.** CrowPilot is under active single-developer development and has not completed acceptance flight testing. Do not fly an aircraft running CrowPilot without reading [Safety](safety.md) and [Disclaimer](disclaimer.md) in full. Propellers can cause serious injury or death.

## At a glance

| | |
|---|---|
| Microcontroller | Raspberry Pi RP2350 (dual Cortex-M33) |
| Control loop | 1 kHz cooperative super-loop |
| Attitude estimate | Madgwick 6-DOF AHRS (gyro and accelerometer) |
| Receiver | SBUS, decoded in PIO, no external inverter |
| Motors | PWM, OneShot125, or DShot300/600 ESCs |
| Configuration | one file, `src/Config.h` |
| Footprint | well under 10 percent of flash on a 2 MB board |
| Code license | GPL-3.0-or-later |

## Supported boards

All boards are RP2350-based. Select one with `BOARD` in `src/Config.h`. Full per-board pinouts are in [Pin maps](reference/pin-maps.md), and adding a new board is a single header, see [Adding a board](developer-guide/adding-a-board.md).

| Board | `BOARD` value | Notes |
|---|---|---|
| WeAct RP2350A_V10 | `BOARD_WEACT_RP2350A_V10` | Default. Pi-Pico form factor. |
| Waveshare RP2350-Tiny | `BOARD_WAVESHARE_RP2350_TINY` | Smallest footprint. |
| Raspberry Pi Pico 2 W | `BOARD_PICO2W` | Onboard WiFi radio, serves the companion UI itself, no separate ESP. |

## Supported airframes

Select one with `AIRFRAME` in `src/Config.h`. The plane airframes require `ENABLE_PLANE_STAB = 1`. Adding an airframe is documented in [Adding an airframe](developer-guide/adding-an-airframe.md).

| Airframe | `AIRFRAME` value | Effectors |
|---|---|---|
| Twin-engine cargo plane (DHC-4 Caribou) | `AIRFRAME_PLANE_TWIN_CARGO` | 2 motors, two ailerons, elevator, rudder. Default and first-flight target. See [Fixed-wing plane](airframes/fixed-wing-plane.md). |
| Single-engine plane | `AIRFRAME_PLANE_SINGLE` | 1 motor, two ailerons, elevator, rudder. |
| Tailsitter VTOL bicopter | `AIRFRAME_TAILSITTER_BICOPTER` | 2 motors, 2 elevons, continuous hover-to-forward transition. See [Tailsitter bicopter](airframes/tailsitter-bicopter.md). |

Multirotors (quadcopter, hexacopter, tricopter, tailsitter-quad) are reserved selectors that halt the build. A multirotor mixer with its rate and angle control modes is a v2.x item on the [Roadmap](roadmap.md).

## Sensors and radio

### IMU (required)

A 3-axis gyro and accelerometer on I2C0 (GP4 SDA, GP5 SCL). Set `IMU_TYPE`. Buy a genuine chip: the driver rejects clones by their `WHO_AM_I` value, see the [Hardware BOM](getting-started/hardware-bom.md).

| Sensor | `IMU_TYPE` | `WHO_AM_I` |
|---|---|---|
| InvenSense MPU-6500 | `IMU_MPU6500` | `0x70` (default) |
| InvenSense MPU-6050 | `IMU_MPU6050` | `0x68` |

### Barometer (optional)

A pressure sensor on the same I2C0 bus, for altitude hold and logging. Set `BARO_TYPE` to `BARO_NONE` to leave it out of a minimal build.

| Sensor | `BARO_TYPE` |
|---|---|
| Bosch BMP388 | `BARO_BMP388` (default) |
| Bosch BMP280 | `BARO_BMP280` |
| None | `BARO_NONE` |

### Receiver

SBUS or CRSF, selected by `RX_PROTOCOL`. SBUS is decoded by an RP2350 PIO state machine with no external inverter (`RX_SBUS_INVERTED` matches the receiver polarity). CRSF (Crossfire and ELRS) is a 420 kbaud UART on the same receiver pin, no inversion. `RX_PPM` and `RX_PWM` are reserved and halt the build. See [Transmitter setup](user-guide/transmitter-setup.md).

## Motors, servos, and ESCs

The mixer drives ESCs on the motor pins and standard servos on the surface pins. Set `MOTOR_PROTOCOL`.

| Protocol | `MOTOR_PROTOCOL` | Signal |
|---|---|---|
| Hobby PWM | `MOTOR_PROTOCOL_PWM` | 1000 to 2000 us, disarm at 1000 us. Default. |
| OneShot125 | `MOTOR_PROTOCOL_ONESHOT125` | 125 to 250 us, disarm below 125 us. |
| DShot300 | `MOTOR_PROTOCOL_DSHOT300` | Digital, 300 kbit/s, clocked out in PIO. |
| DShot600 | `MOTOR_PROTOCOL_DSHOT600` | Digital, 600 kbit/s, clocked out in PIO. |

DShot is a digital protocol: each motor update is a 16-bit, CRC-checked frame, so there is no pulse-width drift and no ESC calibration. A PIO state machine clocks the frame to each motor, on PIO1, leaving the SBUS receiver's PIO0 untouched. The control core still works in microseconds for every protocol, and the native output stage converts that to a DShot throttle value.

Servos run standard 50 Hz, 1 to 2 ms PWM. When disarmed the motor outputs hold the disarm value so the ESCs stay stopped, while servos stay live so you can check control surfaces on the bench. See [Wiring](getting-started/wiring.md) and [Arming and failsafe](user-guide/arming-and-failsafe.md).

## Pin configuration

Pins are fixed per board in `src/boards/<board>.h`. The defaults common to the Pi-Pico-form boards:

| Function | Pin |
|---|---|
| I2C0 SDA / SCL (IMU and baro) | GP4 / GP5 |
| SBUS receiver | GP1 |
| Motors (ESC) | GP10, GP11 |
| Servos | GP6 to GP9 (Tiny: GP12, GP13, GP8, GP9) |
| SD card (SPI0) | GP16 to GP19 |
| Companion UART | GP20 / GP21 |
| Status LED / onboard LED | GP14 / GP25 |

Full per-board tables and the annotated DHC-4 Caribou wiring diagram are in [Pin maps](reference/pin-maps.md).

## Configuration and build profiles

Everything is set at compile time in one file, `src/Config.h`. Every option is listed in [Config options](reference/config-options.md). Two reference profiles bracket the range:

- **Full (default).** The Caribou plane with SD logging, a barometer, the runtime parameter system, and the companion interface.
- **Minimal IMU-only.** A small RP2350 plus an MPU, no SD, no barometer, no WiFi, for a simple 5-channel plane. See the [minimal build preset](reference/config-options.md#minimal-imu-only-build).

## Channel map

The four flight controls sit on the AETR primaries (ch1 roll, ch2 pitch, ch3 throttle, ch4 yaw). The arm and stabilizer switches sit on high channels clear of the primaries. Aircraft-specific aux functions (gear, flaps, bay doors) are read by the [user sketch](user-guide/user-sketch.md). Full map and per-transmitter notes are in [Transmitter setup](user-guide/transmitter-setup.md).

## Companion tools

- **Browser configurator.** A single-page tool that speaks the `cp` serial protocol over WebUSB: edit parameters, watch live telemetry, decode logs, and flash firmware. Lives in `configurator/`.
- **WiFi companion.** A phone web UI, served either by an ESP32-C3 wired to the companion UART or by a Pico 2 W's own radio. Same `cp` protocol, with a cockpit instrument panel and parameter sliders.
- **Log analyzer.** `tools/log_analyzer/` reads a flight log and asks an AI model for a tuning diagnosis and gain suggestions you review and apply.

## Safety and failsafe

CrowPilot is pre-alpha and unflown. Read [Safety](safety.md) and [Disclaimer](disclaimer.md) in full before any motor spins. Arming requires the arm switch to be cycled through disarm since boot, the switch in the armed position, and the throttle at idle. On a lost link the failsafe holds a level, powered descent rather than cutting throttle. See [Arming and failsafe](user-guide/arming-and-failsafe.md).

## Get started

New here? Work through the getting-started path in order:

1. [What is CrowPilot](getting-started/what-is-crowpilot.md) - the pitch and who it is for.
2. [Hardware BOM](getting-started/hardware-bom.md) - the shopping list.
3. [Wiring](getting-started/wiring.md) - how to connect everything.
4. [Flashing](getting-started/flashing.md) - getting the firmware onto the board.
5. [Caribou Bench Test](getting-started/caribou-bench-test.md) - the smoke test before propellers go on, for the current first-flight airframe. The carried tailsitter has its own [bench test](getting-started/first-bench-test.md).

## Documentation map

- **Getting Started** takes you from an empty bench to a flashed, bench-verified flight controller: [what it is](getting-started/what-is-crowpilot.md), [BOM](getting-started/hardware-bom.md), [wiring](getting-started/wiring.md), [flashing](getting-started/flashing.md), [bench test](getting-started/caribou-bench-test.md).
- **User Guide** covers [transmitter setup](user-guide/transmitter-setup.md), [arming and failsafe](user-guide/arming-and-failsafe.md), [flight modes](user-guide/flight-modes.md), [tuning](user-guide/tuning.md), [calibration](user-guide/calibration.md), [reading logs](user-guide/reading-logs.md), and the [user sketch](user-guide/user-sketch.md) for custom aux behavior.
- **Airframes** documents each supported airframe: [fixed-wing plane](airframes/fixed-wing-plane.md), [tailsitter bicopter](airframes/tailsitter-bicopter.md).
- **Developer Guide** explains the [architecture](developer-guide/architecture.md), the [algorithms](developer-guide/algorithms.md), the [coding standards](developer-guide/coding-standards.md), and how to [add a board](developer-guide/adding-a-board.md) or [add an airframe](developer-guide/adding-an-airframe.md).
- **Reference** is the lookup material: every [config option](reference/config-options.md), every [pin map](reference/pin-maps.md), and the [telemetry format](reference/telemetry-format.md).
- **Project** tracks the [roadmap](roadmap.md) and the [changelog](changelog.md).

## License

CrowPilot firmware is licensed GPL-3.0-or-later. Documentation is licensed CC-BY-SA-4.0. See the [LICENSE](https://github.com/crowpilot-fc/crowpilot/blob/main/LICENSE) file in the repository for the full text.

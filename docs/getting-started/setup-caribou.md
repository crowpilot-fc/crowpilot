<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Setup: DHC-4 Caribou twin

The Caribou is CrowPilot's first-flight airframe: a twin-engine cargo plane
based on the open-source DHC-4 Caribou STL set. This page covers the
firmware-side setup. The bench-verification procedure is in
[caribou-bench-test.md](caribou-bench-test.md) and the canonical wiring is in
[wiring.md](wiring.md).

## What you need

- An RP2350 board: WeAct RP2350A_V10 (the default profile), Waveshare
  RP2350-Tiny, Raspberry Pi Pico 2 W, or SmartElex RP2350A NEO.
- An MPU-6500 (preferred) or MPU-6050 IMU. Optional BMP388 or BMP280 barometer.
- An SBUS or CRSF (ELRS) receiver with at least 10 channels (AETR, the high
  switches, and the Caribou's aux for the user sketch).
- The Caribou propulsion and surface set: 2 ESCs and motors, 2 aileron servos,
  elevator, rudder, plus the aux (retracts, two flap pairs, bay doors). A 5 V
  UBEC for the FC, receiver, and servos.

## Wiring

The firmware mixer drives 2 motors and 4 surfaces (2 ailerons + elevator +
rudder). The Caribou user sketch (`user-sketch.ino`, `ENABLE_USER_HOOK = 1`)
drives the aux servos (flaps, retracts, bay doors, nav LED) from extra
channels. The board profiles set the flight pins:

| Function | Tiny | WeAct / Pico 2 W | NEO |
|---|---|---|---|
| IMU SDA / SCL | GP4 / GP5 | GP4 / GP5 | GP28 / GP29 |
| Receiver | GP1 | GP1 | GP1 |
| Motor 1 (right) | GP10 | GP10 | GP14 |
| Motor 2 (left) | GP11 | GP11 | GP15 |
| Aileron L | GP12 | GP6 | GP26 |
| Aileron R | GP13 | GP7 | GP27 |
| Elevator | GP8 | GP8 | GP2 |
| Rudder | GP9 | GP9 | GP3 |

The aux pins (retracts, flaps, bay doors) are documented in
[wiring.md](wiring.md) and the per-board pin maps in
[pin-maps.md](../reference/pin-maps.md). The nose wheel steers mechanically off
the rudder servo.

## Channel plan

The Caribou uses the documented high-channel layout:

| Channel | Function | Owner |
|---|---|---|
| 1-4 | AETR primaries | firmware |
| 5 | Throttle 2 | firmware mixer (driven from the throttle stick) |
| 6 | Aileron 2 | firmware mixer |
| 7 | Landing gear | user sketch |
| 8 | Flap 1 | user sketch |
| 9 | Bay doors | user sketch |
| 10 | Flap 2 | user sketch |
| 13 | Altitude hold (optional) | firmware |
| 14 | Flight mode (LOW angle / MID horizon / HIGH manual) | firmware |
| 15 | Transition (unused on a plane) | firmware |
| 16 | Arm / cut | firmware |

The arm switch on ch8 is mandatory. Map them on the transmitter per
[transmitter-setup.md](../user-guide/transmitter-setup.md).

## `Config.h`

```
#define BOARD              BOARD_WEACT_RP2350A_V10   // or your board
#define AIRFRAME           AIRFRAME_PLANE_TWIN_CARGO  // default
#define IMU_TYPE           IMU_MPU6500
#define BARO_TYPE          BARO_BMP388                // or BARO_NONE
#define RX_PROTOCOL        RX_SBUS                    // or RX_CRSF
#define ENABLE_PLANE_STAB  1
#define ENABLE_USER_HOOK   1                          // drives the Caribou aux
#define ENABLE_TELEMETRY_LOG 1                        // SD blackbox on
```

Optional features that fit a twin: `ENABLE_DIFF_THRUST_YAW = 1` adds a
yaw-by-thrust mix. `ENABLE_TURN_COORDINATION = 1` adds auto-rudder and
bank-compensated pitch. `ENABLE_BATTERY_MONITOR = 1` adds pack-voltage sensing
on an ADC pin (see [config-options.md](../reference/config-options.md)).

## Flight modes

The flight-mode switch (ch7) reads as three positions: LOW angle, MID horizon,
HIGH manual passthrough. Fly the maiden in LOW (angle). See
[flight-modes.md](../user-guide/flight-modes.md).

## Tuning

Plane gains are runtime parameters (`kp_stab_roll` and so on), defaulting to
the `Config.h` values. The configurator sliders set them and a transmitter
live-tune knob trims them in flight, as for the tailsitter PID. See the gain
section in [setup-simple-plane.md](setup-simple-plane.md).

## Before you fly

Work through [caribou-bench-test.md](caribou-bench-test.md). Confirm every
channel, the arm switch behavior, the disarmed motors stay stopped, and the
control surfaces deflect to oppose tilt. The Caribou is the v1.0 acceptance
airframe; the maiden gates the rest of the v1.0 release.

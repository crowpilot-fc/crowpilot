<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Setup: Quad X

A four-motor X quadcopter, no servos. The firmware has two control laws,
selectable by the stabilizer switch (`CHANNEL_STAB`): a self-leveling angle
mode and an acro rate mode. SITL-validated against the bundled quad rigid-body
model.

## What you need

- An RP2350 board: Tiny, WeAct, Pico 2 W, or SmartElex NEO.
- An MPU-6500 (preferred) or MPU-6050 IMU.
- An SBUS or CRSF (ELRS) receiver with at least 6 channels.
- A quad frame, four motors, four ESCs, a 4-in-1 ESC or four discrete ESCs, a
  battery, and a UBEC for the FC and receiver. DShot-capable ESCs strongly
  recommended (see below).

## Wiring

Four motor outputs, no servos. The board's motor pins and the first two servo
pins carry the four ESC signals:

| Function | Tiny | WeAct / Pico 2 W | NEO |
|---|---|---|---|
| IMU SDA / SCL | GP4 / GP5 | GP4 / GP5 | GP28 / GP29 |
| Receiver | GP1 | GP1 | GP1 |
| Motor 1 (front right) | GP10 | GP10 | GP14 |
| Motor 2 (front left) | GP11 | GP11 | GP15 |
| Motor 3 (rear right) | GP12 | GP6 | GP26 |
| Motor 4 (rear left) | GP13 | GP7 | GP27 |

Connect each ESC to its motor-position pin. Motor numbering uses the airframe
constants `MOTOR_FRONT_RIGHT` / `FRONT_LEFT` / `REAR_RIGHT` / `REAR_LEFT`.
Propeller direction: front-right and rear-left spin counter-clockwise (viewed
from above), front-left and rear-right spin clockwise. Diagonal pairs spin the
same direction so their reaction torques cancel in a hover.

## Motor protocol

Use **DShot300** or **DShot600** for a quad. The 16-bit CRC-checked digital
frames have no calibration step, no pulse-width drift, and on a
BLHeli_S/BLHeli_32 ESC unlock bidirectional eRPM telemetry that feeds the
RPM-tracking dynamic notch (`ENABLE_DSHOT_BIDIR = 1` and
`ENABLE_DYNAMIC_NOTCH = 1`). On the RP2350, DShot uses PIO1; the SBUS receiver
uses PIO0, so they do not collide.

PWM and OneShot125 still work but lose the digital advantages.

## Channel plan

A quad needs at least 6 channels: AETR primaries, arm, and the angle/acro
flight-mode switch.

| Channel | Function |
|---|---|
| 1-4 | AETR primaries |
| 5 | Arm | (or 16, set `CHANNEL_ARM` per your radio) |
| 6 | Flight mode | (or 14, set `CHANNEL_STAB`. LOW angle, HIGH manual / acro depending on the map.) |

Default mode mapping treats LOW as angle. To reach acro (rate) mode in flight,
set one of the `PLANE_MODE_SW_*` positions to `PLANE_MODE_RATE` in
`Config.h`, even though the option names live under the plane-stab section.
The quad reads `CHANNEL_STAB > RC_MID_US` as acro and below as angle.

## `Config.h`

```
#define BOARD            BOARD_WAVESHARE_RP2350_TINY     // or your board
#define AIRFRAME         AIRFRAME_QUAD_X
#define IMU_TYPE         IMU_MPU6500
#define BARO_TYPE        BARO_NONE                        // a quad rarely flies altitude hold in v1
#define RX_PROTOCOL      RX_CRSF                          // ELRS is the usual quad RX
#define MOTOR_PROTOCOL   MOTOR_PROTOCOL_DSHOT600
#define ENABLE_PLANE_STAB 0                               // quad uses its own angle/rate loops
// optional: ENABLE_DSHOT_BIDIR = 1 and ENABLE_DYNAMIC_NOTCH = 1 for RPM-tracking notch
// remap CHANNEL_ARM and CHANNEL_STAB to whatever your radio has
```

`MAX_ACRO_RATE_DPS` (default 360 dps) sets the full-stick rate in acro mode.
The angle limits `MAX_ROLL_ANGLE_DEG` and `MAX_PITCH_ANGLE_DEG` cap stick
authority in angle mode.

## Tuning

Quad gains live in `Config.h` as `constexpr` for now (not yet runtime params,
unlike the plane stabilizer). Edit and reflash:

- Angle mode: tailsitter PID gains apply (`Kp_*_hover` / `Kd_*_hover`).
- Acro (rate) mode: `KP_RATE_ROLL`, `KP_RATE_PITCH`, `KP_RATE_YAW`,
  `KI_RATE_*`.

Start low, fly in angle mode, raise gains until the props begin to oscillate,
then back off about a third. Acro tuning comes after.

## Before you fly

Propellers OFF for the bench checks. Confirm: arm switch direction, motor
spin direction and order (front-right CCW, front-left CW, rear-right CW,
rear-left CCW from above), no oscillations at idle, and that tilting the quad
deflects motor commands to oppose the tilt. Then a tethered first hover
before free flight.

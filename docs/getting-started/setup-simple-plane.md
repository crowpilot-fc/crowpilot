<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Setup: simple fixed-wing plane

The smallest CrowPilot build. One RP2350 board, one MPU IMU, a receiver, and
your plane's ESC and servos. No barometer, no SD card, no WiFi. It gives a
standard aileron, elevator, rudder, and throttle plane self-leveling
stabilization, with horizon and manual modes available on a switch.

This is one of a family of per-setup guides in this folder. It covers any
single-motor AETR plane: a trainer, a sport plane, or a Gee Bee-style model.
For the Caribou twin, the flying wing, the quad, or the tailsitter, see their
own setup pages.

## What you need

- An RP2350 board: Waveshare RP2350-Tiny or SmartElex RP2350A NEO.
- An MPU-6500 (preferred) or MPU-6050 IMU on I2C. Buy a genuine part. The
  driver checks the chip ID and refuses clones.
- An SBUS or CRSF (Crossfire / ELRS) receiver with at least 5 channels.
- The plane: one ESC and motor, one or two aileron servos, an elevator servo,
  a rudder servo, and a 5 V UBEC for the receiver and servos.

## Wiring

Power the board and receiver from a 5 V UBEC. Tie all grounds together.

### Waveshare RP2350-Tiny

| Function | Pin |
|---|---|
| IMU SDA / SCL | GP4 / GP5 |
| Receiver (SBUS or CRSF RX) | GP1 |
| ESC signal | GP10 |
| Aileron servo (left) | GP12 |
| Aileron servo (right) | GP13 |
| Elevator servo | GP8 |
| Rudder servo | GP9 |
| Status LED (with 470 ohm) | GP14 |

### SmartElex RP2350A NEO

| Function | Pin |
|---|---|
| IMU SDA / SCL | GP28 / GP29 |
| Receiver (SBUS or CRSF RX) | GP1 |
| ESC signal | GP14 |
| Aileron servo (left) | GP26 |
| Aileron servo (right) | GP27 |
| Elevator servo | GP2 |
| Rudder servo | GP3 |
| Status LED (with 470 ohm) | GP24 |

The NEO onboard LED is a WS2812, which the firmware does not drive, so use an
external status LED on the pin above.

## Ailerons: one channel or two

The firmware mixer always computes two aileron outputs, a left and a right,
driven in opposite directions for roll. How you wire them is your choice:

- **Two servos, two pins (recommended).** Connect each aileron servo to its
  own pin (aileron-left and aileron-right). The flight controller drives them
  differentially. No Y harness, and this is what later enables flaperons,
  differential aileron, and crow braking.
- **One channel, Y harness.** Connect both aileron servos to the aileron-left
  pin through a Y harness, and reverse one servo (mirror mount, reversed horn,
  or a servo reverser) so the two surfaces move oppositely. Fewer pins, but
  you give up independent aileron control on that build. Fine for a basic
  trainer.
- **One aileron servo.** Connect it to the aileron-left pin only.

CrowPilot has no per-servo reverse in software yet, so set aileron direction
mechanically and verify it on the bench: tilting the plane to the right must
deflect the ailerons to roll it back left. Leave the aileron-right pin unused
if you wire a single channel.

## Channel plan

The default channel map is built for a many-channel transmitter, so remap the
switches to low channels in `Config.h` to fit a small radio. Arm
(`CHANNEL_ARM`) is mandatory: a radio that cannot drive it cannot arm.

| Radio | Channels | `Config.h` |
|---|---|---|
| 5-channel | AETR + arm | `CHANNEL_ARM = 5`. The mode channel, undriven, sits at center and selects horizon (self-levels). For pure angle with no mode switch, set all three `PLANE_MODE_SW_*` to `PLANE_MODE_ANGLE`. |
| 6-channel | + flight mode | also `CHANNEL_STAB = 6` (LOW angle, MID horizon, HIGH manual) |
| 7-8 channel | + gain knobs | `LIVE_TUNE_CH_KP = 7`, `LIVE_TUNE_CH_KD = 8`, `ENABLE_LIVE_TUNE = 1`. See the gain note below: the knobs do not affect plane gains yet. |

## `Config.h`

```
#define BOARD       BOARD_WAVESHARE_RP2350_TINY   // or BOARD_SMARTELEX_RP2350A_NEO
#define AIRFRAME    AIRFRAME_PLANE_SINGLE
#define IMU_TYPE    IMU_MPU6500                    // or IMU_MPU6050
#define BARO_TYPE   BARO_NONE
#define RX_PROTOCOL RX_SBUS                        // or RX_CRSF
#define ENABLE_PLANE_STAB    1
#define ENABLE_TELEMETRY_LOG 0
#define ENABLE_PARAM_PERSIST 0
#define ENABLE_COMPANION_CLI 0
// plus the CHANNEL_* remaps from the table above
```

## Flight modes

The flight-mode switch (`CHANNEL_STAB`) is read as three positions:

- LOW: angle, self-leveling. Fly the maiden here.
- MID: horizon, self-levels near center stick, full control at full stick.
- HIGH: manual passthrough, no stabilization.

A two-position switch reaches angle and manual only.

## Tuning the gyro gains

The plane stabilizer gains are compile-time constants in `Config.h`:
`KP_STAB_ROLL`, `KD_STAB_ROLL`, `KP_STAB_PITCH`, `KD_STAB_PITCH`, `KD_STAB_YAW`,
and the rate-mode gains `KP_PLANE_RATE_ROLL` and `KP_PLANE_RATE_PITCH`. Edit
them and reflash.

There is no onboard gain potentiometer on the board. The transmitter live-tune
knobs and the configurator sliders adjust gains in flight, but today they act
on the tailsitter PID parameters only, not the plane stabilizer, so on a plane
they have no effect yet. Tune plane gains in `Config.h` for now. Start low, fly
in angle mode, raise each gain until the surfaces just begin to oscillate, then
back off about a third.

## Before you fly

- Bench-verify every channel and the arm switch per
  [first-bench-test.md](first-bench-test.md).
- Confirm the stabilization direction: tilt the plane and check each surface
  deflects to oppose the tilt. Fix any backward servo mechanically.
- Confirm the disarmed motor stays stopped. Keep propellers off until then.
- Read [SAFETY.md](../../SAFETY.md) and [DISCLAIMER.md](../../DISCLAIMER.md) in
  full first.

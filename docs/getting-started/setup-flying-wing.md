<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Setup: flying wing (delta)

A flying wing has one motor and two elevons (combined elevator and aileron) on
the trailing edge, with no separate rudder. The firmware mixer reuses the
tailsitter's forward-flight elevon allocation, driven by the plane stabilizer.
SITL-validated.

## What you need

- An RP2350 board: Tiny, WeAct, Pico 2 W, or SmartElex NEO.
- An MPU-6500 or MPU-6050 IMU.
- An SBUS or CRSF (ELRS) receiver with at least 5 channels.
- The plane: one ESC and motor, two elevon servos. A 5 V UBEC.

## Wiring

The mixer drives 1 motor and 2 elevons:

| Function | Tiny | WeAct / Pico 2 W | NEO |
|---|---|---|---|
| IMU SDA / SCL | GP4 / GP5 | GP4 / GP5 | GP28 / GP29 |
| Receiver | GP1 | GP1 | GP1 |
| ESC signal | GP10 | GP10 | GP14 |
| Elevon L | GP12 | GP6 | GP26 |
| Elevon R | GP13 | GP7 | GP27 |
| Status LED | GP14 (external) | GP14 (external) | GP24 (external) |

The flying-wing mixer is sized for two servos, so only the first two servo
slots are used. The third and fourth slots are unused on this airframe.

## Mixing

In the firmware, each elevon carries pitch (both deflect together) and roll
(differential), the same allocation the tailsitter uses in forward flight.
Direction follows the conventional plane: a positive roll command lifts the
left elevon and lowers the right. Set elevon direction on the bench. If a
servo is backward, fix it mechanically or enable `ENABLE_SERVO_CONFIG = 1` and
flip its `SERVO_REVERSE[]` entry.

A pure flying wing has no rudder. The plane stabilizer's yaw damper output is
unused on this airframe. The flaperon and airbrake wing options (which fold
into the aileron commands of the conventional planes) do not apply to the
elevon mixer.

## Channel plan

5-channel minimum (AETR + arm). 6-channel adds the flight-mode switch.

| Radio | Channels | `Config.h` |
|---|---|---|
| 5 ch | AETR + arm | `CHANNEL_ARM = 5`. Mode channel undriven sits at center, which selects horizon (self-levels). |
| 6 ch | + mode | also `CHANNEL_STAB = 6`. LOW angle, MID horizon, HIGH manual. |
| 7-8 ch | + gain knobs | `LIVE_TUNE_CH_KP = 7`, `LIVE_TUNE_CH_KD = 8`, `ENABLE_LIVE_TUNE = 1`. |

## `Config.h`

```
#define BOARD              BOARD_WAVESHARE_RP2350_TINY  // or your board
#define AIRFRAME           AIRFRAME_PLANE_FLYING_WING
#define IMU_TYPE           IMU_MPU6500                    // or IMU_MPU6050
#define BARO_TYPE          BARO_NONE
#define RX_PROTOCOL        RX_SBUS                        // or RX_CRSF
#define ENABLE_PLANE_STAB  1
// CHANNEL_* remaps from the table above
```

## Flight modes and tuning

Same as the simple plane: ch7 (or your remapped mode channel) gives LOW
angle, MID horizon, HIGH manual. Plane gains are runtime parameters tunable
from the configurator or a live-tune knob, with the `Config.h` values as
defaults. See [setup-simple-plane.md](setup-simple-plane.md#tuning-the-gyro-gains).

## Before you fly

Bench-verify the elevon directions: pitching the wing up must deflect both
elevons down to push the nose back down; tilting it to the right must deflect
the left elevon up and the right down to roll it back left. Fix any backward
elevon mechanically or with `SERVO_REVERSE[]`. Confirm the disarmed motor
stays stopped. Props off until then. Read
[SAFETY.md](../../SAFETY.md) and [DISCLAIMER.md](../../DISCLAIMER.md) first.

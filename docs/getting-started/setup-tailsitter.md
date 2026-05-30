<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Setup: tailsitter bicopter

A tailsitter VTOL bicopter: two motors plus two elevons, hovering nose-up and
transitioning to wing-borne forward flight by pitching the airframe down. This
was CrowPilot's origin airframe and remains a fully carried airframe alongside
the Caribou. It uses its own angle PID with hover and forward-flight gain sets
blended by a transition fader.

## What you need

- An RP2350 board: Tiny, WeAct, Pico 2 W, or SmartElex NEO.
- An MPU-6500 (preferred) or MPU-6050 IMU. Optional BMP388 or BMP280 baro.
- An SBUS or CRSF (ELRS) receiver with at least 6 channels (AETR, arm, and the
  transition channel).
- The airframe: two motors and ESCs with one CW and one CCW propeller pair,
  two metal-gear servos for the elevons, a 4S LiPo, and a 5 V UBEC. The
  Eclipson E-VTOL-1 (about 1 kg) is the reference build, with the FliteTest
  Kraken Mk2 (about 2.5 kg) as a heavier alternate.

## Wiring

| Function | Tiny | WeAct / Pico 2 W | NEO |
|---|---|---|---|
| IMU SDA / SCL | GP4 / GP5 | GP4 / GP5 | GP28 / GP29 |
| Receiver | GP1 | GP1 | GP1 |
| Motor (right) | GP10 | GP10 | GP14 |
| Motor (left) | GP11 | GP11 | GP15 |
| Elevon L | GP12 | GP6 | GP26 |
| Elevon R | GP13 | GP7 | GP27 |
| Status LED | GP14 (external) | GP14 (external) | GP24 (external) |

Mount the two motors so their propellers spin in opposite directions (one CW,
one CCW) so reaction torque cancels at hover and the differential gives yaw.
The two elevons hinge at the trailing edge of the wing, deflecting downward in
hover (for pitch) and conventionally in forward flight (for pitch and roll).

## Channel plan

The tailsitter needs an extra channel for the transition fader on top of AETR
and arm:

| Channel | Function |
|---|---|
| 1-4 | AETR primaries |
| 15 | Transition (HIGH end = hover, LOW end = forward) |
| 16 | Arm / cut (LOW armed, HIGH disarmed) |

On a low-channel radio you can remap `CHANNEL_TRANSITION` and `CHANNEL_ARM` to
your available channels. The transition switch can be a 2-position (hover and
forward) or a knob.

## `Config.h`

```
#define BOARD              BOARD_WAVESHARE_RP2350_TINY    // or your board
#define AIRFRAME           AIRFRAME_TAILSITTER_BICOPTER
#define IMU_TYPE           IMU_MPU6500
#define BARO_TYPE          BARO_BMP388                     // or BARO_NONE
#define RX_PROTOCOL        RX_SBUS                         // or RX_CRSF
#define ENABLE_PLANE_STAB  0                               // the tailsitter uses its own PID
```

`TRANSITION_SLEW_RATE` (default about a 3-second full transition) bounds how
fast the fader follows the channel command, so even a snapped switch is a
controllable maneuver. The `MAX_ROLL_ANGLE_DEG`, `MAX_PITCH_ANGLE_DEG`, and
`MAX_YAW_RATE_DPS` limits cap stick authority.

## Flight modes

The transition channel commands the regime: HIGH is hover (fader = 1.0), LOW
is forward (fader = 0.0). The fader slews toward the command at the bounded
rate. The attitude reference, controller gains, and mixer feedforward all
blend continuously across the transition. There is no discrete mode switch in
the control path; the displayed `mode=hover/transitioning/forward` is derived
from the fader for telemetry only. See
[flight-modes.md](../user-guide/flight-modes.md).

## Tuning

Tailsitter PID gains are runtime parameters (`kp_roll_hover`, `kd_roll_ff`,
and so on) with hover and forward-flight sets blended by the fader. The
configurator sliders set them and a live-tune knob trims them in flight. Tune
hover first, then forward flight, per
[tuning.md](../user-guide/tuning.md).

## Before you fly

Bench-verify per [first-bench-test.md](first-bench-test.md): every channel,
the arm switch, motor spin directions, that the elevons deflect to oppose
tilt, and that the disarmed motors stay stopped. The first powered flight is a
tethered hover. Free hover, transition out, and transition back in come in
sequence, per the v1 acceptance criteria.

Read [SAFETY.md](../../SAFETY.md) and [DISCLAIMER.md](../../DISCLAIMER.md)
first. The tailsitter is unforgiving of a bad transition.

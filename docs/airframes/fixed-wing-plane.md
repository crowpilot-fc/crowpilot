<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Fixed-wing plane

The fixed-wing plane is the current first-flight airframe. The reference
aircraft is the DHC-4 Caribou, a twin-engine high-wing cargo plane.
Select it with `AIRFRAME = AIRFRAME_PLANE_TWIN_CARGO` in `Config.h`, which
is the default. The plane airframes require `ENABLE_PLANE_STAB = 1`.

A single-engine plane mixer (`AIRFRAME_PLANE_SINGLE`) shares the same
stabilizer and is carried for the v1.2 Gee Bee.

## Layout

The firmware mixer drives four control-surface servos and two motors:

- Two ailerons, one per wing, deflecting oppositely for roll.
- One elevator for pitch.
- One rudder for yaw.
- Two motors, one per nacelle, sharing the throttle command.

Body axes are x out the nose, y out the right wing, z through the belly.
Pin assignments are in [Pin maps](../reference/pin-maps.md) and the wiring
is in [Wiring](../getting-started/wiring.md).

## Control allocation

The mixer takes a throttle, roll, pitch, and yaw demand and allocates them:

- Throttle goes to both motors equally.
- Roll drives the two ailerons differentially.
- Pitch drives the elevator.
- Yaw drives the rudder. With `ENABLE_DIFF_THRUST_YAW = 1`, yaw also splits
  the two motors around the throttle command for differential-thrust yaw.

Surface travel is set per axis by `AILERON_TRAVEL`, `ELEVATOR_TRAVEL`, and
`RUDDER_TRAVEL`. The servo outputs center at the mid-pulse and swing by the
travel fraction.

## Stabilizer and manual passthrough

A transmitter switch on `CHANNEL_STAB` (channel 14) selects the flight mode:

- **LOW: stabilized.** A wing leveler holds the roll angle, a pitch hold
  holds the pitch angle, and a yaw damper takes out the wallow. The sticks
  command attitude rather than raw surface deflection.
- **HIGH: passthrough.** Full manual. The sticks drive the surfaces
  directly with no stabilization.

Trim the aircraft for hands-off level flight in passthrough first, then tune
the stabilizer gains. The procedure is in [Tuning](../user-guide/tuning.md).
The lost-link failsafe holds a level, powered descent with the stabilizer
active.

## Aux functions

The Caribou's other functions are handled by `user-sketch.ino`, not the
firmware mixer: the second engine and second aileron, the landing-gear
retracts, the flaps, the cargo-bay doors, the nose-wheel steering, and the
navigation lights. See the [User sketch](../user-guide/user-sketch.md) and the
[Transmitter setup](../user-guide/transmitter-setup.md) channel map.

## Build notes

- The aileron, elevator, and rudder signs depend on the servo linkages.
  Verify every axis on the bench, propellers off, before flight. See the
  [Caribou bench test](../getting-started/caribou-bench-test.md).
- The Caribou rests on its gear, so the firmware boots level and disarmed.

The stabilizer gains are provisional and are set by bench tuning. See
[Tuning](../user-guide/tuning.md).

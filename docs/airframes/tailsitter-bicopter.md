<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Tailsitter bicopter

The tailsitter bicopter is the v1.0 reference airframe. It is a 3D-printed
VTOL aircraft that takes off and lands on its tail, hovers with the nose
pointing up, and transitions to wing-borne forward flight by pitching roughly
ninety degrees toward horizontal.

Select it with `AIRFRAME = AIRFRAME_TAILSITTER_BICOPTER` in `Config.h`. It is
the default.

## Layout

The airframe has two effectors:

- Two motors on the wing, one to the left and one to the right of
  centerline, with fixed propellers.
- Two elevons, one control surface behind each motor in its propeller wash.

There is no separate rudder. Body axes are x out the nose, y out the right
wing, z through the belly.

## The two regimes

A tailsitter has one airframe and one set of actuators but two
aerodynamically distinct flight regimes.

In hover the aircraft is thrust-borne and sits nose-up. Attitude is held by
differential motor thrust and by the elevons working in propeller wash.

In forward flight the aircraft is wing-borne and roughly horizontal. Attitude
is held by the elevons with airspeed over them, and the motors mostly provide
thrust.

The control core keeps the aircraft controllable continuously across the
whole range between these two regimes, including partial transition. See
[Flight Modes](../user-guide/flight-modes.md).

## Mixing

The mixer allocates throttle and the stabilized roll, pitch, and yaw demands
to the two motors and two elevons. From the geometry:

- Common thrust on both motors gives lift in hover and thrust in forward
  flight.
- Differential thrust between the motors gives yaw.
- Both elevons deflecting together gives pitch.
- The elevons deflecting oppositely gives roll.

That allocation is the same in both regimes, because the geometry is fixed.
The difference between hover and forward flight is in how much authority each
effector has, propeller wash versus free airspeed, and that is handled by the
regime-scheduled controller gains rather than by the mixer. A direct-stick
feedforward on the elevons fades in toward forward flight so the surfaces
respond crisply at airspeed.

## Build notes

- The elevon neutral positions are a mechanical trim. Set `ELEVON_LEFT_TRIM`
  and `ELEVON_RIGHT_TRIM` so the surfaces sit at their true geometric center.
- The elevon and motor mixing signs depend on the servo linkages and the
  motor wiring. Verify every axis on the bench, propellers off, before flight.
  See [First Bench Test](../getting-started/first-bench-test.md).
- The reference build hovers nose-up on its tail at boot, so the firmware
  starts in the hover regime.

The mixer gains are provisional and are set by bench tuning. See
[Tuning](../user-guide/tuning.md).

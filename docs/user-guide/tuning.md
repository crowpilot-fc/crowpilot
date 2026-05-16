<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Tuning

Every gain in the CrowPilot control core ships as a provisional value. The
firmware will hold an attitude with the defaults, but it is not tuned for any
particular airframe. This page describes how to tune it.

Tune on a tether or with a hand-held bench rig first, propellers balanced and
the airframe restrained. Read [Safety](../safety.md) before any powered test.

## The gain sets

The stabilizer runs one PID controller per axis: roll, pitch, and yaw. Each
axis has a proportional, integral, and derivative gain.

The proportional and derivative gains have two sets, one for hover and one
for forward flight, because the two regimes have very different control
authority. The controller interpolates between them continuously by the
transition fader. The integral gains are shared across regimes.

The gains are runtime parameters. The `Config.h` constants seed the registry,
and live tuning and flash persistence work from the registry.

## The tuning order

Tune one axis at a time, and within an axis follow this order:

1. Derivative gain first. Raise it until the axis stops oscillating after a
   disturbance, then back off slightly.
2. Proportional gain next. Raise it until the axis tracks a setpoint
   crisply, then back off before it oscillates.
3. Integral gain last. Raise it just enough to remove a steady offset. The
   integral gains start at zero.

Tune the hover regime before the forward-flight regime. Hover is the regime
the aircraft must hold to take off and land safely.

The bench checks in [First Bench Test](../getting-started/first-bench-test.md)
verify that each stick produces the correct stabilized response and that the
stabilizer opposes a hand-induced disturbance without sustained oscillation.

## Live tuning

When `ENABLE_LIVE_TUNE` is set, two transmitter channels scale the active
proportional and derivative gains in flight. `LIVE_TUNE_CH_KP` scales every
blended P gain and `LIVE_TUNE_CH_KD` scales every blended D gain, each by up
to plus or minus `LIVE_TUNE_RANGE`. A knob at its center leaves the gains
unchanged.

Live tuning lets you find a good multiplier in the air without reflashing.
The integral gains are not live-tuned.

## Saving tuned gains

The live-tune multipliers are transient. To bake them in, use the save
gesture: disarmed, hold the yaw stick hard left for two seconds. The firmware
multiplies the current gains by the multipliers, writes the result to the
persistent parameters in flash, and recenters the knobs so the multipliers
return to one.

After a save, the tuned gains survive a power cycle. To make them the
permanent default, also copy the values into `Config.h`.

## The mixer gains

The mixer gains, the elevon travel, the differential-thrust gain, and the
elevon trims, are also provisional. Set the elevon trims first, mechanically,
so the surfaces sit at their true center. Then verify the travel gains give
full but not saturated deflection at full demand.

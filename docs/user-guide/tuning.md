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

## Plane tuning (fixed-wing airframes)

The Caribou twin-engine plane and the Gee Bee single-engine plane do not
use the tailsitter angle PID described above. They use the fixed-wing
stabilization subsystem (`cp::control::plane_stab`): a wing leveler, a
pitch-attitude hold, a yaw damper, and an optional barometric altitude
hold. The plane mixer consumes the stabilizer output directly.

Plane gains live in `Config.h`: `KP_STAB_ROLL`, `KD_STAB_ROLL`,
`KP_STAB_PITCH`, `KD_STAB_PITCH`, `KD_STAB_YAW`, and `KP_ALT` / `KD_ALT`
for altitude hold. Each axis output is
`STAB_OUTPUT_SCALE * (Kp * angle_error_deg - Kd * body_rate_dps)`,
clamped to the surface range. There is no hover-versus-forward gain set
on the plane. One gain set covers the whole flight envelope.

### Plane tuning order

1. **Bench first.** Confirm every control surface moves the correct way
   for a given stick input, with the airframe restrained on the bench. A
   reversed aileron is the single most common plane-maiden crash cause.
   Reverse on the transmitter, not in `Config.h`.
2. **Fly in passthrough.** Set the stab channel high so every stabilizer
   term drops out. Trim the aircraft for hands-off level flight in
   passthrough first. A plane that will not fly trimmed in manual will
   not fly stabilized.
3. **Enable stabilization.** Flip the stab channel low. With the airframe
   in level cruise, sticks centered, the wing leveler and pitch hold
   should hold attitude. Watch for oscillation on the `DEBUG_PRINT_STAB`
   line.
4. **Tune roll, then pitch.** Same D-then-P discipline as the tailsitter.
   Raise `KD_STAB_*` until a hand-induced disturbance recovers without
   oscillation, then `KP_STAB_*` until the recovery is crisp. A plane
   needs less gain than a tailsitter because the aerodynamic surfaces
   already provide damping.
5. **Yaw damper.** `KD_STAB_YAW` only needs to take the wallow out. Raise
   it until Dutch roll is damped. Do not chase a heading hold (v1 has
   none).
6. **Altitude hold last.** With `ENABLE_ALT_HOLD = 1`, engage the
   alt-hold channel in level cruise. Raise `KP_ALT` until the aircraft
   holds altitude without a slow porpoise. Raise `KD_ALT` if it
   overshoots the captured altitude. Altitude hold needs a healthy
   barometer.

Plane forward-flight tuning otherwise follows the same log-reading
discipline as the tailsitter: non-saturated surface output, low gyro
noise, stable attitude with sticks centered.

### EDF-specific notes

An electric ducted fan behaves differently from a propeller and changes
how the airframe is tuned.

- **High static thrust, narrow efficient band.** An EDF produces strong
  thrust over a narrower efficient throttle band than a propeller.
  Useful thrust starts higher up the throttle curve. Expect to cruise at
  a higher throttle percentage than a comparable prop plane, and set
  `KP_ALT` conservatively, because a small altitude error commands a
  throttle change that on an EDF can translate to a large thrust change.
- **Fast spool-up, but not instant.** An EDF spools faster than it sounds
  but still has rotational inertia. `KD_ALT` needs to be high enough
  that altitude hold does not chase the spool lag into a porpoise.
- **High current draw.** EDFs pull serious amps. Battery sag is
  significant: tune and trim on a battery state representative of
  mid-flight, not a fresh pack. Confirm the loop period and telemetry
  stay clean under sustained full-throttle bench runs before flight,
  because brownout-induced resets show up as gaps in the log.
- **Thrust line.** An EDF mounted on the fuselage usually has a thrust
  line close to the center of mass, so throttle changes produce little
  pitch coupling. If the duct is high or low, expect a throttle-to-pitch
  coupling that the pitch hold has to absorb. Tune pitch with throttle
  steps as well as attitude disturbances.

Start the plane stab gains lower than the prop defaults for an EDF
airframe. An EDF airframe is typically faster and the control surfaces
are more effective at speed, so a given `KP_STAB_*` produces a larger
moment. Raise gains from a conservative base.

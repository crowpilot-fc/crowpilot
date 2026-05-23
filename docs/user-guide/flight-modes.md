<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Flight modes

This page covers the tailsitter airframe. The DHC-4 Caribou plane, the
current first-flight aircraft, has no transition: it flies as a conventional
fixed-wing, with the stabilizer switch (ch14) selecting between the wing
leveler and full manual passthrough. See [transmitter-setup.md](transmitter-setup.md)
for the plane switches.

A tailsitter flies in two regimes, hover and forward flight, with a
continuous transition between them. CrowPilot manages that transition with a
single transmitter channel and a rate-limited internal value called the
fader.

## The transition channel

One transmitter channel, `CHANNEL_TRANSITION` (channel 15 by default),
commands the transition. The low end of the channel commands hover and the
high end commands forward flight. Assign it to a switch or a knob on your
transmitter.

A snapped switch is fine. The firmware does not act on the channel directly.

## The fader

The value the control core acts on is the fader: a normalized scalar, 1.0 at
hover and 0.0 at forward flight. The fader slews toward the commanded value
at a bounded rate, `TRANSITION_SLEW_RATE` fader units per second. The default
rate completes a full transition in about three seconds.

The slew exists because the airframe cannot rotate ninety degrees instantly.
An instantaneous attitude-reference change would command a maneuver the
aircraft cannot fly. The slew turns even a snapped switch into a controllable
transition.

The whole control law is continuous in the fader. The attitude reference, the
controller gains, and the mixer feedforward all blend smoothly as the fader
moves. There is no discrete mode switch in the control path.

## The flight-mode enumeration

For telemetry and display, a named flight mode is derived from the fader:

- Hover, when the fader is near 1.0.
- Transitioning, when the fader is between the two ends.
- Forward, when the fader is near 0.0.

This enumeration is informational only. It is logged and can be shown to the
pilot. It does not feed back into the control law.

## Boot state

A tailsitter rests on its tail, so the firmware boots in the hover regime
with the fader at 1.0. The lost-link failsafe also commands the hover end, so
a transition is never started by a lost link.

## Flying the transition

Take off and hover with the transition channel at the hover end. When you
have altitude and want forward flight, move the channel to the forward end
and let the fader slew the aircraft over. To return to hover, move the
channel back. Practice transitions with plenty of altitude.

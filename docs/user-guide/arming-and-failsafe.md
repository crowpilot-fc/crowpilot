<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Arming and failsafe

Two safety systems decide whether the motors can spin: the arm logic, which
the pilot controls, and the failsafe, which handles a lost radio link.

## Arming

The motors cannot spin unless the pilot has explicitly armed the aircraft.
The boot state is disarmed.

A transmitter switch channel, `CHANNEL_ARM` (channel 16 by default), arms and
disarms. The low position is the armed position and the high position is
disarm.

Disarming takes effect unconditionally and immediately, in any flight state.
Moving the switch to the disarm position always stops the motors.

Arming is gated. The aircraft will arm only when all of the following are
true at once:

- The aircraft is currently disarmed.
- The arm switch is in the armed position.
- The throttle stick is at idle, at or below `ARM_THROTTLE_MAX_US`.
- The arm switch has been seen in the disarm position at least once since
  boot.
- The pre-arm checks pass (see below).

The throttle-idle requirement prevents arming straight into a spun-up
throttle command. If you flip the arm switch with the throttle up, nothing
happens. Lower the throttle, then arm.

### Pre-arm checks

On top of the gates above, the aircraft refuses to arm unless:

- The IMU is healthy.
- The receiver link is up with valid channels and not in failsafe.
- The battery, if monitored (`ENABLE_BATTERY_MONITOR`), is at or above
  `BATTERY_ARM_MIN_CELL_V` per cell. A build with no battery monitor, or a
  bench setup on USB with no pack, skips this check.

Disarming is never gated by the pre-arm checks. The blocking reason is shown
in the `DEV` serial line as `prearm=ok` or `prearm=!IMU`, `!RX`, `!BATT`, and
in the configurator's pre-arm status. If the aircraft will not arm, read that
field first.

The seen-disarmed requirement means a board powered up with the switch
already in the armed position will not arm until you cycle the switch
through disarm. It also means a transmitter that never drives ch16 cannot
arm the aircraft at all, so assign the arm switch before you go to the
bench.

When disarmed, the motor outputs are driven below the ESC protocol's valid
pulse range, so the ESCs see no signal and stay silent. Servos still respond
when disarmed, so you can check control-surface movement on the bench.

## Failsafe

The failsafe handles the loss of the radio link. It does not cut the motors.
On a lost link the firmware holds a level, powered descent, a controlled
fall, rather than dropping the aircraft.

The failsafe triggers on any of three conditions:

- An RC channel outside the valid signal range.
- The SBUS protocol-level failsafe flag set by the receiver.
- No fresh receiver frame within `FS_LINK_TIMEOUT_US`, 100 ms by default.

While a trigger is active the failsafe replaces the receiver channels with a
fixed safe set: a throttle below hover for a gentle sink, centered roll,
pitch, and yaw, the arm channel held armed so the descent stays powered, and
the transition channel held at the hover end. When every trigger clears, the
raw receiver channels pass through again and normal control resumes.

The failsafe is active at boot, before the receiver has seen its first frame,
so the firmware never carries unsafe channel values forward from
uninitialized memory.

## Before every flight

- Confirm the arm switch disarms the aircraft.
- Confirm the aircraft will not arm with the throttle up.
- Confirm a normal arm works with the throttle at idle.
- Confirm that disarmed motors stay silent.

See [Caribou Bench Test](../getting-started/caribou-bench-test.md) for the full
pre-flight bench procedure.

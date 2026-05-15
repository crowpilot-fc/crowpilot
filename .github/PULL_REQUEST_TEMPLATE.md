<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

## Summary

One or two sentences on what this PR does and why.

## Changes

- Bullet list of the changes.

## Safety review

Tick if any apply. If any tick, link to the relevant section of [SAFETY.md](../SAFETY.md) and describe the testing performed.

- [ ] Touches the control loop, mixer, failsafe, actuator output, IMU read path, or RX read path.
- [ ] Changes failsafe behavior.
- [ ] Changes arming logic or throttle cut.
- [ ] Changes any pin assignment that drives a motor or servo.

## Testing

How this change was verified. Bench tests, SITL hover, HIL hover, tethered hover, free flight. Link telemetry logs if relevant.

## Checklist

- [ ] Commits are signed off with `-s` (DCO).
- [ ] CLA signed via CLA Assistant on first PR.
- [ ] New files carry the appropriate SPDX-License-Identifier header.
- [ ] CI passes locally where applicable.
- [ ] Public docs updated if the change is user-visible.

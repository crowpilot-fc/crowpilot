---
name: Bug report
about: Report a defect in firmware behavior, documentation, or build
title: '[bug] '
labels: bug
---

<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

## What happened

A clear description of what went wrong.

## What you expected

What you expected to happen instead.

## How to reproduce

1. Step one
2. Step two
3. Observed behavior

## Hardware

- Board: (e.g. Waveshare RP2350-Tiny)
- IMU: (e.g. MPU-6500)
- Barometer: (e.g. BMP388, BMP280, BARO_NONE)
- Receiver: (e.g. ELRS, SBUS)
- Airframe: (e.g. Eclipson E-VTOL-1, custom)

## Firmware version

Git commit hash or release tag.

## Config.h

Paste the relevant sections of your `Config.h` if non-default.

## Logs

If the crash was airborne, attach the binary log and the decoded CSV from `tools/log_to_csv.py`. If the failure was at bench, paste the serial output.

## Anything else

Anything else that would help diagnose the issue.

<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Calibration

CrowPilot has two one-shot calibration routines, the IMU bias calibration and
the ESC calibration. Each is gated by a compile-time flag, runs at boot, and
halts. Neither is part of normal flight.

## IMU bias calibration

A MEMS gyroscope and accelerometer each read a small non-zero value at rest.
Left uncorrected, the gyro offset makes the attitude estimate drift. The IMU
bias calibration measures the six offsets so they can be removed.

To run it:

1. Set `ENABLE_IMU_CALIBRATION` to `1` in `Config.h` and flash.
2. Place the airframe stationary and level, belly down, on a solid surface.
3. Power on and open the USB serial monitor.
4. The firmware averages a batch of samples over about two seconds, then
   prints the six bias values.
5. Copy the printed lines into the IMU section of `Config.h`, replacing the
   `GYRO_BIAS_X/Y/Z` and `ACC_BIAS_X/Y/Z` values.
6. Set `ENABLE_IMU_CALIBRATION` back to `0` and flash again.

The routine removes one g of gravity from the vertical accelerometer axis, so
the airframe must be belly down and level while it runs. Do not move it.

After applying the values, the gyro should read near zero at rest and the
attitude estimate should converge to level within a second or two of power
on.

## ESC calibration

ESC calibration teaches the ESCs the firmware's throttle pulse range. Most
modern ESCs do not need it. Run it only if your ESCs require manual range
learning.

This routine drives the motor pins. The reference ESCs are calibrated with
the battery disconnected at flash time and reconnected during the routine,
so the motors cannot spin up unexpectedly. Read the procedure printed on the
serial monitor and follow it exactly.

To run it:

1. Set `ENABLE_ESC_CALIBRATION` to `1` in `Config.h` and flash, with the ESC
   battery disconnected.
2. Power the board over USB and open the serial monitor.
3. The firmware emits the maximum pulse for six seconds. Connect the ESC
   battery during that window. The ESC enters calibration mode.
4. The firmware switches to the idle pulse for four seconds. The ESC stores
   the range.
5. The firmware halts. Set `ENABLE_ESC_CALIBRATION` back to `0` and flash
   again.

Propellers off for ESC calibration.

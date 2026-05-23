<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Telemetry format

CrowPilot logs a binary telemetry record to the SD card every tick the logger runs (100 Hz by default). This page is the field reference for the log. For how to decode and read a log in practice, see [user-guide/reading-logs.md](../user-guide/reading-logs.md).

## Overview

- Each record is exactly **109 bytes**, fixed-length.
- Records are written back-to-back with no file header, no footer, no separator. A log file is always an exact multiple of 109 bytes.
- Every record carries a `schema_version` byte so a decoder can detect a format change immediately. The current schema is version 1.
- All multi-byte fields are little-endian, matching the RP2350 and every common host.

At 100 Hz and 109 bytes, a log grows about 10.9 KB per second, so a 4 GB card holds well over 100 hours of flight.

## File naming

The logger writes zero-padded 8.3 filenames to the SD card root:

```
LOG0001.BIN
LOG0002.BIN
...
LOG9999.BIN
```

At boot the logger scans the card, finds the highest existing index, and starts the next file. A new file is also opened when the current one passes 16 MiB (about 25 minutes) or on a write error.

## Decoder

A reference Python 3 decoder ships in the repository at `tools/decode_log.py`. It has no third-party dependencies:

```bash
python tools/decode_log.py LOG0001.BIN > log0001.csv
```

It emits one CSV row per record. Column order follows the field table below, with derived engineering-unit columns appended (raw IMU counts converted to dps and g, raw flags expanded to boolean columns).

## Field reference

| Offset | Size | Field | Type | Meaning |
|---|---|---|---|---|
| 0 | 1 | `schema_version` | uint8 | Always 1 for this schema. |
| 1 | 8 | `t_us` | uint64 | Microseconds since power-on. |
| 9 | 4 | `loop_period_us` | uint32 | Measured loop period this tick. Target 1000. |
| 13 | 6 | `gyro_x/y/z` | int16 x3 | Raw gyro counts. Multiply by the gyro scale below. |
| 19 | 6 | `accel_x/y/z` | int16 x3 | Raw accel counts. Multiply by the accel scale below. |
| 25 | 2 | `imu_temp_raw` | int16 | Raw IMU temperature. MPU-6500: `temp_c = raw / 333.87 + 21.0`. MPU-6050: `temp_c = raw / 340.0 + 36.53`. |
| 27 | 4 | `pressure_pa` | int32 | Barometric pressure in pascals. |
| 31 | 4 | `baro_temp_cc` | int32 | Baro temperature in centidegrees Celsius (degrees C x 100). |
| 35 | 4 | `altitude_cm` | int32 | Altitude in cm, relative to ground pressure captured at init. Positive up. |
| 39 | 16 | `quat_w/x/y/z` | float32 x4 | Attitude quaternion. Magnitude normalized to 1.0 each tick. |
| 55 | 12 | `roll/pitch/yaw_fwd_deg` | float32 x3 | Euler angles, forward-flight convention, degrees. |
| 67 | 12 | `roll/pitch/yaw_hover_deg` | float32 x3 | Euler angles, hover convention, degrees. |
| 79 | 12 | `rc_ch1..ch6_us` | uint16 x6 | RC channel pulse widths in microseconds. |
| 91 | 4 | `fader` | float32 | Transition fader. 0.0 forward, 1.0 hover. |
| 95 | 4 | `motor1/2_us` | uint16 x2 | Commanded motor pulse widths (OneShot125, 125 to 250 us). |
| 99 | 4 | `servo_left/right_us` | uint16 x2 | Commanded servo pulse widths (1000 to 2000 us). |
| 103 | 3 | `pid_roll/pitch/yaw_q7` | int8 x3 | PID output in Q1.7. Divide by 127.0 to recover the normalized value. |
| 106 | 1 | `reserved` | uint8 | Always 0. |
| 107 | 1 | `flight_mode` | uint8 | 0 hover, 1 forward, 2 transitioning. |
| 108 | 1 | `status_flags` | uint8 | Bitmap, see below. |

### Gyro and accel scale

Raw IMU counts convert to engineering units with a scale that depends on the configured full-scale range. The v1 defaults are gyro ±2000 dps and accel ±8 g.

| Gyro range | dps per count | | Accel range | g per count |
|---|---|---|---|---|
| ±250 dps | 1 / 131.0 | | ±2 g | 1 / 16384.0 |
| ±500 dps | 1 / 65.5 | | ±4 g | 1 / 8192.0 |
| ±1000 dps | 1 / 32.8 | | ±8 g | 1 / 4096.0 |
| ±2000 dps | 1 / 16.4 | | ±16 g | 1 / 2048.0 |

The decoder applies the v1 default scales. If you change `IMU_GYRO_RANGE` or `IMU_ACCEL_RANGE` in `Config.h`, adjust the decoder to match.

### status_flags bitmap

| Bit | Mask | Name | Meaning |
|---|---|---|---|
| 0 | 0x01 | `ARMED` | Motors are live. |
| 1 | 0x02 | `FAILSAFE_ACTIVE` | Lost-link override in effect. |
| 2 | 0x04 | `THROTTLE_CUT` | Arm switch in the disarm/cut position (`CHANNEL_ARM` HIGH). |
| 3 | 0x08 | `IMU_FAULT` | IMU produced bad data this tick. |
| 4 | 0x10 | `BARO_FAULT` | Baro produced bad data this tick. |
| 5 | 0x20 | `RX_FAULT` | No RC frame in the last frame interval. |
| 6 | 0x40 | `LOG_OVERFLOW` | Reserved. Not set by v1; the logger rotates files rather than tracking buffer overflow. |
| 7 | 0x80 | reserved | Always 0. |

## Schema versioning

The `schema_version` byte at offset 0 is the contract between the firmware logger and the decoder. If the record layout changes, the version increments and the decoder can refuse or adapt. Any change to the schema must update the firmware logger, the decoder, and bump the version constant together.

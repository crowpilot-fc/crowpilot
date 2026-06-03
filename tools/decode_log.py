#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Nitin Kumar
#
# CrowPilot binary telemetry log decoder.
#
# Reads a `.BIN` file produced by the on-device SD logger and emits a
# CSV with one row per record on stdout. Schema is per
# `internal-docs/TELEMETRY_FORMAT.md` §3 and §5. Schema version is
# checked against the supported set per §6.3.
#
# Usage:
#   python tools/decode_log.py LOG0001.BIN > log0001.csv
#   python tools/decode_log.py --imu-type mpu6050 LOG0001.BIN > log.csv
#
# Python 3.8+. No third-party dependencies.

import argparse
import csv
import struct
import sys

# Record layout per TELEMETRY_FORMAT.md §3. The format string mirrors the
# field order in the C++ struct. 109 bytes total. `<` selects
# little-endian with no padding (matching the firmware's `#pragma pack(1)`).
RECORD_FMT = "<BQI3h3hhiii4f3f3f6Hf4H3bBBB"
RECORD_SIZE = struct.calcsize(RECORD_FMT)
assert RECORD_SIZE == 109, f"record size mismatch: got {RECORD_SIZE} expected 109"

SUPPORTED_SCHEMA_VERSIONS = (0x01,)

# Sensitivity scales per TELEMETRY_FORMAT.md §4.1. These match the v1
# firmware shipping with gyro = +-2000 dps and accel = +-8 g. A future
# revision will embed the active ranges in the log itself.
GYRO_LSB_PER_DPS = 16.4
ACCEL_LSB_PER_G = 4096.0

# Status flag bit masks per TELEMETRY_FORMAT.md §4.3.
FLAG_BITS = [
    ("armed",              0x01),
    ("failsafe_active",    0x02),
    ("throttle_cut",       0x04),
    ("imu_fault",          0x08),
    ("baro_fault",         0x10),
    ("rx_fault",           0x20),
    ("log_overflow",       0x40),
    ("imu_loss_degraded",  0x80),
]

CSV_HEADER = [
    "schema_version", "t_us", "loop_period_us",
    "gyro_x_raw", "gyro_y_raw", "gyro_z_raw",
    "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
    "accel_x_raw", "accel_y_raw", "accel_z_raw",
    "accel_x_g", "accel_y_g", "accel_z_g",
    "imu_temp_c",
    "pressure_pa", "baro_temp_c", "altitude_m",
    "quat_w", "quat_x", "quat_y", "quat_z",
    "roll_fwd_deg", "pitch_fwd_deg", "yaw_fwd_deg",
    "roll_hover_deg", "pitch_hover_deg", "yaw_hover_deg",
    "rc_ch1_us", "rc_ch2_us", "rc_ch3_us", "rc_ch4_us", "rc_ch5_us", "rc_ch6_us",
    "fader",
    "motor1_us", "motor2_us", "servo_left_us", "servo_right_us",
    "pid_roll", "pid_pitch", "pid_yaw",
    "flight_mode",
    "armed", "failsafe_active", "throttle_cut",
    "imu_fault", "baro_fault", "rx_fault", "log_overflow",
]


def decode_record(raw):
    fields = struct.unpack(RECORD_FMT, raw)
    (
        schema_version, t_us, loop_period_us,
        gyro_x, gyro_y, gyro_z,
        accel_x, accel_y, accel_z,
        imu_temp_raw,
        pressure_pa, baro_temp_cc, altitude_cm,
        quat_w, quat_x, quat_y, quat_z,
        roll_fwd_deg, pitch_fwd_deg, yaw_fwd_deg,
        roll_hover_deg, pitch_hover_deg, yaw_hover_deg,
        rc1, rc2, rc3, rc4, rc5, rc6,
        fader,
        motor1_us, motor2_us, servo_left_us, servo_right_us,
        pid_roll_q7, pid_pitch_q7, pid_yaw_q7,
        _reserved, flight_mode, status_flags,
    ) = fields

    # Derived engineering units.
    gyro_x_dps = gyro_x / GYRO_LSB_PER_DPS
    gyro_y_dps = gyro_y / GYRO_LSB_PER_DPS
    gyro_z_dps = gyro_z / GYRO_LSB_PER_DPS
    accel_x_g = accel_x / ACCEL_LSB_PER_G
    accel_y_g = accel_y / ACCEL_LSB_PER_G
    accel_z_g = accel_z / ACCEL_LSB_PER_G
    # IMU temperature formula per ALGORITHMS.md. The two supported IMUs
    # use different scale and offset values, so the active type has to
    # come from outside the log (the v1 schema does not embed it). The
    # MPU-6050 uses raw / 340 + 36.53; the MPU-6500 uses raw / 333.87
    # + 21.0. Pass --imu-type to switch.
    if g_imu_type == "mpu6050":
        imu_temp_c = imu_temp_raw / 340.0 + 36.53
    else:
        imu_temp_c = imu_temp_raw / 333.87 + 21.0
    baro_temp_c = baro_temp_cc / 100.0
    altitude_m = altitude_cm / 100.0

    # PID Q1.7 -> normalized.
    pid_roll = pid_roll_q7 / 127.0
    pid_pitch = pid_pitch_q7 / 127.0
    pid_yaw = pid_yaw_q7 / 127.0

    # Status flag booleans.
    flag_values = [(status_flags & mask) != 0 for _, mask in FLAG_BITS]

    return [
        schema_version, t_us, loop_period_us,
        gyro_x, gyro_y, gyro_z,
        gyro_x_dps, gyro_y_dps, gyro_z_dps,
        accel_x, accel_y, accel_z,
        accel_x_g, accel_y_g, accel_z_g,
        imu_temp_c,
        pressure_pa, baro_temp_c, altitude_m,
        quat_w, quat_x, quat_y, quat_z,
        roll_fwd_deg, pitch_fwd_deg, yaw_fwd_deg,
        roll_hover_deg, pitch_hover_deg, yaw_hover_deg,
        rc1, rc2, rc3, rc4, rc5, rc6,
        fader,
        motor1_us, motor2_us, servo_left_us, servo_right_us,
        pid_roll, pid_pitch, pid_yaw,
        flight_mode,
    ] + [1 if v else 0 for v in flag_values]


g_imu_type = "mpu6500"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", help="path to LOGnnnn.BIN")
    parser.add_argument(
        "--imu-type",
        choices=("mpu6500", "mpu6050"),
        default="mpu6500",
        help="IMU type used by the firmware that produced the log. v1 logs "
             "do not embed it; the user has to know. Default mpu6500.",
    )
    args = parser.parse_args()
    global g_imu_type
    g_imu_type = args.imu_type

    path = args.log
    out = csv.writer(sys.stdout)
    out.writerow(CSV_HEADER)

    record_count = 0
    schema_seen = None
    with open(path, "rb") as fh:
        while True:
            chunk = fh.read(RECORD_SIZE)
            if not chunk:
                break
            if len(chunk) != RECORD_SIZE:
                print(
                    f"warning: trailing partial record of {len(chunk)} bytes ignored",
                    file=sys.stderr,
                )
                break
            row = decode_record(chunk)
            schema = row[0]
            if schema_seen is None:
                schema_seen = schema
                if schema not in SUPPORTED_SCHEMA_VERSIONS:
                    print(
                        f"error: unsupported schema version 0x{schema:02x}. "
                        f"Supported: {SUPPORTED_SCHEMA_VERSIONS}",
                        file=sys.stderr,
                    )
                    sys.exit(1)
            out.writerow(row)
            record_count += 1

    print(
        f"decoded {record_count} records from {path} "
        f"(schema 0x{schema_seen:02x})" if schema_seen is not None
        else f"decoded 0 records from {path}",
        file=sys.stderr,
    )


if __name__ == "__main__":
    main()

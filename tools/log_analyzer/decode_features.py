#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Nitin Kumar
#
# CrowPilot telemetry log feature extractor.
#
# Reads a `.BIN` telemetry log and reduces it to a compact set of
# statistical features: loop-period health, per-axis gyro RMS, dominant
# oscillation frequency, controller and actuator saturation counts, and a
# flight-mode breakdown. The feature set is the input to the log analyzer
# (analyze.py), which turns it into a tuning diagnosis.
#
# The record schema follows internal-docs/TELEMETRY_FORMAT.md, the same
# layout decode_log.py uses.
#
# Usage as a CLI (emits the feature set as JSON):
#   python tools/log_analyzer/decode_features.py LOG0001.BIN
#
# Python 3.8+. No third-party dependencies.

import json
import math
import struct
import sys

# Record layout per TELEMETRY_FORMAT.md section 3, 109 bytes, little-endian.
RECORD_FMT = "<BQI3h3hhiii4f3f3f6Hf4H3bBBB"
RECORD_SIZE = struct.calcsize(RECORD_FMT)
assert RECORD_SIZE == 109, f"record size mismatch: got {RECORD_SIZE} expected 109"

SUPPORTED_SCHEMA_VERSIONS = (0x01,)

GYRO_LSB_PER_DPS = 131.0  # +-250 dps default range

# Status flag masks.
FLAG_ARMED = 0x01
FLAG_FAILSAFE = 0x02
FLAG_THROTTLE_CUT = 0x04

# Thresholds for the saturation and overrun counters.
LOOP_OVERRUN_US = 1100        # loop period above this counts as an overrun
PID_SATURATION = 0.95         # normalized PID output magnitude
MOTOR_SATURATION_US = 248     # OneShot125 commanded pulse close to the 250 max

# Goertzel oscillation scan band, in Hz. Tuning oscillation on a small
# airframe lives between a slow wobble and a fast buzz; scanning 1 to 40 Hz
# at a fine step covers it without a heavy full FFT.
OSC_SCAN_LO_HZ = 1.0
OSC_SCAN_HI_HZ = 40.0
OSC_SCAN_STEP_HZ = 0.5


class Record:
    """One decoded telemetry record, only the fields the analyzer uses."""

    __slots__ = (
        "t_us", "loop_period_us",
        "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
        "pid_roll", "pid_pitch", "pid_yaw",
        "motor1_us", "motor2_us",
        "fader", "flight_mode", "status_flags",
    )

    def __init__(self, raw):
        f = struct.unpack(RECORD_FMT, raw)
        self.t_us = f[1]
        self.loop_period_us = f[2]
        self.gyro_x_dps = f[3] / GYRO_LSB_PER_DPS
        self.gyro_y_dps = f[4] / GYRO_LSB_PER_DPS
        self.gyro_z_dps = f[5] / GYRO_LSB_PER_DPS
        self.pid_roll = f[34] / 127.0
        self.pid_pitch = f[35] / 127.0
        self.pid_yaw = f[36] / 127.0
        self.motor1_us = f[30]
        self.motor2_us = f[31]
        self.fader = f[29]
        self.flight_mode = f[38]
        self.status_flags = f[39]

    @property
    def armed(self):
        return (self.status_flags & FLAG_ARMED) != 0

    @property
    def failsafe(self):
        return (self.status_flags & FLAG_FAILSAFE) != 0


def read_records(path):
    """Yield Record objects from a .BIN log, validating the schema once."""
    schema_checked = False
    with open(path, "rb") as fh:
        while True:
            chunk = fh.read(RECORD_SIZE)
            if not chunk:
                break
            if len(chunk) != RECORD_SIZE:
                break  # trailing partial record, ignore
            if not schema_checked:
                schema = chunk[0]
                if schema not in SUPPORTED_SCHEMA_VERSIONS:
                    raise SystemExit(
                        f"error: unsupported schema version 0x{schema:02x}"
                    )
                schema_checked = True
            yield Record(chunk)


def _rms(values):
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def _mean(values):
    return sum(values) / len(values) if values else 0.0


def _percentile(sorted_values, fraction):
    if not sorted_values:
        return 0.0
    idx = min(len(sorted_values) - 1, int(fraction * len(sorted_values)))
    return sorted_values[idx]


def _goertzel_power(samples, freq_hz, sample_rate_hz):
    """Single-frequency power via the Goertzel algorithm."""
    omega = 2.0 * math.pi * freq_hz / sample_rate_hz
    coeff = 2.0 * math.cos(omega)
    s_prev = 0.0
    s_prev2 = 0.0
    for x in samples:
        s = x + coeff * s_prev - s_prev2
        s_prev2 = s_prev
        s_prev = s
    return s_prev2 * s_prev2 + s_prev * s_prev - coeff * s_prev * s_prev2


def _dominant_oscillation(samples, sample_rate_hz):
    """Scan the oscillation band, return (freq_hz, normalized_amplitude)."""
    if len(samples) < 32:
        return (0.0, 0.0)
    mean = _mean(samples)
    detrended = [x - mean for x in samples]
    n = len(detrended)
    best_freq = 0.0
    best_power = 0.0
    freq = OSC_SCAN_LO_HZ
    while freq <= OSC_SCAN_HI_HZ:
        if freq < sample_rate_hz / 2.0:
            power = _goertzel_power(detrended, freq, sample_rate_hz)
            if power > best_power:
                best_power = power
                best_freq = freq
        freq += OSC_SCAN_STEP_HZ
    # Goertzel power to single-sided amplitude estimate.
    amplitude = 2.0 * math.sqrt(best_power) / n
    return (best_freq, amplitude)


def extract_features(path):
    """Reduce a telemetry log to the feature dictionary used by analyze.py."""
    records = list(read_records(path))
    if not records:
        raise SystemExit(f"error: no records decoded from {path}")

    total = len(records)
    duration_s = (records[-1].t_us - records[0].t_us) / 1e6 if total > 1 else 0.0

    periods = [r.loop_period_us for r in records]
    periods_sorted = sorted(periods)
    overruns = sum(1 for p in periods if p > LOOP_OVERRUN_US)

    armed = [r for r in records if r.armed]
    armed_count = len(armed)

    # Effective sample rate from the armed segment, used for the FFT scan.
    if armed_count > 1:
        span_s = (armed[-1].t_us - armed[0].t_us) / 1e6
        sample_rate_hz = (armed_count - 1) / span_s if span_s > 0 else 100.0
    else:
        sample_rate_hz = 100.0

    gx = [r.gyro_x_dps for r in armed]
    gy = [r.gyro_y_dps for r in armed]
    gz = [r.gyro_z_dps for r in armed]

    osc_roll = _dominant_oscillation(gx, sample_rate_hz)
    osc_pitch = _dominant_oscillation(gy, sample_rate_hz)
    osc_yaw = _dominant_oscillation(gz, sample_rate_hz)

    pid_sat = {
        "roll": sum(1 for r in armed if abs(r.pid_roll) > PID_SATURATION),
        "pitch": sum(1 for r in armed if abs(r.pid_pitch) > PID_SATURATION),
        "yaw": sum(1 for r in armed if abs(r.pid_yaw) > PID_SATURATION),
    }
    motor_sat = sum(
        1 for r in armed
        if r.motor1_us >= MOTOR_SATURATION_US or r.motor2_us >= MOTOR_SATURATION_US
    )

    mode_counts = {0: 0, 1: 0, 2: 0}
    for r in records:
        if r.flight_mode in mode_counts:
            mode_counts[r.flight_mode] += 1

    # Failsafe activation edges.
    failsafe_events = 0
    prev_fs = False
    for r in records:
        if r.failsafe and not prev_fs:
            failsafe_events += 1
        prev_fs = r.failsafe

    def frac(count):
        return round(count / total, 4) if total else 0.0

    return {
        "log_path": path,
        "record_count": total,
        "duration_s": round(duration_s, 2),
        "sample_rate_hz": round(sample_rate_hz, 2),
        "loop_period": {
            "mean_us": round(_mean(periods), 1),
            "max_us": max(periods),
            "p99_us": _percentile(periods_sorted, 0.99),
            "overrun_count": overruns,
            "overrun_fraction": frac(overruns),
        },
        "armed": {
            "record_count": armed_count,
            "fraction": frac(armed_count),
        },
        "gyro_rms_dps": {
            "roll": round(_rms(gx), 3),
            "pitch": round(_rms(gy), 3),
            "yaw": round(_rms(gz), 3),
        },
        "oscillation": {
            "roll": {"freq_hz": round(osc_roll[0], 2),
                     "amplitude_dps": round(osc_roll[1], 3)},
            "pitch": {"freq_hz": round(osc_pitch[0], 2),
                      "amplitude_dps": round(osc_pitch[1], 3)},
            "yaw": {"freq_hz": round(osc_yaw[0], 2),
                    "amplitude_dps": round(osc_yaw[1], 3)},
        },
        "pid_saturation_count": pid_sat,
        "motor_saturation_count": motor_sat,
        "flight_mode_fraction": {
            "hover": frac(mode_counts[0]),
            "forward": frac(mode_counts[1]),
            "transitioning": frac(mode_counts[2]),
        },
        "failsafe_events": failsafe_events,
    }


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <log.BIN>", file=sys.stderr)
        sys.exit(2)
    features = extract_features(sys.argv[1])
    json.dump(features, sys.stdout, indent=2)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()

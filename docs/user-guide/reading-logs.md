<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Reading logs

CrowPilot writes a binary telemetry log to the SD card during flight, one record per tick at 100 Hz. After a flight, decode the log to CSV and inspect it. The log is the single best tool for understanding what the aircraft did.

## The decoder

Pull the SD card and run the decoder:

```bash
python tools/decode_log.py LOG0001.BIN > log0001.csv
```

The decoder is a plain Python 3 script with no third-party dependencies. It reads the binary records, checks the schema version, and writes a CSV with one row per record and a header naming every column.

## What is in the CSV

Each row is one telemetry record. The columns cover the loop timing, raw IMU readings, barometer, attitude estimate, receiver channels, controller output, motor and servo commands, the fader, and the state flags. The full field reference, with units and conversions, is in [reference/telemetry-format.md](../reference/telemetry-format.md).

## Plotting in a spreadsheet

The simplest approach, no coding required:

1. Open the CSV in LibreOffice Calc or Excel.
2. Select the columns you want (for example `t_us` and `gyro_x_dps`).
3. Insert a line chart.

This is enough for most tuning work. Plot one axis at a time against time.

## Plotting in Python

For repeatable analysis, use pandas and matplotlib:

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("log0001.csv")
df["t_s"] = (df["t_us"] - df["t_us"].iloc[0]) / 1e6

fig, ax = plt.subplots(3, 1, sharex=True)
ax[0].plot(df["t_s"], df[["gyro_x_dps", "gyro_y_dps", "gyro_z_dps"]])
ax[0].set_ylabel("body rate (dps)")
ax[1].plot(df["t_s"], df[["pid_roll", "pid_pitch", "pid_yaw"]])
ax[1].set_ylabel("PID output")
ax[2].plot(df["t_s"], df[["motor1_us", "motor2_us"]])
ax[2].set_ylabel("motor (us)")
ax[2].set_xlabel("time (s)")
plt.show()
```

## What good hover looks like

In a clean hover with sticks centered:

- `loop_period_us` stays near 1000 with only small spikes.
- `gyro_*_dps` shows low-amplitude noise around zero, no sustained oscillation.
- `pid_*` output is small and centered, well away from the ±1 saturation rails.
- `motor1_us` and `motor2_us` track each other closely and sit mid-range.
- `fader` is a steady 1.00.
- `armed = 1`, `failsafe_active = 0`, `throttle_cut = 0`.

## What problems look like

- **Oscillation.** A clean sine wave on a `gyro_*` axis at a few Hz or higher. The D term on that axis is too low, or the P term is too high. See [tuning.md](tuning.md).
- **Drift.** The aircraft slowly leans off on one axis with sticks centered. The I term on that axis is too low.
- **Saturation.** `pid_*` pinned at +1 or -1 for a sustained stretch, or `motor*_us` pinned at 250. The controller is asking for more than the airframe can deliver: under-powered propulsion, or a wrong gain ratio.
- **Loop overrun.** `loop_period_us` regularly spiking well above 1000. Something in the tick is taking too long, often SD writes or debug prints. Reduce the debug print rate.

The whole tuning workflow is built around reading these logs. See [tuning.md](tuning.md) for the procedure.

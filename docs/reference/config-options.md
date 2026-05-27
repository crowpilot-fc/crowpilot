<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Config options

Every compile-time selector and tunable constant lives in
[`src/Config.h`](https://github.com/crowpilot-fc/crowpilot/blob/main/src/Config.h).
This page is the field reference. The control-core values are provisional and
are replaced by bench tuning. The defaults below are the v1.0 shipping
values.

`Config.h` has two kinds of entry. Preprocessor selectors and feature flags
are `#define` macros, because the code tests them with `#if`. Typed tunables
are `constexpr` constants in namespace `cp`.

## Board

| Option | Default | Meaning |
|---|---|---|
| `BOARD` | `BOARD_WEACT_RP2350A_V10` | Selects the pin map. Other choices: `BOARD_WAVESHARE_RP2350_TINY`, and `BOARD_PICO2W` (RP2350A with an onboard WiFi radio). Overridable from a build flag (`-DBOARD=...`). |

## Build target

| Option | Default | Meaning |
|---|---|---|
| `BUILD_TARGET` | `BUILD_TARGET_NATIVE` | Selects the HAL. `BUILD_TARGET_SITL` builds the firmware as a host executable against the simulated HAL (see `sitl/`). `BUILD_TARGET_HIL` is scaffolded and not yet implemented. The SITL host build sets this on the compiler command line; the Arduino build keeps the native default. |

## Airframe

| Option | Default | Meaning |
|---|---|---|
| `AIRFRAME` | `AIRFRAME_PLANE_TWIN_CARGO` | Selects the mixer. The plane airframes (`AIRFRAME_PLANE_TWIN_CARGO`, the Caribou first-flight target, and `AIRFRAME_PLANE_SINGLE`) require `ENABLE_PLANE_STAB`. `AIRFRAME_TAILSITTER_BICOPTER` is the carried alternative. `AIRFRAME_QUAD_X` is a four-motor X quad with self-leveling angle and acro rate modes. The remaining frames (`AIRFRAME_HEX_X`, `AIRFRAME_TRICOPTER`, `AIRFRAME_TAILSITTER_QUAD`) are reserved and halt the build. |

## IMU

| Option | Default | Meaning |
|---|---|---|
| `IMU_TYPE` | `IMU_MPU6500` | IMU chip. The other choice is `IMU_MPU6050`. |
| `ENABLE_IMU_CALIBRATION` | `0` | When `1`, the boot calibration routine runs and halts. |
| `IMU_I2C_ADDR` | `0x68` | IMU I2C address. `0x69` when AD0 is high. |
| `IMU_GYRO_RANGE` | `3` | Raw FS_SEL field. `3` is plus or minus 2000 dps. |
| `IMU_ACCEL_RANGE` | `2` | Raw AFS_SEL field. `2` is plus or minus 8 g. |
| `GYRO_BIAS_X/Y/Z` | `0.0` | Gyro bias offsets in dps. Set from the calibration routine. |
| `ACC_BIAS_X/Y/Z` | `0.0` | Accelerometer bias offsets in g. Set from the calibration routine. |
| `IMU_CALIB_SAMPLE_COUNT` | `2000` | Samples averaged by the calibration routine. |

## Barometer

| Option | Default | Meaning |
|---|---|---|
| `BARO_TYPE` | `BARO_BMP388` | Barometer chip. Other choices are `BARO_BMP280` and `BARO_NONE`. |
| `BARO_I2C_ADDR` | `0x77` | Barometer I2C address. `0x76` when SDO is low. |
| `BARO_READ_INTERVAL_TICKS` | `20` | Loop ticks between barometer reads. At 1 kHz, 20 is a 50 Hz read rate. |

## Loop and bus

| Option | Default | Meaning |
|---|---|---|
| `LOOP_HZ` | `1000` | Super-loop rate. |
| `I2C_BUS_HZ` | `400000` | I2C0 clock. |

## Receiver

| Option | Default | Meaning |
|---|---|---|
| `RX_PROTOCOL` | `RX_SBUS` | Receiver protocol. `RX_SBUS` (PIO, inverted) or `RX_CRSF` (Crossfire / ELRS, a 420 kbaud UART on the receiver pin). `RX_PPM` and `RX_PWM` are reserved and halt the build. |
| `RX_SBUS_INVERTED` | `1` | When `1`, the PIO program reads SBUS inverted with no external inverter. |

## Pilot channel map

| Option | Default | Meaning |
|---|---|---|
| `CHANNEL_ROLL` | `1` | Roll channel, 1-based. AETR ch1. |
| `CHANNEL_PITCH` | `2` | Pitch channel. AETR ch2. |
| `CHANNEL_THROTTLE` | `3` | Throttle channel. AETR ch3. |
| `CHANNEL_YAW` | `4` | Yaw channel. AETR ch4. |
| `CHANNEL_ARM` | `16` | Arm switch channel. Top SBUS channel, dedicated. |
| `CHANNEL_TRANSITION` | `15` | Transition channel. Tailsitter fader only; parked on a high channel for the plane build. |
| `RC_MIN_US` / `RC_MID_US` / `RC_MAX_US` | `1000` / `1500` / `2000` | RC pulse widths for a low, centered, and high stick. |

## Failsafe

| Option | Default | Meaning |
|---|---|---|
| `FS_THROTTLE_US` | `1300` | Failsafe throttle, below hover for a gentle descent. |
| `FS_ROLL_US` / `FS_PITCH_US` / `FS_YAW_US` | `1500` | Failsafe sticks, centered. |
| `FS_ARM_US` | `1000` | Failsafe arm value, armed so the descent stays powered. |
| `FS_LINK_TIMEOUT_US` | `100000` | Lost-link timeout in microseconds. |

## Telemetry

| Option | Default | Meaning |
|---|---|---|
| `ENABLE_TELEMETRY_LOG` | `1` | Enables the SD card logger. |
| `TELEMETRY_LOG_INTERVAL_TICKS` | `10` | Loop ticks between records. At 1 kHz, 10 is a 100 Hz log rate. |
| `TELEMETRY_LOG_MAX_BYTES` | `16 MiB` | Log file size at which a new file is started. |

## Parameters and live tuning

| Option | Default | Meaning |
|---|---|---|
| `ENABLE_PARAM_PERSIST` | `1` | Enables flash persistence of the parameter registry. |
| `ENABLE_LIVE_TUNE` | `1` | Enables transmitter-channel live tuning. |
| `ENABLE_CONFIG_CLI` | `1` | Enables the `cp` serial command interface the browser-based configurator speaks. A bench tool; a flight-only build can drop it. |
| `ENABLE_COMPANION_CLI` | `1` | Also runs the `cp` interface on the companion UART (native builds), so an ESP WiFi module can bridge it to a phone. |
| `ENABLE_ONBOARD_WIFI` | `BOARD_HAS_WIFI` | No-ESP path for boards with an integrated radio (e.g. `BOARD_PICO2W`). The flight controller raises its own WiFi access point and serves the companion UI from core1. Defaults on for a WiFi board, errors on a board without one. Validate loop jitter on the bench first. |
| `LIVE_TUNE_CH_KP` | `11` | Channel for the live P-gain knob. |
| `LIVE_TUNE_CH_KD` | `12` | Channel for the live D-gain knob. |
| `LIVE_TUNE_RANGE` | `0.5` | Fractional tuning range, plus or minus. |

## Attitude estimation

| Option | Default | Meaning |
|---|---|---|
| `MADGWICK_BETA` | `0.10` | Madgwick filter gain. Higher tracks the accelerometer faster and noisier. |
| `GYRO_LPF_CUTOFF_HZ` | `90.0` | First-order low-pass on the gyro, applied once in the estimator. 0 disables it. Lower filters harder but adds phase lag. |
| `GYRO_NOTCH_CENTER_HZ` | `0.0` | Fixed gyro notch center, at a measured vibration peak. 0 disables it (the default until a peak is known). |
| `GYRO_NOTCH_Q` | `3.0` | Fixed notch width. Higher is narrower, with less phase lag away from the peak. |
| `ENABLE_DYNAMIC_NOTCH` | `0` | When `1`, the gyro notch tracks the motor frequency from bidirectional DShot instead of sitting at `GYRO_NOTCH_CENTER_HZ`. Needs `ENABLE_DSHOT_BIDIR`. |
| `DYN_NOTCH_MIN_HZ` / `DYN_NOTCH_MAX_HZ` | `80.0` / `500.0` | Clamp on the tracked notch center. The floor ignores ground idle, the ceiling stays below the loop Nyquist. |
| `DYN_NOTCH_Q` | `3.0` | Width of the tracking notch. |
| `DYN_NOTCH_MAX_SLEW_HZ_PER_S` | `2000.0` | How fast the tracked center may move, so the notch glides rather than jumps. |
| `DYN_NOTCH_UPDATE_DIV` | `4` | Retune the notch every Nth loop tick. |

## Transition and pilot setpoints

| Option | Default | Meaning |
|---|---|---|
| `TRANSITION_SLEW_RATE` | `0.33` | Fader slew rate, fader units per second (about a 3 s transition). |
| `MAX_ROLL_ANGLE_DEG` | `35.0` | Roll attitude setpoint at full stick. |
| `MAX_PITCH_ANGLE_DEG` | `35.0` | Pitch attitude setpoint at full stick. |
| `MAX_YAW_RATE_DPS` | `120.0` | Yaw-rate setpoint at full stick. |

## Tailsitter PID gains

The per-axis gains are runtime parameters. The `Config.h` values are the
defaults that seed the parameter registry. There is a hover set and a
forward-flight set for the proportional and derivative gains, and a shared
integral set. All ship provisional.

| Option | Default | Meaning |
|---|---|---|
| `Kp_*_hover` / `Kd_*_hover` | `Kp 0.030` roll-pitch, `0.008` yaw. `Kd 0.004` roll-pitch, `0.002` yaw. | Hover gains. Roll and pitch act on a degree error, yaw on a rate error. |
| `Kp_*_ff` / `Kd_*_ff` | `Kp 0.025` roll-pitch, `0.006` yaw. `Kd 0.003` roll-pitch, `0.0015` yaw. | Forward-flight gains. |
| `Ki_roll` / `Ki_pitch` / `Ki_yaw` | `0.0` | Integral gains. Tuned last. |
| `PID_INTEGRAL_LIMIT` | `0.5` | Anti-windup limit on the integral term, as a fraction of the output range. |

## Tailsitter mixer

| Option | Default | Meaning |
|---|---|---|
| `ELEVON_LEFT_TRIM` / `ELEVON_RIGHT_TRIM` | `0.5` | Per-elevon mechanical neutral. |
| `ELEVON_ROLL_GAIN` / `ELEVON_PITCH_GAIN` | `0.5` | Elevon travel per unit axis demand. |
| `ELEVON_FF_GAIN` | `0.2` | Forward-flight stick feedforward on the elevons. |
| `MOTOR_YAW_GAIN` | `0.3` | Differential thrust per unit yaw demand. |

## ESC, servo, and arming

| Option | Default | Meaning |
|---|---|---|
| `MOTOR_PROTOCOL` | `MOTOR_PROTOCOL_PWM` | Motor signal protocol. `MOTOR_PROTOCOL_PWM` is standard 1000-2000 us hobby PWM. `MOTOR_PROTOCOL_ONESHOT125` is the 125-250 us synchronous burst. `MOTOR_PROTOCOL_DSHOT300` and `MOTOR_PROTOCOL_DSHOT600` are the digital 300 and 600 kbit/s frame protocols, clocked out in PIO (native builds only). |
| `ENABLE_ESC_CALIBRATION` | `0` | When `1`, the boot ESC calibration routine runs and halts. Meaningless for DShot, which has no analog endpoints. |
| `ESC_MAX_PULSE_US` / `ESC_IDLE_PULSE_US` | `2000` / `1000` | Motor full and zero throttle. Values depend on `MOTOR_PROTOCOL` (`250` / `125` for OneShot125). For DShot these microsecond bounds map onto the 48-2047 throttle range. |
| `ESC_DISARM_PULSE_US` | `1000` | Disarmed motor pulse. The motor-stopped width (`120` for OneShot125, `0` for DShot which maps to the motor-stop command, both below the valid running range). |
| `DSHOT_BITRATE_HZ` | (per protocol) | DShot line rate, `300000` or `600000`. Defined only for the DShot protocols and used to set the PIO clock divider. |
| `ENABLE_DSHOT_BIDIR` | `0` | When `1`, bidirectional DShot: the ESC replies with eRPM after each frame, feeding the dynamic notch. Needs a DShot protocol and a telemetry-capable ESC. The receive timing is bench-tuned (see the wiring guide). |
| `MOTOR_POLE_PAIRS` | `7` | Magnet pole pairs of the motors, for converting bidirectional-DShot eRPM to mechanical RPM. A 14-magnet outrunner has 7. |
| `ARM_THROTTLE_MAX_US` | `1050` | Throttle-idle gate for arming. |
| `SERVO_MIN_US` / `SERVO_MAX_US` | `1000` / `2000` | Servo PWM endpoints. |

## Plane stabilization

These apply to the fixed-wing airframes. They are inactive for the
tailsitter.

| Option | Default | Meaning |
|---|---|---|
| `ENABLE_PLANE_STAB` | `1` | Enables the fixed-wing stabilizer. Required by the plane airframes. |
| `ENABLE_ALT_HOLD` | `0` | Enables barometric altitude hold. |
| `ENABLE_DIFF_THRUST_YAW` | `0` | Enables differential-thrust yaw on the twin-engine plane. |
| `CHANNEL_STAB` / `CHANNEL_ALT_HOLD` | `14` / `13` | Stabilization and altitude-hold switch channels. Parked on high SBUS channels so they cannot collide with TX-side primaries or aux. |
| `KP_STAB_ROLL` ... `KD_STAB_YAW` | provisional | Wing-leveler, pitch-hold, and yaw-damper gains. |
| `STAB_OUTPUT_SCALE` | `1.0` | Scales each stabilizer axis output. |
| `ALT_CLIMB_FILTER_ALPHA`, `KP_ALT`, `KD_ALT` | provisional | Altitude-hold filter and gains. |
| `AILERON_TRAVEL`, `ELEVATOR_TRAVEL`, `RUDDER_TRAVEL` | `0.5` | Control-surface travel. |
| `DIFF_THRUST_GAIN` | `0.2` | Differential-thrust yaw gain. |

## User hook

| Option | Default | Meaning |
|---|---|---|
| `ENABLE_USER_HOOK` | `0` | Enables the user extension hook. |
| `USER_HOOK_RATE_DIV` | `20` | Loop ticks between user-hook calls. |
| `USER_HOOK_BUDGET_US` | `150` | Soft time budget for the user hook. |
| `USER_HOOK_HARD_LIMIT_US` | `250` | Hard time limit for the user hook. |

## Debug

Each `DEBUG_PRINT_*` flag is an independent compile-time gate. `DEV` is
on by default; the rest are bench-diagnostic streams enabled one at a
time. Every print is also gated at runtime on a live USB serial
connection.

| Option | Default | Meaning |
|---|---|---|
| `DEBUG_PRINT_DEV` | `1` | One-line status summary and the loop-period report. |
| `DEBUG_PRINT_IMU` | `0` | Accelerometer, gyro, and temperature stream. |
| `DEBUG_PRINT_RX` | `0` | Receiver channel stream. |
| `DEBUG_PRINT_FAILSAFE` | `0` | Failsafe state and effective channels. |
| `DEBUG_PRINT_ATTITUDE` | `0` | Attitude estimate and quaternion. |
| `DEBUG_PRINT_MODE` | `0` | Transition fader and flight mode. |
| `DEBUG_PRINT_MIXER` | `0` | Mixer motor and servo output. |
| `DEBUG_PRINT_USER_HOOK` | `0` | User-hook timing stats. |
| `DEBUG_DEV_INTERVAL_TICKS` | `200` | Ticks between DEV summary lines. |
| `DEBUG_STREAM_INTERVAL_TICKS` | `100` | Ticks between diagnostic stream lines. |
| `DEBUG_LOOP_REPORT_TICKS` | `1000` | Ticks between loop-period reports. |

## Minimal IMU-only build

CrowPilot scales down to a bare flight controller: a small RP2350 board and an MPU IMU, with no SD card, no barometer, and no WiFi. This suits a simple fixed-wing plane on a 5-channel transmitter, and compiles to about 5 percent of flash.

Hardware: a Waveshare RP2350-Tiny (or any RP2350 board) plus an MPU-6500 or MPU-6050 on I2C0 (GP4/GP5). SBUS receiver on GP1. One ESC on GP10 and the control-surface servos on the servo pins. No SD module, no barometer, no ESP.

Set these in `Config.h`:

| Option | Minimal value | Why |
|---|---|---|
| `BOARD` | `BOARD_WAVESHARE_RP2350_TINY` | Smallest board. |
| `AIRFRAME` | `AIRFRAME_PLANE_SINGLE` | One motor plus aileron, elevator, rudder. |
| `BARO_TYPE` | `BARO_NONE` | No barometer. |
| `ENABLE_TELEMETRY_LOG` | `0` | No SD logging. |
| `ENABLE_PARAM_PERSIST` | `0` | No flash parameter store. |
| `ENABLE_LIVE_TUNE` | `0` | No spare channels for tuning knobs. |
| `ENABLE_COMPANION_CLI` | `0` | No ESP companion. |
| `ENABLE_ALT_HOLD` | `0` | Needs a barometer. |
| `CHANNEL_ARM` | `5` | Arm on the fifth channel of a 5-channel TX. |

`ENABLE_CONFIG_CLI` can stay `1`: the USB `cp` interface still tunes over a cable with no extra hardware. With persistence off, `cp save` and `cp load` return an error and gains revert to the `Config.h` defaults on reboot.

Channel map for a 5-channel transmitter (AETR plus arm):

| Channel | Function |
|---|---|
| 1 | Roll (aileron) |
| 2 | Pitch (elevator) |
| 3 | Throttle |
| 4 | Yaw (rudder) |
| 5 | Arm switch |

Leave the stabilizer channel (`CHANNEL_STAB`) unassigned on a 5-channel radio. An undriven channel sits at center, which the firmware reads as stabilized, so the wing leveler stays on. Assign a sixth channel to `CHANNEL_STAB` if you want a manual-passthrough switch.

The `AIRFRAME_QUAD_X` four-motor quad is supported, with self-leveling angle and acro rate modes selected by the stabilizer channel. It is not a 5-channel airframe: it needs the arm and mode switches on top of the four primaries. The other multirotor frames (`AIRFRAME_HEX_X`, `AIRFRAME_TRICOPTER`, `AIRFRAME_TAILSITTER_QUAD`) are reserved and halt the build.

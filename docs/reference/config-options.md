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
| `BOARD` | `BOARD_WAVESHARE_RP2350_TINY` | Selects the pin map. The other choice is `BOARD_WEACT_RP2350A_V10`. |

## Build target

| Option | Default | Meaning |
|---|---|---|
| `BUILD_TARGET` | `BUILD_TARGET_NATIVE` | Selects the HAL. `BUILD_TARGET_HIL` and `BUILD_TARGET_SITL` are scaffolded and not yet implemented. |

## Airframe

| Option | Default | Meaning |
|---|---|---|
| `AIRFRAME` | `AIRFRAME_TAILSITTER_BICOPTER` | Selects the mixer. `AIRFRAME_PLANE_TWIN_CARGO` and `AIRFRAME_PLANE_SINGLE` require `ENABLE_PLANE_STAB`. The remaining values are reserved and halt the build. |

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
| `RX_PROTOCOL` | `RX_SBUS` | Receiver protocol. `RX_PPM`, `RX_PWM`, and `RX_CRSF` are reserved. |
| `RX_SBUS_INVERTED` | `1` | When `1`, the PIO program reads SBUS inverted with no external inverter. |

## Pilot channel map

| Option | Default | Meaning |
|---|---|---|
| `CHANNEL_THROTTLE` | `1` | Throttle channel, 1-based. |
| `CHANNEL_ROLL` | `2` | Roll channel. |
| `CHANNEL_PITCH` | `3` | Pitch channel. |
| `CHANNEL_YAW` | `4` | Yaw channel. |
| `CHANNEL_ARM` | `5` | Arm switch channel. |
| `CHANNEL_TRANSITION` | `6` | Transition channel. |
| `RC_MIN_US` / `RC_MID_US` / `RC_MAX_US` | `1000` / `1500` / `2000` | RC pulse widths for a low, centered, and high stick. |

## Failsafe

| Option | Default | Meaning |
|---|---|---|
| `FS_CH1_THROTTLE` | `1350` | Failsafe throttle, below hover for a gentle descent. |
| `FS_CH2_ROLL` / `FS_CH3_PITCH` / `FS_CH4_YAW` | `1500` | Failsafe sticks, centered. |
| `FS_CH5_ARM` | `1000` | Failsafe arm channel, armed so the descent stays powered. |
| `FS_CH6_TRANSITION` | `1000` | Failsafe transition channel, hover end. |
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
| `LIVE_TUNE_CH_KP` | `9` | Channel for the live P-gain knob. |
| `LIVE_TUNE_CH_KD` | `10` | Channel for the live D-gain knob. |
| `LIVE_TUNE_RANGE` | `0.5` | Fractional tuning range, plus or minus. |

## Attitude estimation

| Option | Default | Meaning |
|---|---|---|
| `MADGWICK_BETA` | `0.10` | Madgwick filter gain. Higher tracks the accelerometer faster and noisier. |

## Transition and pilot setpoints

| Option | Default | Meaning |
|---|---|---|
| `TRANSITION_SLEW_RATE` | `0.5` | Fader slew rate, fader units per second. |
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
| `Kp_roll_hover` ... `Kd_yaw_hover` | `0.50` / `0.05` | Hover proportional and derivative gains. |
| `Kp_roll_ff` ... `Kd_yaw_ff` | `0.40` / `0.04` | Forward-flight proportional and derivative gains. |
| `Ki_roll` / `Ki_pitch` / `Ki_yaw` | `0.0` | Integral gains. Tuned last. |
| `PID_INTEGRAL_LIMIT` | `50.0` | Anti-windup clamp on each integrator. |

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
| `ENABLE_ESC_CALIBRATION` | `0` | When `1`, the boot ESC calibration routine runs and halts. |
| `ESC_MAX_PULSE_US` / `ESC_IDLE_PULSE_US` | `250` / `125` | OneShot125 full and zero throttle. |
| `ESC_DISARM_PULSE_US` | `120` | Disarmed motor pulse, below the valid range. |
| `ARM_THROTTLE_MAX_US` | `1050` | Throttle-idle gate for arming. |
| `SERVO_MIN_US` / `SERVO_MAX_US` | `1000` / `2000` | Servo PWM endpoints. |

## Plane stabilization

These apply to the fixed-wing airframes. They are inactive for the
tailsitter.

| Option | Default | Meaning |
|---|---|---|
| `ENABLE_PLANE_STAB` | `0` | Enables the fixed-wing stabilizer. Required by the plane airframes. |
| `ENABLE_ALT_HOLD` | `0` | Enables barometric altitude hold. |
| `ENABLE_DIFF_THRUST_YAW` | `0` | Enables differential-thrust yaw on the twin-engine plane. |
| `CHANNEL_STAB` / `CHANNEL_ALT_HOLD` | `7` / `8` | Stabilization and altitude-hold switch channels. |
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

<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Algorithms

This page describes the control-core algorithms: attitude estimation, the
transition reference, desired-state generation, stabilization, mixing, and
the supporting calibration and output code. Every tuning constant named here
is provisional and is set by bench tuning.

## Attitude estimation

The estimator fuses the three-axis gyroscope and three-axis accelerometer
with a Madgwick gradient-descent orientation filter, the six-degree-of-freedom
variant with no magnetometer. The state is a unit quaternion representing the
rotation from the body frame to the world frame.

Each step computes the quaternion rate of change from the gyroscope, then
applies one gradient-descent correction toward the orientation that best
explains the measured gravity direction, integrates, and renormalizes the
quaternion. The filter has a single gain, `MADGWICK_BETA`. A higher beta
tracks the accelerometer faster and is noisier. A lower beta trusts the gyro
longer and drifts more.

The first update seeds the quaternion from the accelerometer tilt, so the
estimate starts level instead of converging over several seconds. A
non-finite or degenerate quaternion norm resets the state to identity, which
catches numerical drift.

Without a magnetometer, absolute heading is not observable, so yaw drifts
slowly. Gyro bias calibration bounds that drift.

### Body frame

One fixed right-handed body frame is used everywhere: x out the nose, y out
the right wing, z completing the triad so a level airframe at rest reads one g
on the z accelerometer. There is one physical body frame regardless of flight
regime, so the body-rate signal needs no per-regime remapping.

### Gyro filtering

The gyro is filtered once, in the estimator, so the Madgwick integration and
every controller see the same body-rate signal. Each axis runs an optional
notch then a first-order low-pass (`GYRO_NOTCH_*` then `GYRO_LPF_CUTOFF_HZ`).
The low-pass keeps motor and propeller vibration off the derivative term,
which amplifies high-frequency noise the most. The notch removes a single
narrow vibration peak with less broadband phase lag than the low-pass.

The motor vibration peak is the motor rotation frequency, which moves with
throttle, so a fixed notch only catches one RPM. With `ENABLE_DYNAMIC_NOTCH`
the notch tracks it. The dynamic-notch module reads the per-motor electrical
RPM that bidirectional DShot reports through the HAL, converts it to a mean
mechanical frequency through `MOTOR_POLE_PAIRS`, clamps it to a configured
band, slew-rate limits the center, and retunes the notch every few ticks
through a coefficient update that preserves the filter state so the move is
glitch-free. With no eRPM telemetry the notch falls back to the fixed center.
See [Motor output](#motor-output) for the bidirectional DShot side.

## The attitude reference across the transition

A tailsitter flies at roughly zero degrees pitch in forward flight and roughly
ninety degrees pitch in hover. Any single Euler-angle convention places a
gimbal-lock singularity inside that envelope. The quaternion state is
singularity-free, so the estimate itself is always well-conditioned.

The stabilizer input is a quaternion attitude error. `errorToReference` builds
the orientation the airframe should hold, the nose-up hover attitude rotated
toward forward-flight level by the transition fader, with the pilot roll and
pitch setpoints applied as body-frame offsets. It then returns the rotation
from the current estimate to that reference as a small-angle body-frame vector
in degrees. That signal is continuous everywhere from hover to forward flight.

Two Euler views, `eulerForwardFlight` and `eulerHover`, are derived from the
same quaternion for telemetry, display, and the fixed-wing subsystem. Each is
singular in the opposite regime and is never used to close the tailsitter
control loop.

## Transition management

A transmitter channel commands the transition. The raw channel maps to a
normalized command, one at the hover end and zero at the forward end. The
value the control core acts on, the fader, is rate-limited: it slews toward
the command at `TRANSITION_SLEW_RATE` fader units per second. The airframe
cannot rotate ninety degrees instantly, so an instantaneous reference change
would command an impossible maneuver. The slew makes even a snapped switch
produce a controllable transition.

A flight-mode enumeration, hover, transitioning, or forward, is derived from
the fader for telemetry and display. It does not feed back into the control
law, which stays continuous in the fader value.

## Desired-state generation

The pilot stick channels map to control setpoints. Throttle becomes a
normalized thrust command. The roll and pitch sticks become attitude
setpoints scaled by `MAX_ROLL_ANGLE_DEG` and `MAX_PITCH_ANGLE_DEG`. The yaw
stick becomes a yaw-rate setpoint scaled by `MAX_YAW_RATE_DPS`. The unshaped
stick deflections are also exposed for the mixer feedforward. The mapping is
the same in both regimes. The hover and forward dynamics difference is
handled by the gain schedule, not by reshaping the setpoints.

## Stabilization

One PID controller per axis drives the errors toward zero. Roll and pitch
stabilize on the quaternion attitude error. Yaw stabilizes on a rate error,
the commanded yaw rate minus the measured yaw rate, because yaw has no
absolute reference without a magnetometer.

All three axes share one PID form:

```
demand = Kp * error + Ki * integral(error) - Kd * body_rate
```

The derivative term is taken from the measured body rate, not from the error,
so a stepped setpoint does not produce a derivative kick.

The gains are regime-scheduled. Hover and forward flight have substantially
different dynamics, control authority from differential thrust and propeller
wash versus from aerodynamic surfaces at airspeed. Each axis interpolates
continuously between a hover gain set and a forward-flight gain set by the
transition fader. The blended proportional and derivative gains are then
scaled by the live-tune multipliers.

The integrator is clamped to `PID_INTEGRAL_LIMIT` for anti-windup and is held
at zero while the aircraft is not flying, so it does not wind up on the
bench. Each axis demand is clamped to the normalized range from minus one to
plus one.

## Mixing

The tailsitter mixer allocates throttle and the three stabilized axis demands
to the two motors and two elevons. Two motors sit on the wing, separated along
the body y-axis, with one elevon behind each motor in its propeller wash.

From that geometry the effector-to-moment map is:

- Common thrust gives the body x force, lift in hover and thrust in forward
  flight.
- Differential thrust gives the body z moment, yaw.
- Symmetric elevon deflection gives the body y moment, pitch.
- Differential elevon deflection gives the body x moment, roll.

The map is geometric and the same in both regimes. The regime difference is
in aerodynamic authority, handled by the gain schedule. The one allocation
term that changes with regime is a direct-stick feedforward on the elevons,
faded in toward forward flight. Every actuator output is clamped to its valid
range before it leaves the mixer.

## IMU bias calibration

A MEMS gyroscope and accelerometer each have a fixed zero offset. The
calibration routine, run at boot when `ENABLE_IMU_CALIBRATION` is set,
averages `IMU_CALIB_SAMPLE_COUNT` raw samples with the airframe held
stationary and level. The three gyro axes and the two horizontal
accelerometer axes average to the bias directly. The vertical accelerometer
axis carries one g of gravity, which is removed. The six values are printed
for pasting into `Config.h`.

## Motor output

The control core works in microseconds for every motor protocol. The mixer
produces a normalized motor command, the output stage maps it to a pulse
width in the configured range, and the native HAL turns that into the signal
`MOTOR_PROTOCOL` selects. When disarmed every protocol emits its motor-stop
value, so the ESCs stay silent regardless.

Standard PWM (the default) drives the motor pins through the Servo library at
1000 to 2000 microseconds on a 50 Hz schedule, the same path as the servos.
The disarm value is 1000 microseconds.

OneShot125 is a pulse of 125 to 250 microseconds repeated once per loop tick.
The output stage buffers the per-motor pulse widths and emits them as one
synchronous burst: all motor pins are raised together and each is lowered
when its own pulse width has elapsed. When disarmed the motors emit a
sub-valid pulse below 125 microseconds.

DShot300 and DShot600 are digital. Each motor update is a 16-bit frame: an
11-bit throttle value (0 stops the motor, 48 to 2047 is the throttle band), a
telemetry-request bit, and a 4-bit CRC. The native HAL converts the motor
pulse to a throttle value, builds the frame, and clocks it out bit by bit
from a PIO state machine, one per motor on PIO1 (the SBUS receiver owns
PIO0). Each bit is a pulse whose high time encodes its value, the same
pulse-width scheme as WS2812, at 300 or 600 kbit/s. The line idles low
between frames, so before the first frame and whenever disarmed the ESCs see
the motor-stop frame. The frame builder is plain integer code in
`src/libs/Dshot.*` and is unit-tested on the host; the PIO timing is verified
on the bench with a logic analyzer or a real ESC.

Bidirectional DShot (`ENABLE_DSHOT_BIDIR`) makes the ESC report its electrical
RPM. The signaling is inverted (line idles high) and the command CRC is
inverted, which is what requests the reply. The transmit program is the same
bit scheme with flipped polarity on PIO1. After each frame the HAL releases
the pin and a sampler on PIO2 oversamples the reply, which the HAL reconstructs
into the 21-bit GCR frame and decodes with the host-tested `decodeErpm` (GCR
quintet map, 4-bit checksum, exponent-mantissa period to eRPM). That eRPM
feeds the dynamic notch. The decode math is unit-tested, but the receive
timing (turnaround, telemetry bit rate, oversample) is bench-tuned and
unverified in v1, so the path is off by default.

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// Attitude estimation for the tailsitter. See
// docs/developer-guide/algorithms.md.
//
// The estimator fuses the three-axis gyroscope and three-axis
// accelerometer with a Madgwick gradient-descent orientation filter, the
// six-degree-of-freedom variant (no magnetometer). The state is a unit
// quaternion. Absolute heading is not observable without a magnetometer,
// so yaw drifts slowly. That is expected for v1.
//
// Body-frame convention. One fixed right-handed body frame is used
// everywhere: x out the nose, z pointing up so a level airframe at
// rest reads +1 g on the z accelerometer, and y completing the
// right-handed triad. With x out the nose and z up, that right-handed
// y points out the left wing. The gyroscope reports rates about these
// same axes. There is one physical body frame regardless of flight
// regime, so the body-rate signal needs no per-regime remapping.
//
// The transition problem (spec section 5). A tailsitter flies at roughly
// 0 degrees pitch in forward flight and roughly 90 degrees pitch in
// hover. Any single Euler convention places a gimbal-lock singularity
// inside that envelope. The quaternion state is singularity-free, so the
// estimate itself is always well-conditioned. The stabilizer input is
// errorTo(), a quaternion attitude error that is continuous everywhere
// from hover to forward flight. The two Euler views below are derived
// quantities for telemetry, display, and the fixed-wing subsystem only.
// They are each singular in the opposite regime and are never used to
// close the tailsitter control loop.

namespace cp::estimation::attitude {

// Unit quaternion, rotation from the body frame to the world frame.
// w is the scalar part.
struct Quaternion {
  float w;
  float x;
  float y;
  float z;
};

// An angle triple in degrees. Used for the Euler views and for the
// attitude-error vector.
struct Euler {
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
};

// Body angular rates in degrees per second, in the fixed body frame.
struct BodyRates {
  float roll_dps;
  float pitch_dps;
  float yaw_dps;
};

// Reset the estimator. The quaternion is marked unseeded; the first
// update() call seeds it from the accelerometer so the estimate
// converges to level immediately rather than over several seconds.
void init();

// One AHRS step. gx/gy/gz are the calibrated gyro in degrees per second
// and ax/ay/az the calibrated accelerometer in g, both in the body
// frame. dt_s is the elapsed loop period in seconds.
void update(float gx_dps, float gy_dps, float gz_dps,
            float ax_g, float ay_g, float az_g,
            float dt_s);

// The current orientation estimate. Always a unit quaternion.
const Quaternion& quaternion();

// The most recent body rates (the calibrated gyro sample).
const BodyRates& bodyRates();

// Forward-flight Euler view. Standard aircraft 3-2-1 angles extracted
// from the estimate. Zero when the airframe is wings-level in forward
// flight. Well-conditioned in the forward regime, singular at the hover
// orientation.
Euler eulerForwardFlight();

// Hover Euler view. The estimate expressed relative to the nose-up
// hover reference. Zero when the airframe hovers nose-up and wings
// level. Well-conditioned in the hover regime, singular at the forward
// orientation.
Euler eulerHover();

// Singularity-free attitude error. Returns the rotation that carries
// the current estimate onto `reference`, as a small-angle body-frame
// vector in degrees (roll about x, pitch about y, yaw about z). The
// shortest rotation is always chosen. Continuous everywhere from hover
// to forward flight.
Euler errorTo(const Quaternion& reference);

// The attitude reference across the transition, per spec section 5.
// Builds the orientation the airframe should hold, then returns the
// errorTo() of it. The reference is the nose-up hover attitude rotated
// toward forward-flight level by `fader` (1.0 hover to 0.0 forward),
// with the pilot roll and pitch setpoints applied as body-frame
// offsets. The roll and pitch components of the result drive the
// stabilizer. The yaw component is informational, since yaw has no
// absolute reference without a magnetometer and is rate-controlled.
Euler errorToReference(float fader,
                       float roll_setpoint_deg,
                       float pitch_setpoint_deg);

}  // namespace cp::estimation::attitude

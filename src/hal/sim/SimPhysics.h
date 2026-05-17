// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_HIL || BUILD_TARGET == BUILD_TARGET_SITL

// Rotational rigid-body model of the tailsitter, for closed-loop SITL.
// The sim HAL drives it: OutSim feeds the actuator commands and steps it,
// ImuSim reads the resulting attitude back as gyro and accelerometer
// samples. It models attitude dynamics only, enough to exercise the
// attitude control law. Translational motion and altitude are not
// modelled. The airframe constants are plausible estimates for a 1 kg
// tailsitter, not measured values.

namespace cp::sim {

// Reset the model to a near-hover attitude with a small offset, so the
// controller has a disturbance to correct once it is armed.
void physics_init();

// Latch one actuator's normalised command. Motor thrust is 0..1, servo
// deflection is -1..+1 about centre. idx selects the motor or servo.
void physics_set_motor(int idx, float thrust_norm);
void physics_set_servo(int idx, float deflection);

// Integrate the rigid-body dynamics by dt seconds using the latched
// actuator commands.
void physics_step(float dt_s);

// Body angular rates, degrees per second.
void physics_gyro_dps(float& gx, float& gy, float& gz);

// Specific force in the body frame, g. In this attitude-only model that
// is the gravity-up direction expressed in body axes, magnitude 1.
void physics_accel_g(float& ax, float& ay, float& az);

// Tilt of the body x-axis from world-up, degrees. Zero in true hover.
// Ground truth for judging the controller: a converging loop drives the
// tilt and the body rates toward zero.
float physics_tilt_deg();

}  // namespace cp::sim

#endif  // SITL or HIL

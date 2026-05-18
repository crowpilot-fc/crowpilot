// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_HIL || BUILD_TARGET == BUILD_TARGET_SITL

// Rigid-body model behind the closed-loop SITL. The sim HAL drives it:
// OutSim feeds the actuator commands and steps it, ImuSim reads the
// resulting attitude back as gyro and accelerometer samples. It models
// rotational dynamics only, enough to exercise the attitude control law.
// Translation and altitude are not modelled.
//
// Two implementations satisfy this API, selected by AIRFRAME:
// SimPhysicsTailsitter.cpp for the tailsitter, SimPhysicsPlane.cpp for
// the fixed-wing airframes. The airframe constants in each are plausible
// estimates, not measured values.

namespace cp::sim {

// Reset the model to its reference attitude with a small disturbance, so
// the controller has something to correct once it is armed.
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

// Ground-truth attitude for judging the controller: roll and pitch from
// the reference attitude, degrees. A converging loop drives both, and
// the body rates, toward zero.
float physics_roll_deg();
float physics_pitch_deg();

}  // namespace cp::sim

#endif  // SITL or HIL

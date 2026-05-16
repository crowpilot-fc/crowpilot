// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

#include "src/Config.h"

namespace cp::airframes {

// Per-airframe geometry. Exactly one airframe compiles in, selected by
// AIRFRAME in Config.h. Each branch declares the motor and servo counts
// and the index constants its mixer uses. SERVO_LEFT and SERVO_RIGHT
// (indices 0 and 1) are defined for every airframe because the telemetry
// logger keys its two servo fields off them; airframes with more surfaces
// add their own semantic index names.

#if AIRFRAME == AIRFRAME_TAILSITTER_BICOPTER

// Tailsitter bicopter per SPEC.md §5.9: two motors, two elevon servos.
constexpr uint8_t N_MOTORS = 2;
constexpr uint8_t N_SERVOS = 2;

constexpr uint8_t MOTOR_RIGHT = 0;
constexpr uint8_t MOTOR_LEFT  = 1;
constexpr uint8_t SERVO_LEFT  = 0;
constexpr uint8_t SERVO_RIGHT = 1;

#elif AIRFRAME == AIRFRAME_PLANE_TWIN_CARGO

// Twin-engine cargo plane (Caribou): two motors plus four control
// surfaces. SERVO_LEFT/SERVO_RIGHT alias the two aileron channels so the
// telemetry logger keeps logging the first two surfaces unchanged.
constexpr uint8_t N_MOTORS = 2;
constexpr uint8_t N_SERVOS = 4;

constexpr uint8_t MOTOR_RIGHT = 0;
constexpr uint8_t MOTOR_LEFT  = 1;

constexpr uint8_t SERVO_AILERON_LEFT  = 0;
constexpr uint8_t SERVO_AILERON_RIGHT = 1;
constexpr uint8_t SERVO_ELEVATOR      = 2;
constexpr uint8_t SERVO_RUDDER        = 3;

constexpr uint8_t SERVO_LEFT  = SERVO_AILERON_LEFT;
constexpr uint8_t SERVO_RIGHT = SERVO_AILERON_RIGHT;

#elif AIRFRAME == AIRFRAME_PLANE_SINGLE

// Single-engine plane (Gee Bee): one engine plus four control surfaces.
// The firmware emits a motor signal on both motor pins; a single-engine
// build connects one ESC and leaves the other pin unused. The surface
// layout matches the twin-cargo plane.
constexpr uint8_t N_MOTORS = 2;
constexpr uint8_t N_SERVOS = 4;

constexpr uint8_t MOTOR_RIGHT = 0;
constexpr uint8_t MOTOR_LEFT  = 1;

constexpr uint8_t SERVO_AILERON_LEFT  = 0;
constexpr uint8_t SERVO_AILERON_RIGHT = 1;
constexpr uint8_t SERVO_ELEVATOR      = 2;
constexpr uint8_t SERVO_RUDDER        = 3;

constexpr uint8_t SERVO_LEFT  = SERVO_AILERON_LEFT;
constexpr uint8_t SERVO_RIGHT = SERVO_AILERON_RIGHT;

#elif AIRFRAME == AIRFRAME_QUAD_X
  #error "AIRFRAME_QUAD_X is a v2.x deliverable."
#elif AIRFRAME == AIRFRAME_HEX_X
  #error "AIRFRAME_HEX_X is a v2.x deliverable."
#elif AIRFRAME == AIRFRAME_TRICOPTER
  #error "AIRFRAME_TRICOPTER is a v2.x deliverable."
#elif AIRFRAME == AIRFRAME_TAILSITTER_QUAD
  #error "AIRFRAME_TAILSITTER_QUAD is a v2.x deliverable."
#else
  #error "Unknown AIRFRAME. Pick AIRFRAME_TAILSITTER_BICOPTER for v1.0."
#endif

// Per-actuator command output from the mixer. Motor values are clamped to
// [0, 1] (0 idle, 1 full thrust). Servo values are clamped to [0, 1] with
// 0.5 as geometric center. The actuator stage turns these into OneShot125
// pulses and servo PWM.
struct Output {
  float motor[N_MOTORS];
  float servo[N_SERVOS];
};

// Reset the latest output to safe defaults (motors at zero, servos at
// center or the configured trim).
void init();

// One mixer step. For the tailsitter the inputs are the throttle command,
// the angle-PID outputs, and the pass-through stick values, blended by
// `fader`. For the plane airframes the mixer reads the plane stabilizer
// output directly and most of these arguments are unused. The shared
// signature lets cp::core::Loop call the mixer without knowing the
// airframe.
void update(float throttle,
            float roll_pid, float pitch_pid, float yaw_pid,
            float roll_pt,  float pitch_pt,  float yaw_pt,
            float fader);

// Latest mixer output. Always safe to call after init.
const Output& output();

}  // namespace cp::airframes

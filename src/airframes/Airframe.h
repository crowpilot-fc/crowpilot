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

// Quadcopter in the X configuration: four motors, no control surfaces.
// Viewed from above with x out the nose (the FLU body frame the estimator
// uses), the motors sit at the four arm tips. Diagonal pairs spin opposite
// directions so their reaction torques cancel in a hover and differential
// spin gives yaw.
constexpr uint8_t N_MOTORS = 4;
constexpr uint8_t N_SERVOS = 0;

constexpr uint8_t MOTOR_FRONT_RIGHT = 0;
constexpr uint8_t MOTOR_FRONT_LEFT  = 1;
constexpr uint8_t MOTOR_REAR_RIGHT  = 2;
constexpr uint8_t MOTOR_REAR_LEFT   = 3;

// The telemetry logger names SERVO_LEFT and SERVO_RIGHT. A quad has no
// servos, so they are placeholders only; the logger detects a servo-less
// airframe and logs the rear motors in those two fields instead.
constexpr uint8_t SERVO_LEFT  = 0;
constexpr uint8_t SERVO_RIGHT = 0;

#elif AIRFRAME == AIRFRAME_PLANE_FLYING_WING

// Flying wing (delta): one engine plus two elevons, no rudder. Each elevon
// carries both pitch (the two move together) and roll (they move opposite),
// the same allocation the tailsitter uses in forward flight. Driven by the
// plane stabilizer. The firmware drives both motor pins with the same signal;
// a single-engine build connects one ESC and leaves the other pin idle.
constexpr uint8_t N_MOTORS = 2;
constexpr uint8_t N_SERVOS = 2;

constexpr uint8_t MOTOR_RIGHT = 0;
constexpr uint8_t MOTOR_LEFT  = 1;

constexpr uint8_t SERVO_ELEVON_LEFT  = 0;
constexpr uint8_t SERVO_ELEVON_RIGHT = 1;

constexpr uint8_t SERVO_LEFT  = SERVO_ELEVON_LEFT;
constexpr uint8_t SERVO_RIGHT = SERVO_ELEVON_RIGHT;

#elif AIRFRAME == AIRFRAME_PLANE_VTAIL

// V-tail plane: one engine, two ailerons, and two ruddervators (the V tail
// surfaces). The ailerons are differential for roll. The ruddervators carry
// pitch as their common mode and yaw as their differential, the same way the
// flying-wing elevons carry pitch and roll. Driven by the plane stabilizer.
constexpr uint8_t N_MOTORS = 2;
constexpr uint8_t N_SERVOS = 4;

constexpr uint8_t MOTOR_RIGHT = 0;
constexpr uint8_t MOTOR_LEFT  = 1;

constexpr uint8_t SERVO_AILERON_LEFT  = 0;
constexpr uint8_t SERVO_AILERON_RIGHT = 1;
constexpr uint8_t SERVO_VTAIL_LEFT    = 2;
constexpr uint8_t SERVO_VTAIL_RIGHT   = 3;

constexpr uint8_t SERVO_LEFT  = SERVO_AILERON_LEFT;
constexpr uint8_t SERVO_RIGHT = SERVO_AILERON_RIGHT;

#elif AIRFRAME == AIRFRAME_PLANE_GENERIC

// Generic plane: composed at runtime from PLANE_MOTORS_COUNT, PLANE_TAIL_STYLE,
// and PLANE_AILERON_COUNT params. Replaces the four specific plane airframes
// (single, twin cargo, flying wing, V-tail) with one mixer that handles all
// 12 combinations. The slot count is the max needed across every combination
// so the actuator stage iterates a fixed array. Unused slots stay at neutral.
//
// Output slot to header mapping (locked to the v1.0 PCB):
//   motor[0]  -> J2 (ESC1)   GP10. Always active.
//   motor[1]  -> J7 (ESC2)   GP11. Active when PLANE_MOTORS_COUNT == 2.
//   servo[0]  -> J3 (Servo1) GP12. Left aileron / left elevon / sole aileron.
//   servo[1]  -> J4 (Servo2) GP13. Right aileron / right elevon.
//   servo[2]  -> J5 (Servo3) GP8.  Elevator / V-tail left.
//   servo[3]  -> J6 (Servo4) GP9.  Rudder / V-tail right.
constexpr uint8_t N_MOTORS = 2;
constexpr uint8_t N_SERVOS = 4;

constexpr uint8_t MOTOR_RIGHT = 0;
constexpr uint8_t MOTOR_LEFT  = 1;

// Generic role indices. The mixer interprets them per the configured
// PLANE_TAIL_STYLE and PLANE_AILERON_COUNT at runtime.
constexpr uint8_t SERVO_OUT_0 = 0;
constexpr uint8_t SERVO_OUT_1 = 1;
constexpr uint8_t SERVO_OUT_2 = 2;
constexpr uint8_t SERVO_OUT_3 = 3;

constexpr uint8_t SERVO_LEFT  = SERVO_OUT_0;
constexpr uint8_t SERVO_RIGHT = SERVO_OUT_1;

#elif AIRFRAME == AIRFRAME_HEX_X
  #error "AIRFRAME_HEX_X is a v2.x deliverable."
#elif AIRFRAME == AIRFRAME_TRICOPTER
  #error "AIRFRAME_TRICOPTER is a v2.x deliverable."
#elif AIRFRAME == AIRFRAME_TAILSITTER_QUAD
  #error "AIRFRAME_TAILSITTER_QUAD is a v2.x deliverable."
#else
  #error "Unknown AIRFRAME. Pick AIRFRAME_TAILSITTER_BICOPTER for v1.0."
#endif

// Array sizing for the servo arrays. A servo-less airframe (a multirotor,
// N_SERVOS = 0) would otherwise declare zero-size arrays, which are not
// standard C++. Sizing by this floor-of-1 constant keeps one unused slot in
// that case; the loops that touch servos iterate N_SERVOS, so they simply do
// not run. For every airframe with servos this equals N_SERVOS, so nothing
// changes.
constexpr uint8_t N_SERVO_SLOTS = (N_SERVOS > 0) ? N_SERVOS : 1;

// Per-actuator command output from the mixer. Motor values are clamped to
// [0, 1] (0 idle, 1 full thrust). Servo values are clamped to [0, 1] with
// 0.5 as geometric center. The actuator stage turns these into OneShot125
// pulses and servo PWM.
struct Output {
  float motor[N_MOTORS];
  float servo[N_SERVO_SLOTS];
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

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// CrowPilot compile-time configuration.
//
// This is the single place a builder selects hardware and sets tunable
// constants. Two kinds of entry live here:
//
//   Preprocessor selectors and feature flags (#define). These gate
//   compilation: board, build target, airframe, sensor type, receiver
//   protocol, and the ENABLE_* feature switches. They must be macros
//   because the code tests them with #if.
//
//   Typed tunable constants (constexpr, in namespace cp). Pin maps,
//   bus rates, failsafe defaults, gains, limits, and trims. Module code
//   refers to them unqualified from inside its own cp:: sub-namespace.
//
// The tailsitter control-core tuning constants near the end are
// provisional. They are placeholders sized from physical scale and
// signal ranges, to be replaced by bench-tuned values per the tuning
// guide, docs/user-guide/tuning.md. Control-core modules that are
// implemented in later build phases add their own tuning constants to
// the matching section as they land.

// ===========================================================================
// Firmware identity
// ===========================================================================
// Reported by the configurator handshake. Bump on every release.

#define CROWPILOT_VERSION "1.0.0-dev"

// ===========================================================================
// Board
// ===========================================================================
// Selects the pin map. Exactly one board header is included. Each board
// header hoists its PIN_* constants into namespace cp.

#define BOARD_WAVESHARE_RP2350_TINY 0
#define BOARD_WEACT_RP2350A_V10     1

#define BOARD BOARD_WEACT_RP2350A_V10

#if   BOARD == BOARD_WAVESHARE_RP2350_TINY
  #include "src/boards/waveshare_rp2350_tiny.h"
#elif BOARD == BOARD_WEACT_RP2350A_V10
  #include "src/boards/weact_rp2350a_v10.h"
#else
  #error "Unknown BOARD. Pick BOARD_WAVESHARE_RP2350_TINY or BOARD_WEACT_RP2350A_V10."
#endif

// ===========================================================================
// Build target
// ===========================================================================
// Selects the HAL implementation. NATIVE drives real hardware. SITL
// builds the firmware as a host binary against the simulated HAL in
// src/hal/sim/. HIL is scaffolded and not yet implemented.

#define BUILD_TARGET_NATIVE 0
#define BUILD_TARGET_HIL    1
#define BUILD_TARGET_SITL   2

// The host SITL build sets BUILD_TARGET on the compiler command line.
// The Arduino build leaves it undefined and gets the native default.
#ifndef BUILD_TARGET
#define BUILD_TARGET BUILD_TARGET_NATIVE
#endif

// ===========================================================================
// Airframe
// ===========================================================================
// Selects the mixer geometry. v1.0 ships the tailsitter bicopter. The
// two plane airframes require ENABLE_PLANE_STAB = 1. The remaining
// values are reserved for v2.x and halt the build if selected.

#define AIRFRAME_TAILSITTER_BICOPTER 0
#define AIRFRAME_PLANE_TWIN_CARGO    1
#define AIRFRAME_PLANE_SINGLE        2
#define AIRFRAME_QUAD_X              3
#define AIRFRAME_HEX_X               4
#define AIRFRAME_TRICOPTER           5
#define AIRFRAME_TAILSITTER_QUAD     6

#define AIRFRAME AIRFRAME_PLANE_TWIN_CARGO

// ===========================================================================
// IMU
// ===========================================================================
// Three-axis gyro plus accelerometer on I2C0. The range selectors are
// the raw FS_SEL and AFS_SEL register fields (0 to 3).
//   Gyro range:  0 = 250, 1 = 500, 2 = 1000, 3 = 2000 deg/s.
//   Accel range: 0 = 2,   1 = 4,   2 = 8,    3 = 16 g.
// 2000 deg/s keeps fast transition rotations from clipping. 8 g leaves
// headroom for transient load without losing too much resolution on the
// gravity vector.

#define IMU_MPU6500 0
#define IMU_MPU6050 1

#define IMU_TYPE IMU_MPU6500

// One-shot IMU bias calibration. With this set, the firmware measures
// the gyro and accelerometer offsets at boot, prints them, and halts.
// Off for flight builds.
#define ENABLE_IMU_CALIBRATION 0

namespace cp {

constexpr uint8_t IMU_I2C_ADDR   = 0x68;  // AD0 tied low. 0x69 if AD0 high.
constexpr uint8_t IMU_GYRO_RANGE  = 3;    // FS_SEL  = 3 -> 2000 deg/s.
constexpr uint8_t IMU_ACCEL_RANGE = 2;    // AFS_SEL = 2 -> 8 g.

// IMU bias offsets, subtracted from every sample. Provisional zero. The
// IMU calibration routine (build phase 5) measures these with the
// vehicle held level and the builder writes the results back here.
constexpr float GYRO_BIAS_X = 0.0f;  // deg/s
constexpr float GYRO_BIAS_Y = 0.0f;  // deg/s
constexpr float GYRO_BIAS_Z = 0.0f;  // deg/s
constexpr float ACC_BIAS_X  = 0.0f;  // g
constexpr float ACC_BIAS_Y  = 0.0f;  // g
constexpr float ACC_BIAS_Z  = 0.0f;  // g

// Samples averaged by the IMU bias calibration routine. The routine
// paces one read per millisecond, so 2000 samples span about two
// seconds, long enough for a stable mean of the MEMS sensor noise.
constexpr uint32_t IMU_CALIB_SAMPLE_COUNT = 2000;

}  // namespace cp

// ===========================================================================
// Barometer
// ===========================================================================
// Optional pressure sensor on I2C0. BARO_NONE builds with no barometer.

#define BARO_BMP388 0
#define BARO_BMP280 1
#define BARO_NONE   2

#define BARO_TYPE BARO_BMP388

namespace cp {

constexpr uint8_t BARO_I2C_ADDR = 0x77;  // SDO high. 0x76 if SDO low.

// Barometer read cadence. The sensor is read once every N loop ticks.
// At LOOP_HZ = 1000 a value of 20 gives a 50 Hz barometer rate.
constexpr uint32_t BARO_READ_INTERVAL_TICKS = 20;

}  // namespace cp

// ===========================================================================
// Loop rate
// ===========================================================================
// The cooperative super-loop runs at this rate. The whole control core
// is timed against it.

namespace cp {

constexpr uint32_t LOOP_HZ = 1000;  // 1 kHz tick.

}  // namespace cp

// ===========================================================================
// I2C
// ===========================================================================

namespace cp {

constexpr uint32_t I2C_BUS_HZ = 400000;  // 400 kHz fast mode on I2C0.

}  // namespace cp

// ===========================================================================
// Receiver
// ===========================================================================
// v1.0 decodes SBUS with an RP2350 PIO state machine. SBUS is an
// inverted serial signal, so the PIO program reads it inverted with no
// external hardware inverter. The other protocols are reserved.

#define RX_SBUS 0
#define RX_PPM  1
#define RX_PWM  2
#define RX_CRSF 3

#define RX_PROTOCOL RX_SBUS

// v1 supports inverted SBUS only. The PIO program is fixed to the
// inverted-SBUS line polarity; this flag only XORs the captured data
// bits. Setting it to 0 does not make a non-inverted receiver decode.
#define RX_SBUS_INVERTED 1

// ===========================================================================
// Pilot channel map and RC signal range
// ===========================================================================
// The pilot transmitter assigns one function per channel. Channel
// numbers are 1-based to match transmitter numbering. RC_MIN_US,
// RC_MID_US, and RC_MAX_US are the pulse widths a centered and a fully
// deflected stick produce, used to normalize raw channels into
// setpoints.

namespace cp {

constexpr uint8_t CHANNEL_THROTTLE   = 1;
constexpr uint8_t CHANNEL_ROLL       = 2;
constexpr uint8_t CHANNEL_PITCH      = 3;
constexpr uint8_t CHANNEL_YAW        = 4;
constexpr uint8_t CHANNEL_ARM        = 5;
constexpr uint8_t CHANNEL_TRANSITION = 6;

constexpr uint16_t RC_MIN_US = 1000;
constexpr uint16_t RC_MID_US = 1500;
constexpr uint16_t RC_MAX_US = 2000;

}  // namespace cp

// ===========================================================================
// Failsafe
// ===========================================================================
// On lost link the failsafe replaces the receiver channels with these
// values to fly a gentle, level, powered descent. The throttle sits
// below hover so the aircraft sinks slowly. The arm channel stays in
// the armed position so the descent stays powered. The transition
// channel holds the hover end. Sticks center.

namespace cp {

constexpr uint16_t FS_CH1_THROTTLE    = 1300;  // us. Below hover, gentle sink.
constexpr uint16_t FS_CH2_ROLL        = 1500;  // us. Centered.
constexpr uint16_t FS_CH3_PITCH       = 1500;  // us. Centered.
constexpr uint16_t FS_CH4_YAW         = 1500;  // us. Centered.
constexpr uint16_t FS_CH5_ARM         = 1000;  // us. Armed position, descent stays powered.
constexpr uint16_t FS_CH6_TRANSITION  = 2000;  // us. Hover end (high pulse).

// Link is considered lost when no fresh receiver frame arrives within
// this window.
constexpr uint32_t FS_LINK_TIMEOUT_US = 100000;  // 100 ms.

}  // namespace cp

// ===========================================================================
// Telemetry
// ===========================================================================
// Fixed-size binary records logged to SD. The logger is rate-limited to
// one record every N ticks and rotates to a new file at a size cap.

#define ENABLE_TELEMETRY_LOG 1

namespace cp {

// At LOOP_HZ = 1000 a value of 10 gives a 100 Hz log rate.
constexpr uint32_t TELEMETRY_LOG_INTERVAL_TICKS = 10;

// New log file started once the current file reaches this size.
constexpr uint32_t TELEMETRY_LOG_MAX_BYTES = 16u * 1024u * 1024u;  // 16 MiB.

}  // namespace cp

// ===========================================================================
// Parameters
// ===========================================================================
// Runtime parameter registry with optional flash persistence and
// optional transmitter-channel live tuning. Live tuning reads two spare
// channels to scale the active P and D gains within +/- LIVE_TUNE_RANGE.
// ENABLE_CONFIG_CLI adds the serial command interface the browser-based
// configurator speaks. It is a bench tool. A flight-only build can drop
// it to save code space.

#define ENABLE_PARAM_PERSIST 1
#define ENABLE_LIVE_TUNE     1
#define ENABLE_CONFIG_CLI    1

namespace cp {

constexpr uint8_t LIVE_TUNE_CH_KP = 9;   // 1-based channel index for the P knob.
constexpr uint8_t LIVE_TUNE_CH_KD = 10;  // 1-based channel index for the D knob.

// Fractional tuning range. 0.5 lets a knob scale a gain over [0.5x, 1.5x].
constexpr float LIVE_TUNE_RANGE = 0.5f;

}  // namespace cp

// ===========================================================================
// Plane stabilization
// ===========================================================================
// Fixed-wing stabilizer for the plane airframes. Inactive for the
// tailsitter, but the constants are still defined so the carried plane
// code compiles. Selecting a plane airframe requires ENABLE_PLANE_STAB
// to be 1.

#define ENABLE_PLANE_STAB      1
#define ENABLE_ALT_HOLD        0
#define ENABLE_DIFF_THRUST_YAW 0

namespace cp {

constexpr uint8_t CHANNEL_STAB     = 7;  // 1-based: stabilization on/off switch.
constexpr uint8_t CHANNEL_ALT_HOLD = 8;  // 1-based: altitude-hold on/off switch.

// Wing leveler, pitch hold, and yaw damper gains. Provisional.
constexpr float KP_STAB_ROLL  = 0.50f;
constexpr float KD_STAB_ROLL  = 0.05f;
constexpr float KP_STAB_PITCH = 0.50f;
constexpr float KD_STAB_PITCH = 0.05f;
constexpr float KD_STAB_YAW   = 0.10f;

// Scales each stabilizer axis output before the final clamp. Provisional.
constexpr float STAB_OUTPUT_SCALE = 1.0f;

// Barometric altitude hold. Provisional.
constexpr float ALT_CLIMB_FILTER_ALPHA = 0.10f;  // climb-rate low-pass alpha.
constexpr float KP_ALT = 0.05f;
constexpr float KD_ALT = 0.10f;

// Control-surface travel as a fraction of half throw from center.
// Provisional.
constexpr float AILERON_TRAVEL  = 0.50f;
constexpr float ELEVATOR_TRAVEL = 0.50f;
constexpr float RUDDER_TRAVEL   = 0.50f;

// Differential-thrust yaw assist for the twin-cargo plane. Provisional.
constexpr float DIFF_THRUST_GAIN = 0.20f;

}  // namespace cp

// ===========================================================================
// ESC, servo, and arming
// ===========================================================================
// MOTOR_PROTOCOL selects how the motor pins are driven. ONESHOT125 is a
// 125 to 250 us synchronous pulse, bit-banged once per loop tick. PWM is
// standard 1000 to 2000 us hobby PWM at 50 Hz, the same signal a servo
// takes, for ESCs that do not support OneShot. Servos always take
// standard PWM. Arming requires the throttle stick at or below
// ARM_THROTTLE_MAX_US. ESC calibration is a one-shot bench routine,
// kept off for flight builds.

#define MOTOR_PROTOCOL_ONESHOT125 0
#define MOTOR_PROTOCOL_PWM        1

#define MOTOR_PROTOCOL MOTOR_PROTOCOL_PWM

#define ENABLE_ESC_CALIBRATION 0

namespace cp {

#if MOTOR_PROTOCOL == MOTOR_PROTOCOL_PWM
constexpr uint16_t ESC_MAX_PULSE_US    = 2000; // Standard PWM full throttle.
constexpr uint16_t ESC_IDLE_PULSE_US   = 1000; // Standard PWM zero throttle.
constexpr uint16_t ESC_DISARM_PULSE_US = 1000; // Motor stopped when disarmed.
#else
constexpr uint16_t ESC_MAX_PULSE_US    = 250;  // OneShot125 full throttle.
constexpr uint16_t ESC_IDLE_PULSE_US   = 125;  // OneShot125 zero throttle.
constexpr uint16_t ESC_DISARM_PULSE_US = 120;  // Below valid range: no signal.
#endif
constexpr uint16_t ARM_THROTTLE_MAX_US = 1050; // Throttle-idle gate for arming.

constexpr uint16_t SERVO_MIN_US = 1000;        // Servo PWM at command 0.
constexpr uint16_t SERVO_MAX_US = 2000;        // Servo PWM at command 1.

}  // namespace cp

// ===========================================================================
// User hook
// ===========================================================================
// Optional non-flight-critical user extension, run rate-limited and
// time-budgeted. Off by default.

#define ENABLE_USER_HOOK 0

namespace cp {

// The hook runs once every N loop ticks. At LOOP_HZ = 1000 a value of
// 20 gives a 50 Hz hook rate.
constexpr uint32_t USER_HOOK_RATE_DIV = 20;

// Soft budget warns over serial. Hard limit marks the hook tripped.
constexpr uint32_t USER_HOOK_BUDGET_US     = 150;
constexpr uint32_t USER_HOOK_HARD_LIMIT_US = 250;

}  // namespace cp

// ===========================================================================
// Attitude estimation
// ===========================================================================
// The AHRS is a Madgwick gradient-descent orientation filter, gyro and
// accelerometer only. MADGWICK_BETA is its single gain. Madgwick 2011
// derives beta as sqrt(3/4) times the gyro measurement error in rad/s.
// For an assumed 7.5 deg/s MEMS gyro error that is about 0.11. A higher
// beta tracks the accelerometer faster and is noisier, a lower beta
// trusts the gyro longer and drifts more. Provisional, refined by bench
// observation per spec section 3.

namespace cp {

constexpr float MADGWICK_BETA = 0.10f;

}  // namespace cp

// ===========================================================================
// Transition and pilot setpoints
// ===========================================================================
// The transition channel commands the move between hover and forward
// flight. The fader slews toward that command at TRANSITION_SLEW_RATE,
// in fader units per second, so even a snapped switch yields a
// controllable maneuver. The setpoint limits scale a full stick
// deflection to a roll or pitch angle and a yaw rate. All provisional,
// set by bench tuning per spec section 12.

namespace cp {

constexpr float TRANSITION_SLEW_RATE = 0.33f;  // full transition in about 3 s.

constexpr float MAX_ROLL_ANGLE_DEG  = 35.0f;
constexpr float MAX_PITCH_ANGLE_DEG = 35.0f;
constexpr float MAX_YAW_RATE_DPS    = 120.0f;

}  // namespace cp

// ===========================================================================
// Tailsitter control-core tuning (provisional)
// ===========================================================================
// Per-axis PID gains for the tailsitter stabilizer, registered as
// runtime parameters in src/params/params.def. Two gain sets per axis,
// one for hover and one for forward flight, blended continuously by the
// transition value. The integral gains start at zero.
//
// These are provisional values to be replaced by bench tuning. The
// documented procedure is to tune the D term first, then P, then I, per
// axis, hover before forward flight. See docs/user-guide/tuning.md.
// The params.def clamp bounds are P in [0, 4] and D and I
// in [0, 1]. The starting magnitudes here are sized small so the first
// powered tests are gentle.

namespace cp {

// Roll and pitch stabilize on an attitude error in degrees. Yaw
// stabilizes on a rate error in degrees per second, a larger-numbered
// signal, so its gains are correspondingly smaller. The magnitudes
// below are deliberately low so the controller responds gently on the
// first powered tests and is tuned up from there.

// Hover gain set.
constexpr float Kp_roll_hover  = 0.030f;
constexpr float Kd_roll_hover  = 0.004f;
constexpr float Kp_pitch_hover = 0.030f;
constexpr float Kd_pitch_hover = 0.004f;
constexpr float Kp_yaw_hover   = 0.008f;
constexpr float Kd_yaw_hover   = 0.002f;

// Forward-flight gain set.
constexpr float Kp_roll_ff  = 0.025f;
constexpr float Kd_roll_ff  = 0.003f;
constexpr float Kp_pitch_ff = 0.025f;
constexpr float Kd_pitch_ff = 0.003f;
constexpr float Kp_yaw_ff   = 0.006f;
constexpr float Kd_yaw_ff   = 0.0015f;

// Integral gains, shared across regimes. Start at zero and tune last.
constexpr float Ki_roll  = 0.0f;
constexpr float Ki_pitch = 0.0f;
constexpr float Ki_yaw   = 0.0f;

// Anti-windup limit: the maximum magnitude of the integral term, Ki
// times the accumulated error, as a fraction of the normalized output
// range. Provisional.
constexpr float PID_INTEGRAL_LIMIT = 0.5f;

}  // namespace cp

// ===========================================================================
// Tailsitter mixer (provisional)
// ===========================================================================
// Allocation of throttle and the stabilized axis demands to the two
// motors and two elevons. The elevon trims are the per-airframe
// mechanical neutral, 0.5 being geometric center. The gain constants
// scale a unit axis demand to actuator travel. The feedforward gain
// adds direct pilot stick to the elevons and fades in toward forward
// flight. All provisional, set by bench tuning per spec section 12.

namespace cp {

constexpr float ELEVON_LEFT_TRIM  = 0.5f;  // servo command, [0, 1]
constexpr float ELEVON_RIGHT_TRIM = 0.5f;

constexpr float ELEVON_ROLL_GAIN  = 0.5f;  // elevon travel per unit roll demand
constexpr float ELEVON_PITCH_GAIN = 0.5f;  // elevon travel per unit pitch demand
constexpr float ELEVON_FF_GAIN    = 0.2f;  // forward-flight stick feedforward

constexpr float MOTOR_YAW_GAIN    = 0.3f;  // differential thrust per unit yaw demand

}  // namespace cp

// ===========================================================================
// Debug output
// ===========================================================================
// Compile-time gates for the main loop's serial debug prints. Each is
// independent. DEBUG_PRINT_DEV is on by default and prints a one-line
// status summary plus a loop-period report. The others are bench-
// diagnostic streams, enabled one at a time while working through the
// first-bench-test procedure. Every print is also gated at runtime on
// a live USB serial connection.

#define DEBUG_PRINT_DEV        1
#define DEBUG_PRINT_IMU        0
#define DEBUG_PRINT_RX         0
#define DEBUG_PRINT_FAILSAFE   0
#define DEBUG_PRINT_ATTITUDE   0
#define DEBUG_PRINT_MODE       0
#define DEBUG_PRINT_MIXER      0
#define DEBUG_PRINT_USER_HOOK  0

namespace cp {

// Print cadences in loop ticks. At LOOP_HZ = 1000: the DEV summary
// every 200 ticks (5 Hz), the bench-diagnostic streams every 100 ticks
// (10 Hz), the loop-period report every 1000 ticks (1 Hz).
constexpr uint32_t DEBUG_DEV_INTERVAL_TICKS    = 200;
constexpr uint32_t DEBUG_STREAM_INTERVAL_TICKS = 100;
constexpr uint32_t DEBUG_LOOP_REPORT_TICKS     = 1000;

}  // namespace cp

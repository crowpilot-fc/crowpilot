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
// signal ranges, to be replaced by bench-tuned values per
// TAILSITTER_CONTROL_SPEC.md section 12. Control-core modules that are
// implemented in later build phases add their own tuning constants to
// the matching section as they land.

// ===========================================================================
// Board
// ===========================================================================
// Selects the pin map. Exactly one board header is included. Each board
// header hoists its PIN_* constants into namespace cp.

#define BOARD_WAVESHARE_RP2350_TINY 0
#define BOARD_WEACT_RP2350A_V10     1

#define BOARD BOARD_WAVESHARE_RP2350_TINY

#if   BOARD == BOARD_WAVESHARE_RP2350_TINY
  #include "boards/waveshare_rp2350_tiny.h"
#elif BOARD == BOARD_WEACT_RP2350A_V10
  #include "boards/weact_rp2350a_v10.h"
#else
  #error "Unknown BOARD. Pick BOARD_WAVESHARE_RP2350_TINY or BOARD_WEACT_RP2350A_V10."
#endif

// ===========================================================================
// Build target
// ===========================================================================
// Selects the HAL implementation. NATIVE drives real hardware. HIL and
// SITL are scaffolded for a future simulation path and are not yet
// implemented.

#define BUILD_TARGET_NATIVE 0
#define BUILD_TARGET_HIL    1
#define BUILD_TARGET_SITL   2

#define BUILD_TARGET BUILD_TARGET_NATIVE

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

#define AIRFRAME AIRFRAME_TAILSITTER_BICOPTER

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

#define RX_SBUS_INVERTED 1

// ===========================================================================
// Failsafe
// ===========================================================================
// On lost link the failsafe replaces the receiver channels with these
// values to fly a gentle, level, powered descent. The throttle sits
// below hover so the aircraft sinks slowly. The arm channel stays in
// the armed position so the descent stays powered. The transition
// channel holds the hover end. Sticks center.

namespace cp {

constexpr uint16_t FS_CH1_THROTTLE    = 1350;  // us. Below hover, gentle sink.
constexpr uint16_t FS_CH2_ROLL        = 1500;  // us. Centered.
constexpr uint16_t FS_CH3_PITCH       = 1500;  // us. Centered.
constexpr uint16_t FS_CH4_YAW         = 1500;  // us. Centered.
constexpr uint16_t FS_CH5_ARM         = 1000;  // us. Armed position, descent stays powered.
constexpr uint16_t FS_CH6_TRANSITION  = 1000;  // us. Hover end.

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

#define ENABLE_PARAM_PERSIST 1
#define ENABLE_LIVE_TUNE     1

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

#define ENABLE_PLANE_STAB      0
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
// ESC and arming
// ===========================================================================
// Motors run the OneShot125 protocol: 125 us idle, 250 us full. The
// disarm pulse sits below the valid range so a disarmed ESC sees no
// signal and stays silent. Arming requires the throttle stick at or
// below ARM_THROTTLE_MAX_US. ESC calibration is a one-shot bench
// routine, kept off for flight builds.

#define ENABLE_ESC_CALIBRATION 0

namespace cp {

constexpr uint16_t ESC_MAX_PULSE_US    = 250;  // OneShot125 full throttle.
constexpr uint16_t ESC_IDLE_PULSE_US   = 125;  // OneShot125 zero throttle.
constexpr uint16_t ESC_DISARM_PULSE_US = 120;  // Below valid range: no signal.
constexpr uint16_t ARM_THROTTLE_MAX_US = 1050; // Throttle-idle gate for arming.

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
// Tailsitter control-core tuning (provisional)
// ===========================================================================
// Per-axis PID gains for the tailsitter stabilizer, registered as
// runtime parameters in src/params/params.def. Two gain sets per axis,
// one for hover and one for forward flight, blended continuously by the
// transition value. The integral gains start at zero.
//
// These are provisional values to be replaced by bench tuning. The
// documented procedure is to tune the D term first, then P, then I, per
// axis, hover before forward flight (TAILSITTER_CONTROL_SPEC.md
// section 12). The params.def clamp bounds are P in [0, 4] and D and I
// in [0, 1]. The starting magnitudes here are sized small so the first
// powered tests are gentle.
//
// Further control-core tuning constants (attitude filter gain, IMU
// calibration sample count, transition slew rate, pilot stick limits,
// elevon trim) are added to this file in the build phases that
// implement the modules that use them.

namespace cp {

// Hover gain set.
constexpr float Kp_roll_hover  = 0.50f;
constexpr float Kd_roll_hover  = 0.05f;
constexpr float Kp_pitch_hover = 0.50f;
constexpr float Kd_pitch_hover = 0.05f;
constexpr float Kp_yaw_hover   = 0.50f;
constexpr float Kd_yaw_hover   = 0.05f;

// Forward-flight gain set.
constexpr float Kp_roll_ff  = 0.40f;
constexpr float Kd_roll_ff  = 0.04f;
constexpr float Kp_pitch_ff = 0.40f;
constexpr float Kd_pitch_ff = 0.04f;
constexpr float Kp_yaw_ff   = 0.30f;
constexpr float Kd_yaw_ff   = 0.03f;

// Integral gains, shared across regimes. Start at zero and tune last.
constexpr float Ki_roll  = 0.0f;
constexpr float Ki_pitch = 0.0f;
constexpr float Ki_yaw   = 0.0f;

}  // namespace cp

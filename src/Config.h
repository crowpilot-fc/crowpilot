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
#define BOARD_PICO2W                2
#define BOARD_SMARTELEX_RP2350A_NEO 3

#ifndef BOARD
  #define BOARD BOARD_WEACT_RP2350A_V10
#endif

#if   BOARD == BOARD_WAVESHARE_RP2350_TINY
  #include "src/boards/waveshare_rp2350_tiny.h"
#elif BOARD == BOARD_WEACT_RP2350A_V10
  #include "src/boards/weact_rp2350a_v10.h"
#elif BOARD == BOARD_PICO2W
  #include "src/boards/pico2w.h"
#elif BOARD == BOARD_SMARTELEX_RP2350A_NEO
  #include "src/boards/smartelex_rp2350a_neo.h"
#else
  #error "Unknown BOARD. Pick BOARD_WAVESHARE_RP2350_TINY, BOARD_WEACT_RP2350A_V10, BOARD_PICO2W, or BOARD_SMARTELEX_RP2350A_NEO."
#endif

// A board header defines BOARD_HAS_WIFI when it carries an integrated radio
// (e.g. the Pico 2 W's CYW43439). Boards without one default to 0.
#ifndef BOARD_HAS_WIFI
  #define BOARD_HAS_WIFI 0
#endif

// A board header defines BOARD_HAS_ESP_FLASH when it exposes PIN_ESP_EN and
// PIN_ESP_IO0 (the two optional control jumpers from the FC to the ESP
// companion's EN and GPIO9 strap pins). With both wired, the cp esp flash
// command on the FC drops the ESP into its UART ROM bootloader and bridges
// USB CDC to the companion UART so esptool can program the chip without ever
// touching its own USB port. The NEO is pin-tight and leaves this at 0.
#ifndef BOARD_HAS_ESP_FLASH
  #define BOARD_HAS_ESP_FLASH 0
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

// SIM_SCENARIO picks the initial conditions and any persistent
// disturbance the SITL physics applies. Only used by the SITL plane
// model in src/hal/sim/SimPhysicsPlane.cpp. Adding more scenarios is a
// small extension there.
//
//   DEFAULT      20 deg bank + 12 deg pitch upset, calm air. The
//                wing-leveler and pitch-hold must recover to level.
//   LEVEL        no initial disturbance, calm air. Baseline /
//                regression check that the controller does not drift
//                from level on its own.
//   LARGE_UPSET  45 deg bank + 30 deg pitch upset, calm air. Stress
//                test of the stabilizer's recovery from a big upset.
//   WIND         small upset plus a steady body-axis disturbance
//                torque, equivalent to a constant cross-wind. The
//                stabilizer must hold level against the persistent
//                tendency to roll and pitch off axis.

#define SIM_SCENARIO_DEFAULT     0
#define SIM_SCENARIO_LEVEL       1
#define SIM_SCENARIO_LARGE_UPSET 2
#define SIM_SCENARIO_WIND        3

#ifndef SIM_SCENARIO
#define SIM_SCENARIO SIM_SCENARIO_DEFAULT
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
#define AIRFRAME_PLANE_FLYING_WING   7
#define AIRFRAME_PLANE_VTAIL         8

#ifndef AIRFRAME
  #define AIRFRAME AIRFRAME_PLANE_TWIN_CARGO
#endif

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
// SBUS is decoded by an RP2350 PIO state machine: SBUS is an inverted serial
// signal, so the PIO reads it inverted with no external hardware inverter.
// CRSF (Crossfire / ELRS) is decoded from a 420 kbaud UART on the same
// receiver pin (it is not inverted, so no PIO). PPM and PWM are reserved and
// halt the build.

#define RX_SBUS 0
#define RX_PPM  1
#define RX_PWM  2
#define RX_CRSF 3

#ifndef RX_PROTOCOL
  #define RX_PROTOCOL RX_SBUS
#endif

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

// AETR primaries on ch1-4. ch5 and ch6 carry a second throttle and a
// second aileron on the Caribou's transmitter and are read by the user
// sketch rather than the firmware mixer. ch7, ch8, ch9 are user-sketch
// aux switches (gear, flap, bay). Stab and arm sit on the high channels
// so they cannot collide with any TX-side primary or aux assignment.
constexpr uint8_t CHANNEL_ROLL       = 1;
constexpr uint8_t CHANNEL_PITCH      = 2;
constexpr uint8_t CHANNEL_THROTTLE   = 3;
constexpr uint8_t CHANNEL_YAW        = 4;
constexpr uint8_t CHANNEL_ARM        = 16;  // dedicated arm switch, top SBUS channel
constexpr uint8_t CHANNEL_TRANSITION = 15;  // tailsitter fader only; parked here so the plane build leaves it at neutral 1500

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

// Failsafe channel values, written into the channel-1-based slot of
// whichever CHANNEL_* the firmware reads for that role. All other slots
// stay at 1500 (centred). Naming is by role, not by channel index, so
// the channel map can move without renaming the constants.
constexpr uint16_t FS_THROTTLE_US   = 1300;  // below hover, gentle sink
constexpr uint16_t FS_ROLL_US       = 1500;  // centred
constexpr uint16_t FS_PITCH_US      = 1500;  // centred
constexpr uint16_t FS_YAW_US        = 1500;  // centred
constexpr uint16_t FS_ARM_US        = 1000;  // armed; descent stays powered
constexpr uint16_t FS_TRANSITION_US = 2000;  // hover end (high pulse is hover)

// Link is considered lost when no fresh receiver frame arrives within
// this window.
constexpr uint32_t FS_LINK_TIMEOUT_US = 100000;  // 100 ms.

}  // namespace cp

// ===========================================================================
// Battery monitor
// ===========================================================================
// Optional pack-voltage monitoring through a resistor divider into an ADC pin
// (GP26 to GP29 on the RP2350). The monitor reports the pack voltage and the
// per-cell voltage, auto-detects the cell count at power-up, and raises a
// low-voltage warning flag surfaced in telemetry. The warning is informational
// in v1: there is no automatic in-flight throttle action, since a safe
// low-battery response for a plane is return-to-home, which needs GPS (v2).
// The pre-arm checks also refuse to arm a pack below BATTERY_ARM_MIN_CELL_V.
//
// The divider ratio is Vbattery / Vadc. For a 10 k to 1 k divider that is 11.
// Size the divider so a full pack stays under the ADC reference (about 3.3 V)
// at the pin: a 6S pack at 25.2 V needs a ratio of at least 8.

#define ENABLE_BATTERY_MONITOR 0

namespace cp {

constexpr uint8_t BATTERY_ADC_PIN        = 28;     // GP28 = ADC2.
constexpr float   BATTERY_DIVIDER_RATIO  = 11.0f;  // Vbattery / Vadc.
constexpr float   BATTERY_ADC_VREF       = 3.3f;   // ADC full-scale voltage.
constexpr uint8_t BATTERY_CELLS          = 0;      // 0 = auto-detect, else fixed cell count.

constexpr float CELL_MAX_V             = 4.30f;  // detect ceiling, per cell.
constexpr float CELL_WARN_V            = 3.50f;  // low-voltage warning, per cell.
constexpr float CELL_MIN_PRESENT_V     = 3.00f;  // below this per cell the pack reads as absent (USB power).
constexpr float BATTERY_ARM_MIN_CELL_V = 3.40f;  // pre-arm refuses to arm below this.

// Voltage low-pass to settle ADC noise and motor-current sag. 0.05 is a slow,
// steady reading; raise it for a faster but noisier response.
constexpr float BATTERY_FILTER_ALPHA = 0.05f;

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
//
// ENABLE_COMPANION_CLI also runs that interface on the companion UART
// (PIN_COMPANION_TX / PIN_COMPANION_RX), so an ESP WiFi module can bridge
// it to a phone. Native builds only; shares the UART with a future GPS.
//
// ENABLE_ONBOARD_WIFI is the no-ESP path for boards with an integrated radio
// (BOARD_HAS_WIFI, e.g. the Pico 2 W). The flight controller raises its own
// WiFi access point and serves the companion UI from core1, leaving the 1 kHz
// flight loop on core0. It defaults on for a WiFi board and is meaningless
// without one. Validate loop jitter on the bench before relying on it.

#define ENABLE_PARAM_PERSIST 1
#define ENABLE_LIVE_TUNE     1
#define ENABLE_CONFIG_CLI    1
#define ENABLE_COMPANION_CLI 1

#ifndef ENABLE_ONBOARD_WIFI
  #define ENABLE_ONBOARD_WIFI BOARD_HAS_WIFI
#endif
#if ENABLE_ONBOARD_WIFI && !BOARD_HAS_WIFI
  #error "ENABLE_ONBOARD_WIFI needs a board with an integrated radio (BOARD_PICO2W)."
#endif

namespace cp {

constexpr uint8_t LIVE_TUNE_CH_KP = 11;  // 1-based channel index for the P knob.
constexpr uint8_t LIVE_TUNE_CH_KD = 12;  // 1-based channel index for the D knob.

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

#define ENABLE_PLANE_STAB        1
#define ENABLE_ALT_HOLD          0
#define ENABLE_DIFF_THRUST_YAW   0

// Coordinated-turn assist for the self-leveling plane modes (angle and
// horizon). Two first-order corrections: an auto-rudder proportional to bank
// so the turn stays coordinated, and a bank-compensated up-elevator (about
// 1/cos(bank) - 1) so the plane holds altitude through the turn instead of
// dropping its nose. Off by default, like the other optional plane assists.
// True coordination needs a slip sensor, which v1 does not have, so this is a
// feedforward approximation, not a closed-loop ball-centering.
#define ENABLE_TURN_COORDINATION 0

// Plane flight modes, selected by the three positions of CHANNEL_STAB.
//   MANUAL   full passthrough, no stabilization.
//   RATE     the sticks command a body rate and the gyro damps to it. No
//            self-level. This is the "gyro" or "3D" mode of a stabilizer.
//   ANGLE    the sticks command an attitude and the stabilizer self-levels
//            to it. The beginner-safe mode.
//   HORIZON  self-levels near center stick and blends to rate at full stick.
// A two-position switch only reaches the LOW and HIGH positions, so the
// default mapping keeps LOW = ANGLE and HIGH = MANUAL, matching the older
// stabilized-or-manual switch. A three-position switch adds the MID position.
// Remap the three positions below to reach RATE, for example MID = RATE.
#define PLANE_MODE_MANUAL  0
#define PLANE_MODE_RATE    1
#define PLANE_MODE_ANGLE   2
#define PLANE_MODE_HORIZON 3

#define PLANE_MODE_SW_LOW  PLANE_MODE_ANGLE
#define PLANE_MODE_SW_MID  PLANE_MODE_HORIZON
#define PLANE_MODE_SW_HIGH PLANE_MODE_MANUAL

namespace cp {

constexpr uint8_t CHANNEL_STAB     = 14;  // 1-based: flight-mode switch (high SBUS channel, away from TX primaries and aux).
constexpr uint8_t CHANNEL_ALT_HOLD = 13;  // 1-based: altitude-hold on/off switch.

// Mode-switch thresholds on CHANNEL_STAB, in microseconds. Below LOW_MAX is
// the low position, below MID_MAX is the middle, at or above is the high
// position.
constexpr uint16_t PLANE_MODE_SW_LOW_MAX_US = 1300;
constexpr uint16_t PLANE_MODE_SW_MID_MAX_US = 1700;

// Wing leveler, pitch hold, and yaw damper gains. Provisional.
constexpr float KP_STAB_ROLL  = 0.50f;
constexpr float KD_STAB_ROLL  = 0.05f;
constexpr float KP_STAB_PITCH = 0.50f;
constexpr float KD_STAB_PITCH = 0.05f;
constexpr float KD_STAB_YAW   = 0.10f;

// Rate-mode gains and the full-stick body rate. In RATE and HORIZON modes the
// sticks command a roll or pitch rate up to PLANE_RATE_MAX_DPS, and a
// proportional law drives the rate error into the surfaces. Provisional.
constexpr float PLANE_RATE_MAX_DPS  = 180.0f;
constexpr float KP_PLANE_RATE_ROLL  = 0.005f;
constexpr float KP_PLANE_RATE_PITCH = 0.005f;

// Scales each stabilizer axis output before the final clamp. Provisional.
constexpr float STAB_OUTPUT_SCALE = 1.0f;

// Coordinated-turn assist gains. Used only when ENABLE_TURN_COORDINATION is 1.
// TURN_COORD_RUDDER_GAIN scales sin(bank) into the rudder. PITCH_TURN_COMP_GAIN
// scales the (1/cos(bank) - 1) altitude-hold-in-turn term into the elevator.
// TURN_COMP_MAX_BANK_DEG clamps the bank used for that term so 1/cos stays
// bounded near knife-edge. Provisional.
constexpr float TURN_COORD_RUDDER_GAIN = 0.30f;
constexpr float PITCH_TURN_COMP_GAIN   = 0.50f;
constexpr float TURN_COMP_MAX_BANK_DEG = 60.0f;

// Barometric altitude hold. Provisional.
constexpr float ALT_CLIMB_FILTER_ALPHA = 0.10f;  // climb-rate low-pass alpha.
constexpr float KP_ALT = 0.05f;
constexpr float KD_ALT = 0.10f;

// Control-surface travel as a fraction of half throw from center.
// Provisional.
constexpr float AILERON_TRAVEL  = 0.50f;
constexpr float ELEVATOR_TRAVEL = 0.50f;
constexpr float RUDDER_TRAVEL   = 0.50f;

// Differential ailerons: the down-going aileron throw as a fraction of the
// up-going. 1.0 is symmetric (no differential). Lower it (for example 0.6) to
// cut adverse yaw. Applied on every aileron plane, no enable flag needed.
constexpr float DIFF_AILERON_RATIO = 1.0f;

// Differential-thrust yaw assist for the twin-cargo plane. Provisional.
constexpr float DIFF_THRUST_GAIN = 0.20f;

}  // namespace cp

// Wing braking and flaps on the ailerons (flaperon / airbrake), for the
// conventional aileron planes. A flap channel droops both ailerons as flaps,
// and a brake channel reflexes both up for a glide-path brake. On a
// two-aileron wing the brake is the achievable part of four-surface crow,
// which also needs dedicated flap servos this airframe set does not have. Both
// off by default.
#define ENABLE_FLAPERON 0
#define ENABLE_CROW     0

namespace cp {

constexpr uint8_t CHANNEL_FLAP = 8;   // 1-based flap channel (flaperon droop).
constexpr uint8_t CHANNEL_CROW = 7;   // 1-based brake channel (airbrake reflex).

constexpr float FLAPERON_TRAVEL = 0.30f;  // aileron droop at full flap.
constexpr float CROW_TRAVEL     = 0.40f;  // aileron up-reflex at full brake.

}  // namespace cp

// ===========================================================================
// Launch assist (hand-launch)
// ===========================================================================
// A hand-launch sequence for the plane airframes, gyro and accelerometer only.
// The pilot arms with the throttle at idle and throws the plane. A forward
// acceleration spike marks the throw, after which the assist commands a
// wings-level climb-out (a launch throttle and a nose-up pitch held by the
// stabilizer) for a fixed window, so the pilot can let go during the throw.
// The window ends on a timeout, a stick input, or disarm, and does not
// re-trigger until the next arm cycle. Intended for the angle and horizon
// modes. Off by default. Provisional, bench-tuned before use.

#define ENABLE_LAUNCH_ASSIST 0

namespace cp {

constexpr float    LAUNCH_DETECT_ACCEL_G = 2.0f;   // forward accel (g) that marks the throw
constexpr float    LAUNCH_IDLE_MAX       = 0.15f;  // throttle below which the detector arms
constexpr float    LAUNCH_THROTTLE       = 0.70f;  // climb-out throttle, normalized
constexpr float    LAUNCH_PITCH_DEG      = 15.0f;  // climb-out nose-up attitude
constexpr uint32_t LAUNCH_DURATION_MS    = 1500;   // climb-out window
constexpr float    LAUNCH_STICK_EXIT     = 0.20f;  // stick deflection that hands control back

}  // namespace cp

// ===========================================================================
// ESC, servo, and arming
// ===========================================================================
// MOTOR_PROTOCOL selects how the motor pins are driven.
//
//   ONESHOT125  a 125 to 250 us synchronous pulse, bit-banged once per
//               loop tick.
//   PWM         standard 1000 to 2000 us hobby PWM at 50 Hz, the same
//               signal a servo takes, for ESCs that do not support
//               OneShot. The default.
//   DSHOT300    a digital 300 kbit/s frame, 16 bits per motor update,
//               clocked out by a PIO state machine. No calibration and
//               no pulse-width drift.
//   DSHOT600    the same digital frame at 600 kbit/s, for ESCs that
//               accept the faster rate.
//
// DShot is a native-only protocol: it uses a PIO block, which the SITL
// host build does not have, so the simulated HAL keeps treating the
// motor command as a pulse width. Servos always take standard PWM.
// Arming requires the throttle stick at or below ARM_THROTTLE_MAX_US.
// ESC calibration is a one-shot bench routine, kept off for flight
// builds, and is meaningless for DShot (which has no analog endpoints).
//
// The control core and the arm logic work in microseconds for every
// protocol. The pulse widths below are the abstraction the rest of the
// firmware sees. For DShot the native output stage converts that pulse
// to a 0 or 48 to 2047 throttle value: a width below ESC_IDLE_PULSE_US
// (the disarm pulse) becomes the DShot motor-stop command 0, and the
// idle-to-max band maps onto the 48 to 2047 throttle range.

#define MOTOR_PROTOCOL_ONESHOT125 0
#define MOTOR_PROTOCOL_PWM        1
#define MOTOR_PROTOCOL_DSHOT300   2
#define MOTOR_PROTOCOL_DSHOT600   3

#ifndef MOTOR_PROTOCOL
  #define MOTOR_PROTOCOL MOTOR_PROTOCOL_PWM
#endif

// True for either DShot rate. Shared by the HAL output stage and the bidir
// and dynamic-notch guards below.
#if MOTOR_PROTOCOL == MOTOR_PROTOCOL_DSHOT300 || \
    MOTOR_PROTOCOL == MOTOR_PROTOCOL_DSHOT600
  #define MOTOR_PROTOCOL_IS_DSHOT 1
#else
  #define MOTOR_PROTOCOL_IS_DSHOT 0
#endif

#define ENABLE_ESC_CALIBRATION 0

// ENABLE_DSHOT_BIDIR turns on bidirectional DShot: after each command frame
// the ESC replies on the same wire with its electrical RPM, which feeds the
// RPM-tracking dynamic notch. It needs a DShot protocol and a BLHeli_S or
// BLHeli_32 ESC flashed for bidirectional telemetry. MOTOR_POLE_PAIRS (below)
// converts the reported eRPM to a mechanical frequency. Native-only. The
// frame encode and the eRPM decode are unit-tested, but the PIO receive
// timing is bench-pending and unverified in v1, so this defaults off.
#define ENABLE_DSHOT_BIDIR 0

#if ENABLE_DSHOT_BIDIR && !MOTOR_PROTOCOL_IS_DSHOT
  #error "ENABLE_DSHOT_BIDIR needs MOTOR_PROTOCOL_DSHOT300 or _DSHOT600."
#endif

namespace cp {

#if MOTOR_PROTOCOL == MOTOR_PROTOCOL_PWM
constexpr uint16_t ESC_MAX_PULSE_US    = 2000; // Standard PWM full throttle.
constexpr uint16_t ESC_IDLE_PULSE_US   = 1000; // Standard PWM zero throttle.
constexpr uint16_t ESC_DISARM_PULSE_US = 1000; // Motor stopped when disarmed.
#elif MOTOR_PROTOCOL == MOTOR_PROTOCOL_ONESHOT125
constexpr uint16_t ESC_MAX_PULSE_US    = 250;  // OneShot125 full throttle.
constexpr uint16_t ESC_IDLE_PULSE_US   = 125;  // OneShot125 zero throttle.
constexpr uint16_t ESC_DISARM_PULSE_US = 120;  // Below valid range: no signal.
#else  // DShot300 or DShot600.
constexpr uint16_t ESC_MAX_PULSE_US    = 2000; // Maps to DShot throttle 2047.
constexpr uint16_t ESC_IDLE_PULSE_US   = 1000; // Maps to DShot throttle 48.
constexpr uint16_t ESC_DISARM_PULSE_US = 0;    // Below idle: DShot motor-stop 0.
#endif
constexpr uint16_t ARM_THROTTLE_MAX_US = 1050; // Throttle-idle gate for arming.

#if MOTOR_PROTOCOL == MOTOR_PROTOCOL_DSHOT300
constexpr uint32_t DSHOT_BITRATE_HZ = 300000;  // DShot300 line rate.
#elif MOTOR_PROTOCOL == MOTOR_PROTOCOL_DSHOT600
constexpr uint32_t DSHOT_BITRATE_HZ = 600000;  // DShot600 line rate.
#endif

// Magnet pole pairs of the motors, for converting bidirectional-DShot eRPM
// to mechanical RPM and frequency: mechanical_hz = eRPM / (60 * pole_pairs).
// A common 14-magnet outrunner has 7 pole pairs. Set this for your motor.
constexpr uint32_t MOTOR_POLE_PAIRS = 7;

constexpr uint16_t SERVO_MIN_US = 1000;        // Servo PWM at command 0.
constexpr uint16_t SERVO_MAX_US = 2000;        // Servo PWM at command 1.

}  // namespace cp

// ===========================================================================
// Per-servo output config
// ===========================================================================
// Optional per-surface trim in the output stage, so servo direction, center,
// and travel are set in the firmware instead of mechanically or on the
// transmitter. Indexed by the airframe's SERVO_* slot. Off by default, when
// the single SERVO_MIN_US / SERVO_MAX_US range above applies to every servo.
//
// reverse  flips the command, for a servo mounted or linked backward.
// subtrim  shifts the center, in microseconds.
// endpoints  set the per-servo travel (the pulse at command 0 and command 1).
//
// Expo and stick rates are pilot input shaping and stay on the transmitter,
// not here, since this stage sees the already-mixed surface command.

#define ENABLE_SERVO_CONFIG 0

namespace cp {

// Sized to the largest servo count any airframe uses. Output.cpp static-asserts
// that the active airframe's N_SERVOS fits.
constexpr bool     SERVO_REVERSE[6]         = {false, false, false, false, false, false};
constexpr int16_t  SERVO_SUBTRIM_US[6]      = {0, 0, 0, 0, 0, 0};
constexpr uint16_t SERVO_ENDPOINT_MIN_US[6] = {1000, 1000, 1000, 1000, 1000, 1000};
constexpr uint16_t SERVO_ENDPOINT_MAX_US[6] = {2000, 2000, 2000, 2000, 2000, 2000};

// Absolute safe pulse clamp applied after reverse, travel, and subtrim, so a
// trim can never command an out-of-range pulse.
constexpr uint16_t SERVO_ABS_MIN_US = 800;
constexpr uint16_t SERVO_ABS_MAX_US = 2200;

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
// LED flasher (aircraft lighting)
// ===========================================================================
// A built-in switched LED output on one spare GPIO, blinking in an aircraft
// lighting pattern, driven from the main loop with no user hook required. The
// pin is a logic on/off signal, so the same output works with:
//   - a small 2-pin LED through a series resistor (direct drive),
//   - a high-power LED, up to about 1 W, through a low-side N-channel MOSFET
//     with the LED powered from a separate 5 V rail, or
//   - a 3-pin LED module that takes a signal plus its own 5 V and ground.
// It does not speak the WS2812 data protocol, so it is not for addressable
// strips. Off by default, which leaves the pin free for other use.

#define LED_FLASH_STROBE 0   // two quick flashes then a pause (airliner strobe)
#define LED_FLASH_BEACON 1   // one flash about once a second (anti-collision)
#define LED_FLASH_STEADY 2   // always on (position or landing light)

#define ENABLE_LED_FLASHER  0
#define LED_FLASHER_PATTERN LED_FLASH_STROBE

namespace cp {

// GPIO the flasher drives. Only used when ENABLE_LED_FLASHER is 1. Set it to a
// pin that is free on your board and airframe: there is no pin-conflict check,
// so do not pick a motor, servo, receiver, or I2C pin. GP3 is free on the
// Caribou twin but is the rudder servo on the NEO single-plane profile, so
// check your pin map.
constexpr uint8_t LED_FLASHER_PIN = 3;

// Drive polarity. true: a high pin lights the LED (a direct LED, or a low-side
// MOSFET that conducts when the gate is high). false: inverted, for a driver
// that lights on a low signal.
constexpr bool LED_FLASHER_ACTIVE_HIGH = true;

// Pattern timing in milliseconds. The strobe is two short flashes then a dark
// gap to the end of the period. The beacon is one flash per period.
constexpr uint32_t LED_STROBE_FLASH_MS  = 60;    // each strobe flash on-time
constexpr uint32_t LED_STROBE_GAP_MS    = 110;   // dark gap between the two flashes
constexpr uint32_t LED_STROBE_PERIOD_MS = 1400;  // full strobe cycle
constexpr uint32_t LED_BEACON_PERIOD_MS = 1000;  // full beacon cycle
constexpr uint32_t LED_BEACON_ON_MS     = 130;   // beacon flash on-time

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
//
// GYRO_LPF_CUTOFF_HZ is a first-order low-pass on the gyro, applied once in
// the estimator so the attitude filter and every controller see the same
// filtered body rates. It keeps motor and propeller vibration off the
// derivative term, which amplifies high-frequency noise the most. A lower
// cutoff filters harder but adds phase lag. 0 disables it. A clean prop-free
// bench needs little filtering; a vibrating multirotor needs more.
//
// GYRO_NOTCH_CENTER_HZ adds a narrow notch on the gyro at a known vibration
// peak (the dominant motor or propeller frequency, read from a flight-log
// FFT). It runs before the low-pass. 0 disables it, which is the right
// default until a peak has been measured for the airframe. GYRO_NOTCH_Q sets
// the width: higher is narrower and adds less phase lag away from the peak.
//
// ENABLE_DYNAMIC_NOTCH moves that gyro notch in real time to track the motor
// frequency reported by bidirectional DShot, instead of sitting at a fixed
// GYRO_NOTCH_CENTER_HZ. The motor vibration peak rises and falls with
// throttle, so a tracking notch stays on the peak across the throttle range
// where a fixed notch only catches one RPM. It needs ENABLE_DSHOT_BIDIR and a
// DShot protocol; with no eRPM telemetry it does nothing and the fixed notch
// (if any) stays in effect. The tracking frequency is the mean of the motor
// frequencies, clamped to [DYN_NOTCH_MIN_HZ, DYN_NOTCH_MAX_HZ] and slew-rate
// limited by DYN_NOTCH_MAX_SLEW_HZ_PER_S so the notch never jumps.

#define ENABLE_DYNAMIC_NOTCH 0

namespace cp {

constexpr float MADGWICK_BETA = 0.10f;

constexpr float GYRO_LPF_CUTOFF_HZ   = 90.0f;
constexpr float GYRO_NOTCH_CENTER_HZ = 0.0f;  // 0 = disabled until measured.
constexpr float GYRO_NOTCH_Q         = 3.0f;

// Dynamic (RPM-tracking) gyro notch. Used only when ENABLE_DYNAMIC_NOTCH is 1.
constexpr float    DYN_NOTCH_MIN_HZ            = 80.0f;   // floor, ignores ground idle
constexpr float    DYN_NOTCH_MAX_HZ           = 500.0f;  // ceiling, below the loop Nyquist
constexpr float    DYN_NOTCH_Q                = 3.0f;    // width, as for the fixed notch
constexpr float    DYN_NOTCH_MAX_SLEW_HZ_PER_S = 2000.0f; // limits how fast the center moves
constexpr uint32_t DYN_NOTCH_UPDATE_DIV       = 4;       // retune every Nth loop tick

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

// Extra low-pass on the derivative path, on top of the gyro low-pass in the
// estimator. The derivative amplifies high-frequency noise the most, so it
// gets its own, usually lower, cutoff. 0 disables it and the D term then uses
// the estimator-filtered rate directly.
constexpr float DTERM_LPF_CUTOFF_HZ = 60.0f;

}  // namespace cp

// ===========================================================================
// Multirotor rate control (quad)
// ===========================================================================
// The quad's acro (rate) mode. The sticks command a body rate up to
// MAX_ACRO_RATE_DPS at full deflection, and a per-axis proportional-plus-
// integral loop drives the rate error to zero. The stabilizer-mode switch
// (CHANNEL_STAB) selects it: low is self-leveling angle mode, high is this
// rate mode. Gains are provisional, set by bench tuning. The integral gains
// start at zero and are tuned last, the same discipline as the angle PID.

namespace cp {

constexpr float MAX_ACRO_RATE_DPS = 360.0f;  // body rate at full stick.

constexpr float KP_RATE_ROLL  = 0.0025f;
constexpr float KI_RATE_ROLL  = 0.0f;
constexpr float KP_RATE_PITCH = 0.0025f;
constexpr float KI_RATE_PITCH = 0.0f;
constexpr float KP_RATE_YAW   = 0.0040f;
constexpr float KI_RATE_YAW   = 0.0f;

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

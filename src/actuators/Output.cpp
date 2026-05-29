// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/actuators/Output.h"

#include "src/Config.h"
#include "src/hal/Hal.h"

// Actuator output stage and arm logic. See
// docs/developer-guide/algorithms.md.
//
// Arming is a safety function. The boot state is NOT_ARMED. Disarm
// takes effect unconditionally and immediately whenever the arm switch
// leaves the arm position. Arming is permitted only when the aircraft
// is currently disarmed, the switch is in the arm position, and the
// throttle stick is at idle, so the aircraft cannot arm straight into
// a spun-up throttle command.
//
// When disarmed the motor pulses go out at ESC_DISARM_PULSE_US, the
// motor-stopped width, so the motors do not spin. Servos respond
// regardless of arm state.

namespace cp::actuators {

namespace {

// The arm switch is in the arm position when the arm channel is below the
// channel midpoint.
constexpr uint16_t kArmSwitchThresholdUs = RC_MID_US;

// ARM_THROTTLE_MAX_US expressed as a normalized throttle fraction, to
// compare against the normalized throttle the stabilizer works in.
constexpr float kArmThrottleMaxNorm =
    static_cast<float>(ARM_THROTTLE_MAX_US - RC_MIN_US) /
    static_cast<float>(RC_MAX_US - RC_MIN_US);

ArmState     s_arm    = ArmState::NOT_ARMED;
LatestPulses s_pulses = {};

// True once the arm switch has been seen in the disarm position since
// boot. Arming is refused until then, so a board powered up with the
// switch already in the arm position cannot arm without a deliberate
// disarm-to-arm transition by the pilot.
bool s_arm_ready = false;

float clampf(float v, float lo, float hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

// Normalized motor command [0, 1] to a motor pulse width, in the
// configured ESC pulse range (OneShot125 or standard PWM).
uint16_t motorPulseUs(float command) {
  const float c = clampf(command, 0.0f, 1.0f);
  const float span = static_cast<float>(ESC_MAX_PULSE_US - ESC_IDLE_PULSE_US);
  return static_cast<uint16_t>(static_cast<float>(ESC_IDLE_PULSE_US) +
                               c * span + 0.5f);
}

// Normalized servo command [0, 1] for servo `idx` to a PWM pulse width. With
// ENABLE_SERVO_CONFIG the per-servo reverse, travel endpoints, and subtrim
// apply, with a final clamp to the absolute safe range. Otherwise the single
// SERVO_MIN_US / SERVO_MAX_US range applies to every servo.
uint16_t servoPulseUs(uint8_t idx, float command) {
  float c = clampf(command, 0.0f, 1.0f);
#if ENABLE_SERVO_CONFIG
  static_assert(cp::airframes::N_SERVOS <=
                    sizeof(SERVO_REVERSE) / sizeof(SERVO_REVERSE[0]),
                "Per-servo config arrays are too small for this airframe.");
  if (SERVO_REVERSE[idx]) {
    c = 1.0f - c;
  }
  const float lo   = static_cast<float>(SERVO_ENDPOINT_MIN_US[idx]);
  const float hi   = static_cast<float>(SERVO_ENDPOINT_MAX_US[idx]);
  float us = lo + c * (hi - lo) + static_cast<float>(SERVO_SUBTRIM_US[idx]) +
             0.5f;
  if (us < static_cast<float>(SERVO_ABS_MIN_US)) {
    us = static_cast<float>(SERVO_ABS_MIN_US);
  } else if (us > static_cast<float>(SERVO_ABS_MAX_US)) {
    us = static_cast<float>(SERVO_ABS_MAX_US);
  }
  return static_cast<uint16_t>(us);
#else
  const float span = static_cast<float>(SERVO_MAX_US - SERVO_MIN_US);
  return static_cast<uint16_t>(static_cast<float>(SERVO_MIN_US) +
                               c * span + 0.5f);
#endif
}

}  // anonymous namespace

void init() {
  s_arm       = ArmState::NOT_ARMED;
  s_arm_ready = false;
  s_pulses    = LatestPulses{};
  cp::hal::out_init();
}

void update(const cp::airframes::Output& mix,
            float throttle_norm,
            uint16_t arm_us,
            bool prearm_ok) {
  // Arm state machine. Disarm is unconditional. Arming needs the switch
  // in the arm position, the aircraft currently disarmed, the throttle
  // at idle, the switch to have been seen in the disarm position at least
  // once since boot (so a power-up with the switch already armed cannot arm
  // without a deliberate switch transition), and the pre-arm checks to pass.
  const bool switch_in_arm_position = arm_us < kArmSwitchThresholdUs;
  if (!switch_in_arm_position) {
    s_arm       = ArmState::NOT_ARMED;
    s_arm_ready = true;
  } else if (s_arm == ArmState::NOT_ARMED && s_arm_ready && prearm_ok &&
             throttle_norm <= kArmThrottleMaxNorm) {
    s_arm = ArmState::ARMED;
  }

  // Motors. Armed motors follow the mixer. Disarmed motors emit the
  // motor-stopped disarm pulse so the motors do not spin.
  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    const uint16_t pulse = (s_arm == ArmState::ARMED)
                               ? motorPulseUs(mix.motor[i])
                               : ESC_DISARM_PULSE_US;
    s_pulses.motor_us[i] = pulse;
    cp::hal::out_set_motor_us(i, pulse);
  }

  // Servos respond in any arm state.
  for (uint8_t i = 0; i < cp::airframes::N_SERVOS; ++i) {
    const uint16_t pulse = servoPulseUs(i, mix.servo[i]);
    s_pulses.servo_us[i] = pulse;
    cp::hal::out_set_servo_us(i, pulse);
  }

  // Commit the motor pulses: a synchronous OneShot125 burst, or a no-op
  // under standard PWM where the Servo schedule already drives the pins.
  cp::hal::out_commit_motors();
}

ArmState arm_state() {
  return s_arm;
}

const LatestPulses& latest_pulses() {
  return s_pulses;
}

}  // namespace cp::actuators

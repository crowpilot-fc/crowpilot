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
// When disarmed the motor pulses go out at the sub-valid disarm width,
// so the ESCs treat them as no signal and stay silent. Servos respond
// regardless of arm state.

namespace cp::actuators {

namespace {

// The arm switch is in the arm position when channel 5 is below the
// channel midpoint.
constexpr uint16_t kArmSwitchThresholdUs = RC_MID_US;

// ARM_THROTTLE_MAX_US expressed as a normalized throttle fraction, to
// compare against the normalized throttle the stabilizer works in.
constexpr float kArmThrottleMaxNorm =
    static_cast<float>(ARM_THROTTLE_MAX_US - RC_MIN_US) /
    static_cast<float>(RC_MAX_US - RC_MIN_US);

ArmState     s_arm    = ArmState::NOT_ARMED;
LatestPulses s_pulses = {};

float clampf(float v, float lo, float hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

// Normalized motor command [0, 1] to a OneShot125 pulse width.
uint16_t motorPulseUs(float command) {
  const float c = clampf(command, 0.0f, 1.0f);
  const float span = static_cast<float>(ESC_MAX_PULSE_US - ESC_IDLE_PULSE_US);
  return static_cast<uint16_t>(static_cast<float>(ESC_IDLE_PULSE_US) +
                               c * span);
}

// Normalized servo command [0, 1] to a servo PWM pulse width.
uint16_t servoPulseUs(float command) {
  const float c = clampf(command, 0.0f, 1.0f);
  const float span = static_cast<float>(SERVO_MAX_US - SERVO_MIN_US);
  return static_cast<uint16_t>(static_cast<float>(SERVO_MIN_US) +
                               c * span);
}

}  // anonymous namespace

void init() {
  s_arm    = ArmState::NOT_ARMED;
  s_pulses = LatestPulses{};
  cp::hal::out_init();
}

void update(const cp::airframes::Output& mix,
            float throttle_norm,
            uint16_t ch5_us) {
  // Arm state machine. Disarm is unconditional. Arming needs the switch
  // in the arm position, the aircraft currently disarmed, and the
  // throttle at idle.
  const bool switch_in_arm_position = ch5_us < kArmSwitchThresholdUs;
  if (!switch_in_arm_position) {
    s_arm = ArmState::NOT_ARMED;
  } else if (s_arm == ArmState::NOT_ARMED &&
             throttle_norm <= kArmThrottleMaxNorm) {
    s_arm = ArmState::ARMED;
  }

  // Motors. Armed motors follow the mixer. Disarmed motors emit the
  // sub-valid disarm pulse so the ESCs stay silent.
  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    const uint16_t pulse = (s_arm == ArmState::ARMED)
                               ? motorPulseUs(mix.motor[i])
                               : ESC_DISARM_PULSE_US;
    s_pulses.motor_us[i] = pulse;
    cp::hal::out_set_motor_us(i, pulse);
  }

  // Servos respond in any arm state.
  for (uint8_t i = 0; i < cp::airframes::N_SERVOS; ++i) {
    const uint16_t pulse = servoPulseUs(mix.servo[i]);
    s_pulses.servo_us[i] = pulse;
    cp::hal::out_set_servo_us(i, pulse);
  }

  // Emit the synchronous OneShot125 burst on the motor pins.
  cp::hal::out_commit_motors();
}

ArmState arm_state() {
  return s_arm;
}

const LatestPulses& latest_pulses() {
  return s_pulses;
}

}  // namespace cp::actuators

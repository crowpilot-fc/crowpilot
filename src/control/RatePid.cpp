// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/control/RatePid.h"

#include "src/Config.h"

namespace cp::control::rate {

namespace {

constexpr float kOutputLimit = 1.0f;

float s_integral_roll  = 0.0f;
float s_integral_pitch = 0.0f;
float s_integral_yaw   = 0.0f;

Output s_out = {0.0f, 0.0f, 0.0f};

float clampf(float v, float lo, float hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

// One axis: proportional plus anti-windup-bounded integral on the rate
// error. No derivative term yet: the derivative of a rate is angular
// acceleration, which is noisy, so it is a later refinement once a flying
// quad shows it is needed. Integral-term contribution is bounded to
// PID_INTEGRAL_LIMIT of the normalized output, the same as the angle PID.
float axisStep(float error, float kp, float ki, bool flying, float dt_s,
               float& integral) {
  if (flying && ki > 1.0e-6f) {
    integral += error * dt_s;
    const float integral_max = PID_INTEGRAL_LIMIT / ki;
    integral = clampf(integral, -integral_max, integral_max);
  } else {
    integral = 0.0f;
  }
  return clampf(kp * error + ki * integral, -kOutputLimit, kOutputLimit);
}

}  // anonymous namespace

void init() {
  s_integral_roll  = 0.0f;
  s_integral_pitch = 0.0f;
  s_integral_yaw   = 0.0f;
  s_out = Output{0.0f, 0.0f, 0.0f};
}

void update(float roll_rate_sp_dps,
            float pitch_rate_sp_dps,
            float yaw_rate_sp_dps,
            const cp::estimation::attitude::BodyRates& rates,
            bool flying,
            float dt_s) {
  s_out.roll = axisStep(roll_rate_sp_dps - rates.roll_dps,
                        KP_RATE_ROLL, KI_RATE_ROLL, flying, dt_s,
                        s_integral_roll);
  s_out.pitch = axisStep(pitch_rate_sp_dps - rates.pitch_dps,
                         KP_RATE_PITCH, KI_RATE_PITCH, flying, dt_s,
                         s_integral_pitch);
  s_out.yaw = axisStep(yaw_rate_sp_dps - rates.yaw_dps,
                       KP_RATE_YAW, KI_RATE_YAW, flying, dt_s,
                       s_integral_yaw);
}

const Output& output() {
  return s_out;
}

}  // namespace cp::control::rate

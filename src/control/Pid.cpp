// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/control/Pid.h"

#include "src/Config.h"
#include "src/libs/Filter.h"
#include "src/params/LiveTune.h"
#include "src/params/Params.h"

namespace cp::control::pid {

namespace {

// Normalized demand range handed to the mixer.
constexpr float kOutputLimit = 1.0f;

// One axis integrator.
float s_integral_roll  = 0.0f;
float s_integral_pitch = 0.0f;
float s_integral_yaw   = 0.0f;

// Per-axis low-pass on the derivative path. The gyro is already filtered in
// the estimator; this is the extra, lower-cutoff stage on the rate the D term
// reads. See DTERM_LPF_CUTOFF_HZ in Config.h.
cp::libs::filter::Pt1 s_dterm_lpf_roll;
cp::libs::filter::Pt1 s_dterm_lpf_pitch;
cp::libs::filter::Pt1 s_dterm_lpf_yaw;

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

// Linear interpolation. At t = 0 returns the forward-flight value, at
// t = 1 the hover value, matching the fader convention.
float schedule(float forward, float hover, float fader) {
  return forward + (hover - forward) * fader;
}

// One axis PID step. Updates the integrator and returns the clamped
// normalized demand. The derivative term uses the measured body rate.
float axisStep(float error, float body_rate,
               float kp, float ki, float kd,
               bool flying, float dt_s, float& integral) {
  // Integrate only while flying, and only when the integral term is in
  // use. Anti-windup bounds the integral-term contribution Ki*integral
  // to +/- PID_INTEGRAL_LIMIT of the normalized output, so a saturated
  // integrator cannot dominate the demand.
  if (flying && ki > 1.0e-6f) {
    integral += error * dt_s;
    const float integral_max = PID_INTEGRAL_LIMIT / ki;
    integral = clampf(integral, -integral_max, integral_max);
  } else {
    integral = 0.0f;
  }

  const float demand = kp * error + ki * integral - kd * body_rate;
  return clampf(demand, -kOutputLimit, kOutputLimit);
}

}  // anonymous namespace

void init() {
  s_integral_roll  = 0.0f;
  s_integral_pitch = 0.0f;
  s_integral_yaw   = 0.0f;
  s_dterm_lpf_roll.reset();
  s_dterm_lpf_pitch.reset();
  s_dterm_lpf_yaw.reset();
  s_out = Output{0.0f, 0.0f, 0.0f};
}

void update(const cp::estimation::attitude::Euler& attitude_error,
            const cp::estimation::attitude::BodyRates& body_rates,
            float yaw_rate_setpoint_dps,
            float fader,
            bool flying,
            float dt_s) {
  namespace pr = cp::params;

  // Live-tune multipliers, applied to every blended P and D gain so a
  // transmitter knob can scale the gains in flight.
  const float kp_mult = cp::params::live::kpMultiplier();
  const float kd_mult = cp::params::live::kdMultiplier();

  // Regime-scheduled gains. P and D interpolate between the hover and
  // forward-flight sets and are then scaled by the live-tune
  // multipliers. I is shared and not live-tuned. All read from the
  // runtime registry.
  const float kp_roll  = schedule(pr::get(pr::KP_ROLL_FF),
                                  pr::get(pr::KP_ROLL_HOVER), fader) * kp_mult;
  const float kd_roll  = schedule(pr::get(pr::KD_ROLL_FF),
                                  pr::get(pr::KD_ROLL_HOVER), fader) * kd_mult;
  const float kp_pitch = schedule(pr::get(pr::KP_PITCH_FF),
                                  pr::get(pr::KP_PITCH_HOVER), fader) * kp_mult;
  const float kd_pitch = schedule(pr::get(pr::KD_PITCH_FF),
                                  pr::get(pr::KD_PITCH_HOVER), fader) * kd_mult;
  const float kp_yaw   = schedule(pr::get(pr::KP_YAW_FF),
                                  pr::get(pr::KP_YAW_HOVER), fader) * kp_mult;
  const float kd_yaw   = schedule(pr::get(pr::KD_YAW_FF),
                                  pr::get(pr::KD_YAW_HOVER), fader) * kd_mult;
  const float ki_roll  = pr::get(pr::KI_ROLL);
  const float ki_pitch = pr::get(pr::KI_PITCH);
  const float ki_yaw   = pr::get(pr::KI_YAW);

  // Derivative-path rates: the estimator-filtered body rates passed through
  // the extra D-term low-pass. The error terms still use the unfiltered
  // (estimator-level) rates so only the D contribution gets the extra lag.
  const float d_roll  = s_dterm_lpf_roll.apply(body_rates.roll_dps, DTERM_LPF_CUTOFF_HZ, dt_s);
  const float d_pitch = s_dterm_lpf_pitch.apply(body_rates.pitch_dps, DTERM_LPF_CUTOFF_HZ, dt_s);
  const float d_yaw   = s_dterm_lpf_yaw.apply(body_rates.yaw_dps, DTERM_LPF_CUTOFF_HZ, dt_s);

  // Roll and pitch stabilize on attitude error.
  s_out.roll = axisStep(attitude_error.roll_deg, d_roll,
                        kp_roll, ki_roll, kd_roll,
                        flying, dt_s, s_integral_roll);
  s_out.pitch = axisStep(attitude_error.pitch_deg, d_pitch,
                         kp_pitch, ki_pitch, kd_pitch,
                         flying, dt_s, s_integral_pitch);

  // Yaw stabilizes on rate error.
  const float yaw_rate_error = yaw_rate_setpoint_dps - body_rates.yaw_dps;
  s_out.yaw = axisStep(yaw_rate_error, d_yaw,
                       kp_yaw, ki_yaw, kd_yaw,
                       flying, dt_s, s_integral_yaw);
}

const Output& output() {
  return s_out;
}

}  // namespace cp::control::pid

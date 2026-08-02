// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/control/Gimbal.h"

#include "src/Config.h"
#include "src/params/Params.h"

#if BUILD_TARGET == BUILD_TARGET_NATIVE
#include <Arduino.h>
#include <Servo.h>
#endif

namespace cp::control::gimbal {

namespace {

#if BUILD_TARGET == BUILD_TARGET_NATIVE
Servo s_servo_pan;
Servo s_servo_tilt;
#endif

bool s_active = false;

// Washout state: a first-order low-pass per axis. The high-passed rate the
// damping actually uses is (raw - low-pass), so a sustained rotation decays
// out of the term while fast disturbances pass through.
float s_yaw_lp   = 0.0f;
float s_pitch_lp = 0.0f;

inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

}  // anonymous namespace

void init() {
  s_active   = false;
  s_yaw_lp   = 0.0f;
  s_pitch_lp = 0.0f;

#if BOARD_HAS_GIMBAL
  namespace pr = cp::params;
  const bool enable = pr::get(pr::GIMBAL_ENABLE) >= 0.5f;
  if (!enable) {
    // Disabled. Leave the pins untouched so they remain free GPIO for any
    // user-sketch or future feature that wants them.
    return;
  }

#if BUILD_TARGET == BUILD_TARGET_NATIVE
  // attach() returns 0 when the core has no PIO state machine left. Claiming
  // the gimbal is active after a failed attach would report a working camera
  // mount that never moves, so both pins have to succeed.
  const int pan_ch  = s_servo_pan.attach(cp::PIN_GIMBAL_PAN);
  const int tilt_ch = s_servo_tilt.attach(cp::PIN_GIMBAL_TILT);
  if (pan_ch == 0 || tilt_ch == 0) {
    if (pan_ch  != 0) s_servo_pan.detach();
    if (tilt_ch != 0) s_servo_tilt.detach();
    return;  // s_active stays false
  }
  // Center both servos so the gimbal does not snap on power-up.
  s_servo_pan.writeMicroseconds(1500);
  s_servo_tilt.writeMicroseconds(1500);
#endif
  s_active = true;
#endif  // BOARD_HAS_GIMBAL
}

bool active() {
  return s_active;
}

void tick(const uint16_t* channels,
          const cp::estimation::attitude::BodyRates& rates,
          float dt_s) {
#if BOARD_HAS_GIMBAL
  if (!s_active) {
    return;
  }

  namespace pr = cp::params;

  const uint8_t ch_pan  = static_cast<uint8_t>(pr::get(pr::GIMBAL_CHANNEL_PAN)  + 0.5f);
  const uint8_t ch_tilt = static_cast<uint8_t>(pr::get(pr::GIMBAL_CHANNEL_TILT) + 0.5f);
  if (ch_pan == 0 || ch_pan > 16 || ch_tilt == 0 || ch_tilt > 16) {
    return;  // No headtracker channel assigned; do nothing.
  }

  const float k_pan      = pr::get(pr::GIMBAL_GAIN_PAN);
  const float k_tilt     = pr::get(pr::GIMBAL_GAIN_TILT);
  const float trim_pan   = pr::get(pr::GIMBAL_TRIM_PAN);
  const float trim_tilt  = pr::get(pr::GIMBAL_TRIM_TILT);
  const float min_pan    = pr::get(pr::GIMBAL_MIN_PAN_US);
  const float max_pan    = pr::get(pr::GIMBAL_MAX_PAN_US);
  const float min_tilt   = pr::get(pr::GIMBAL_MIN_TILT_US);
  const float max_tilt   = pr::get(pr::GIMBAL_MAX_TILT_US);
  const bool  rev_pan    = pr::get(pr::GIMBAL_REVERSE_PAN)  >= 0.5f;
  const bool  rev_tilt   = pr::get(pr::GIMBAL_REVERSE_TILT) >= 0.5f;

  float pan_raw  = static_cast<float>(channels[ch_pan  - 1]);
  float tilt_raw = static_cast<float>(channels[ch_tilt - 1]);

  if (rev_pan)  pan_raw  = 3000.0f - pan_raw;
  if (rev_tilt) tilt_raw = 3000.0f - tilt_raw;

  // Rate damping. Subtracting K * gyro counter-rotates the camera against
  // the plane's motion. When the plane is still, both terms are zero and
  // the servo follows the headtracker exactly.
  // Pan corrects yaw rate; tilt corrects pitch rate. Roll is not
  // compensated (a 2-axis gimbal cannot).
  //
  // The reverse flags describe a servo whose linkage runs the opposite way,
  // so the damping term has to flip with them. Reversing only the
  // headtracker input (as this did originally) leaves the gyro correction
  // pushing the same physical direction as the disturbance on a reversed
  // axis, which turns damping into amplification and makes the camera chase
  // the shake instead of cancelling it. Trim is deliberately not flipped: it
  // is an output-space offset applied after the reversal.
  const float pan_damp_sign  = rev_pan  ? -1.0f : 1.0f;
  const float tilt_damp_sign = rev_tilt ? -1.0f : 1.0f;

  // Washout. Track a slow low-pass of each rate and damp on the difference,
  // so a sustained commanded rotation (a loop, a long turn) falls out of the
  // correction while turbulence and airframe jitter still get cancelled.
  // A time constant of 0 disables it and restores raw rate damping.
  const float washout_tc = pr::get(pr::GIMBAL_WASHOUT_TC_S);
  float yaw_damp   = rates.yaw_dps;
  float pitch_damp = rates.pitch_dps;
  if (washout_tc > 0.0f && dt_s > 0.0f) {
    const float alpha = dt_s / (washout_tc + dt_s);
    s_yaw_lp   += (rates.yaw_dps   - s_yaw_lp)   * alpha;
    s_pitch_lp += (rates.pitch_dps - s_pitch_lp) * alpha;
    yaw_damp   = rates.yaw_dps   - s_yaw_lp;
    pitch_damp = rates.pitch_dps - s_pitch_lp;
  }

  const float pan_us  = clampf(
      pan_raw  + trim_pan  - pan_damp_sign  * k_pan  * yaw_damp,
      min_pan,  max_pan);
  const float tilt_us = clampf(
      tilt_raw + trim_tilt - tilt_damp_sign * k_tilt * pitch_damp,
      min_tilt, max_tilt);

#if BUILD_TARGET == BUILD_TARGET_NATIVE
  s_servo_pan.writeMicroseconds(static_cast<int>(pan_us  + 0.5f));
  s_servo_tilt.writeMicroseconds(static_cast<int>(tilt_us + 0.5f));
#else
  (void)pan_us;
  (void)tilt_us;
#endif
#else
  (void)channels;
  (void)rates;
  (void)dt_s;
#endif  // BOARD_HAS_GIMBAL
}

}  // namespace cp::control::gimbal

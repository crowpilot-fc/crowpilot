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

inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

}  // anonymous namespace

void init() {
  s_active = false;

#ifdef PIN_GIMBAL_PAN
  namespace pr = cp::params;
  const bool enable = pr::get(pr::GIMBAL_ENABLE) >= 0.5f;
  if (!enable) {
    // Disabled. Leave the pins untouched so they remain free GPIO for any
    // user-sketch or future feature that wants them.
    return;
  }

#if BUILD_TARGET == BUILD_TARGET_NATIVE
  s_servo_pan.attach(cp::PIN_GIMBAL_PAN);
  s_servo_tilt.attach(cp::PIN_GIMBAL_TILT);
  // Center both servos so the gimbal does not snap on power-up.
  s_servo_pan.writeMicroseconds(1500);
  s_servo_tilt.writeMicroseconds(1500);
#endif
  s_active = true;
#endif  // PIN_GIMBAL_PAN
}

bool active() {
  return s_active;
}

void tick(const uint16_t* channels,
          const cp::estimation::attitude::BodyRates& rates) {
#ifdef PIN_GIMBAL_PAN
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
  const float pan_us  = clampf(pan_raw  + trim_pan  - k_pan  * rates.yaw_dps,   min_pan,  max_pan);
  const float tilt_us = clampf(tilt_raw + trim_tilt - k_tilt * rates.pitch_dps, min_tilt, max_tilt);

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
#endif  // PIN_GIMBAL_PAN
}

}  // namespace cp::control::gimbal

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/control/PlaneStab.h"

#include "src/Config.h"

namespace cp::control::plane_stab {

namespace {

Output s_output = {};

// Altitude-hold state. The target is captured on the rising edge of the
// alt-hold switch. The climb rate is a low-passed derivative of the baro
// altitude so the D term does not chase per-sample baro noise.
float s_alt_target_m   = 0.0f;
bool  s_alt_hold_armed = false;
float s_prev_alt_m     = 0.0f;
bool  s_have_prev_alt  = false;
float s_climb_rate_mps = 0.0f;

inline float clampUnit(float x) {
  if (x >  1.0f) return  1.0f;
  if (x < -1.0f) return -1.0f;
  return x;
}

inline float clamp01(float x) {
  if (x > 1.0f) return 1.0f;
  if (x < 0.0f) return 0.0f;
  return x;
}

// Switch helper. A channel reads HIGH above mid-stick.
inline bool channelHigh(const uint16_t* channels, uint8_t channel_1based) {
  return channels[channel_1based - 1] > 1500;
}

}  // anonymous namespace

void init() {
  s_output         = {};
  s_alt_target_m   = 0.0f;
  s_alt_hold_armed = false;
  s_prev_alt_m     = 0.0f;
  s_have_prev_alt  = false;
  s_climb_rate_mps = 0.0f;
}

void update(const cp::estimation::attitude::Euler& euler,
            const cp::estimation::attitude::BodyRates& rates,
            const cp::control::desired::State& desired,
            float baro_altitude_m,
            bool baro_valid,
            const uint16_t* channels,
            float dt_s) {
  const bool passthrough = channelHigh(channels, CHANNEL_STAB);
  s_output.passthrough_active = passthrough;

  if (passthrough) {
    // Full manual: surfaces follow the sticks, no stabilization. The
    // desired-state passthrough values are already the normalized
    // [-1, +1] surface-command range.
    s_output.roll  = clampUnit(desired.roll_passthru);
    s_output.pitch = clampUnit(desired.pitch_passthru);
    s_output.yaw   = clampUnit(desired.yaw_passthru);
    s_output.throttle = clamp01(desired.throttle);
    s_output.alt_hold_active = false;
    s_alt_hold_armed = false;
    s_have_prev_alt  = false;
    return;
  }

  // Wing leveler. Drive the measured roll toward the pilot's roll-angle
  // setpoint. D on the measured body rate to avoid derivative kick.
  {
    const float err = desired.roll_deg - euler.roll_deg;
    s_output.roll = clampUnit(STAB_OUTPUT_SCALE *
        (KP_STAB_ROLL * err - KD_STAB_ROLL * rates.roll_dps));
  }

  // Pitch-attitude hold. Same shape on the pitch axis.
  {
    const float err = desired.pitch_deg - euler.pitch_deg;
    s_output.pitch = clampUnit(STAB_OUTPUT_SCALE *
        (KP_STAB_PITCH * err - KD_STAB_PITCH * rates.pitch_dps));
  }

  // Yaw damper. No heading hold in v1; the rudder term simply opposes the
  // measured yaw rate to take the wallow out of the airframe.
  s_output.yaw = clampUnit(STAB_OUTPUT_SCALE *
      (-KD_STAB_YAW * rates.yaw_dps));

  // Altitude hold. Optional, and only meaningful with a healthy baro.
  float throttle_out = clamp01(desired.throttle);
  bool alt_hold_active = false;

#if ENABLE_ALT_HOLD
  const bool alt_switch = channelHigh(channels, CHANNEL_ALT_HOLD);
  if (alt_switch && baro_valid) {
    // Climb-rate estimate: low-passed derivative of baro altitude.
    if (s_have_prev_alt && dt_s > 0.0f) {
      const float raw_rate = (baro_altitude_m - s_prev_alt_m) / dt_s;
      s_climb_rate_mps += ALT_CLIMB_FILTER_ALPHA *
          (raw_rate - s_climb_rate_mps);
    }
    s_prev_alt_m    = baro_altitude_m;
    s_have_prev_alt = true;

    if (!s_alt_hold_armed) {
      // Rising edge: capture the current altitude as the target.
      s_alt_target_m   = baro_altitude_m;
      s_alt_hold_armed = true;
      s_climb_rate_mps = 0.0f;
    }
    const float alt_err = s_alt_target_m - baro_altitude_m;
    throttle_out = clamp01(desired.throttle +
        KP_ALT * alt_err - KD_ALT * s_climb_rate_mps);
    alt_hold_active = true;
  } else {
    s_alt_hold_armed = false;
    s_have_prev_alt  = false;
    s_climb_rate_mps = 0.0f;
  }
#else
  (void)baro_altitude_m;
  (void)baro_valid;
  (void)dt_s;
#endif

  s_output.throttle        = throttle_out;
  s_output.alt_hold_active = alt_hold_active;
}

const Output& output() {
  return s_output;
}

}  // namespace cp::control::plane_stab

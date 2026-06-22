// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/control/PlaneStab.h"

#include <math.h>

#include "src/Config.h"
#include "src/params/LiveTune.h"
#include "src/params/Params.h"

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

// True while the main loop is forcing MANUAL passthrough because the IMU
// is unhealthy mid-flight on a fixed-wing airframe. See
// setImuLossOverride().
bool  s_imu_loss_override = false;

// Low-pass-filtered stab authority. Seeded to the param default at init so
// the first stab tick uses a sensible value before any channel data has
// arrived.
float s_authority_filt = 1.0f;

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

// Map the three positions of CHANNEL_STAB to the configured flight mode.
// If the IMU-loss override is active, force MANUAL regardless of the
// pilot's switch position. MANUAL is full pilot-passthrough, no
// stabilization terms, so the plane flies on sticks alone after IMU
// failure.
inline uint8_t selectMode(const uint16_t* channels) {
  if (s_imu_loss_override) {
    return PLANE_MODE_MANUAL;
  }
  const uint16_t us = channels[CHANNEL_STAB - 1];
  if (us < PLANE_MODE_SW_LOW_MAX_US) {
    return PLANE_MODE_SW_LOW;
  }
  if (us < PLANE_MODE_SW_MID_MAX_US) {
    return PLANE_MODE_SW_MID;
  }
  return PLANE_MODE_SW_HIGH;
}

}  // anonymous namespace

TurnFeedforward turnCoordination(float bank_deg) {
  constexpr float kDegToRad = 0.0174532925f;
  float clamped = bank_deg;
  if (clamped >  TURN_COMP_MAX_BANK_DEG) clamped =  TURN_COMP_MAX_BANK_DEG;
  if (clamped < -TURN_COMP_MAX_BANK_DEG) clamped = -TURN_COMP_MAX_BANK_DEG;
  TurnFeedforward ff;
  // Auto-rudder follows the actual bank (sin is bounded, so no clamp needed).
  ff.rudder = TURN_COORD_RUDDER_GAIN * sinf(bank_deg * kDegToRad);
  // Up-elevator to hold altitude in the turn, with the clamped bank so 1/cos
  // stays bounded near knife-edge.
  ff.pitch = PITCH_TURN_COMP_GAIN * (1.0f / cosf(clamped * kDegToRad) - 1.0f);
  return ff;
}

void init() {
  s_output         = {};
  s_alt_target_m   = 0.0f;
  s_alt_hold_armed = false;
  s_prev_alt_m     = 0.0f;
  s_have_prev_alt  = false;
  s_climb_rate_mps = 0.0f;
  s_authority_filt = cp::params::get(cp::params::GAIN_AUTHORITY);
}

void update(const cp::estimation::attitude::Euler& euler,
            const cp::estimation::attitude::BodyRates& rates,
            const cp::control::desired::State& desired,
            float baro_altitude_m,
            bool baro_valid,
            const uint16_t* channels,
            float dt_s) {
  namespace pr = cp::params;

  const uint8_t mode = selectMode(channels);
  s_output.mode = mode;
  const bool passthrough = (mode == PLANE_MODE_MANUAL);
  s_output.passthrough_active = passthrough;

  // Stab authority: pick a fresh raw value from the gain channel (or the
  // param default when no channel is assigned) and low-pass filter it. This
  // runs every tick regardless of flight mode so the filter tracks the
  // pilot's knob across manual-to-stab switches without resetting.
  {
    const uint8_t gain_ch = static_cast<uint8_t>(pr::get(pr::GAIN_CHANNEL) + 0.5f);
    float authority_raw;
    if (gain_ch == 0 || gain_ch > 16) {
      authority_raw = pr::get(pr::GAIN_AUTHORITY);
    } else {
      const float ch_us = static_cast<float>(channels[gain_ch - 1]);
      authority_raw = (ch_us - 1000.0f) * 0.001f;
      if (authority_raw < 0.0f) authority_raw = 0.0f;
      if (authority_raw > 1.0f) authority_raw = 1.0f;
    }
    const float gain_tc = pr::get(pr::GAIN_LPF_TC_S);
    const float alpha   = (gain_tc + dt_s > 0.0f) ? dt_s / (gain_tc + dt_s) : 1.0f;
    s_authority_filt = alpha * authority_raw + (1.0f - alpha) * s_authority_filt;
  }

  // Flaperon and airbrake commands from their channels, applied to the
  // ailerons by the mixer in every mode. 0 when the option is off.
#if ENABLE_FLAPERON
  s_output.flap = clamp01((static_cast<float>(channels[CHANNEL_FLAP - 1]) -
                           1000.0f) / 1000.0f);
#else
  s_output.flap = 0.0f;
#endif
#if ENABLE_CROW
  s_output.brake = clamp01((static_cast<float>(channels[CHANNEL_CROW - 1]) -
                            1000.0f) / 1000.0f);
#else
  s_output.brake = 0.0f;
#endif

  if (passthrough) {
    // Full manual: surfaces follow the sticks, no stabilization. The
    // desired-state passthrough values are already the normalized
    // [-1, +1] surface-command range.
    s_output.roll  = clampUnit(desired.roll_passthru);
    s_output.pitch = clampUnit(desired.pitch_passthru);
    s_output.yaw   = clampUnit(desired.yaw_passthru);
    s_output.throttle = clamp01(desired.throttle);
    s_output.alt_hold_active = false;
    s_output.stab_authority  = 0.0f;
    s_alt_hold_armed = false;
    s_have_prev_alt  = false;
    return;
  }

  // Gains from the runtime registry, scaled by the live-tune multipliers, so a
  // transmitter knob (or a configurator slider) adjusts them in flight. The P
  // knob scales the P gains, the D knob the D gains, exactly as for the
  // tailsitter PID. The registry value is the base; the knob is a multiplier
  // on top.
  const float kp_mult = cp::params::live::kpMultiplier();
  const float kd_mult = cp::params::live::kdMultiplier();

  const float kp_roll       = pr::get(pr::KP_ROLL_PLANE)       * kp_mult;
  const float kd_roll       = pr::get(pr::KD_ROLL_PLANE)       * kd_mult;
  const float kp_pitch      = pr::get(pr::KP_PITCH_PLANE)      * kp_mult;
  const float kd_pitch      = pr::get(pr::KD_PITCH_PLANE)      * kd_mult;
  const float kd_yaw        = pr::get(pr::KD_YAW_PLANE)        * kd_mult;
  const float kp_rate_roll  = pr::get(pr::KP_RATE_ROLL_PLANE)  * kp_mult;
  const float kp_rate_pitch = pr::get(pr::KP_RATE_PITCH_PLANE) * kp_mult;

  // Angle-mode command per axis: drive the measured attitude toward the
  // pilot's angle setpoint, D on the measured body rate to avoid derivative
  // kick. This is the wing leveler and pitch-attitude hold.
  const float angle_roll = kp_roll * (desired.roll_deg - euler.roll_deg) -
                           kd_roll * rates.roll_dps;
  const float angle_pitch =
      kp_pitch * (desired.pitch_deg - euler.pitch_deg) -
      kd_pitch * rates.pitch_dps;

  // Rate-mode command per axis: the stick commands a body rate up to
  // PLANE_RATE_MAX_DPS and a proportional law drives the rate error. No
  // self-leveling.
  const float rate_roll = kp_rate_roll *
      (desired.roll_passthru * PLANE_RATE_MAX_DPS - rates.roll_dps);
  const float rate_pitch = kp_rate_pitch *
      (desired.pitch_passthru * PLANE_RATE_MAX_DPS - rates.pitch_dps);

  float roll_cmd;
  float pitch_cmd;
  if (mode == PLANE_MODE_RATE) {
    roll_cmd  = rate_roll;
    pitch_cmd = rate_pitch;
  } else if (mode == PLANE_MODE_HORIZON) {
    // Blend by stick deflection: pure self-level at center, pure rate at full
    // stick. fabsf of the normalized stick is the rate weight.
    const float wr = fabsf(desired.roll_passthru);
    const float wp = fabsf(desired.pitch_passthru);
    roll_cmd  = (1.0f - wr) * angle_roll  + wr * rate_roll;
    pitch_cmd = (1.0f - wp) * angle_pitch + wp * rate_pitch;
  } else {  // PLANE_MODE_ANGLE, and the safe default for any other value.
    roll_cmd  = angle_roll;
    pitch_cmd = angle_pitch;
  }
  // Yaw damper, active in every stabilized mode. No heading hold in v1; the
  // rudder term simply opposes the measured yaw rate to take the wallow out
  // of the airframe.
  float yaw_cmd = -kd_yaw * rates.yaw_dps;

#if ENABLE_TURN_COORDINATION
  // Coordinated-turn feedforward in the self-leveling modes: auto-rudder plus
  // bank-compensated up-elevator. Rate and manual get no coordination.
  if (mode == PLANE_MODE_ANGLE || mode == PLANE_MODE_HORIZON) {
    const TurnFeedforward ff = turnCoordination(euler.roll_deg);
    yaw_cmd   += ff.rudder;
    pitch_cmd += ff.pitch;
  }
#endif

  // Stab authority blend. Authority 1.0 = full PID output (the historical
  // behaviour). Authority 0.0 = pure pilot passthrough (the surfaces follow
  // the sticks with no stabilization). The pilot sets this on the fly via
  // a TX knob assigned to GAIN_CHANNEL; the LPF state s_authority_filt is
  // already updated at the top of this function.
  const float authority  = s_authority_filt;
  const float roll_stab  = STAB_OUTPUT_SCALE * roll_cmd;
  const float pitch_stab = STAB_OUTPUT_SCALE * pitch_cmd;
  const float yaw_stab   = STAB_OUTPUT_SCALE * yaw_cmd;
  s_output.roll  = clampUnit((1.0f - authority) * desired.roll_passthru  + authority * roll_stab);
  s_output.pitch = clampUnit((1.0f - authority) * desired.pitch_passthru + authority * pitch_stab);
  s_output.yaw   = clampUnit((1.0f - authority) * desired.yaw_passthru   + authority * yaw_stab);
  s_output.stab_authority = authority;

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

void setImuLossOverride(bool active) {
  s_imu_loss_override = active;
}

}  // namespace cp::control::plane_stab

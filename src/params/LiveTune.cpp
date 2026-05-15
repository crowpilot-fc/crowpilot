// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "params/LiveTune.h"

#include <Arduino.h>

#include "Config.h"
#include "params/Params.h"
#include "radio/Receiver.h"

namespace cp::params::live {

namespace {

float s_kp_mult = 1.0f;
float s_kd_mult = 1.0f;

#if ENABLE_LIVE_TUNE

// Channel position treated as multiplier 1.0. Recentred after a save so the
// baked-in gain is not scaled a second time by a still-deflected knob.
uint16_t s_kp_neutral_us = 1500;
uint16_t s_kd_neutral_us = 1500;

// Save-gesture state. The gesture is yaw held hard left while disarmed; the
// latch makes it fire exactly once until the stick is released.
uint32_t s_save_hold_ticks = 0;
bool     s_save_latched    = false;

constexpr uint32_t kSaveHoldTicks   = 2UL * LOOP_HZ;  // two seconds
constexpr uint16_t kYawSaveThreshUs = 1150;           // hard-left yaw
constexpr float    kChannelSpanUs   = 500.0f;         // 1500 +/- 500

// Parameters scaled by the Kp and Kd multipliers. Kept explicit so the save
// gesture knows exactly which registry entries to bake.
constexpr ParamId kKpParams[] = {
  KP_ROLL_HOVER, KP_PITCH_HOVER, KP_YAW_HOVER,
  KP_ROLL_FF,    KP_PITCH_FF,    KP_YAW_FF,
};
constexpr ParamId kKdParams[] = {
  KD_ROLL_HOVER, KD_PITCH_HOVER, KD_YAW_HOVER,
  KD_ROLL_FF,    KD_PITCH_FF,    KD_YAW_FF,
};

float multiplierFor(uint16_t channel_us, uint16_t neutral_us) {
  const float offset = (static_cast<float>(channel_us) -
                        static_cast<float>(neutral_us)) / kChannelSpanUs;
  float m = 1.0f + LIVE_TUNE_RANGE * offset;
  const float lo = 1.0f - LIVE_TUNE_RANGE;
  const float hi = 1.0f + LIVE_TUNE_RANGE;
  if (m < lo) m = lo;
  if (m > hi) m = hi;
  return m;
}

void commitAndSave() {
  for (ParamId id : kKpParams) {
    cp::params::set(id, cp::params::get(id) * s_kp_mult);
  }
  for (ParamId id : kKdParams) {
    cp::params::set(id, cp::params::get(id) * s_kd_mult);
  }
  const bool ok = cp::params::save();
  if (Serial) {
    Serial.print("LiveTune: gains baked and ");
    Serial.println(ok ? "saved to flash." : "save FAILED (storage off).");
  }
}

#endif  // ENABLE_LIVE_TUNE

}  // namespace

void init() {
  s_kp_mult = 1.0f;
  s_kd_mult = 1.0f;
#if ENABLE_LIVE_TUNE
  s_kp_neutral_us   = 1500;
  s_kd_neutral_us   = 1500;
  s_save_hold_ticks = 0;
  s_save_latched    = false;
#endif
}

void update(const uint16_t* channels, bool armed) {
#if ENABLE_LIVE_TUNE
  static_assert(LIVE_TUNE_CH_KP >= 1 && LIVE_TUNE_CH_KP <= cp::radio::MAX_CHANNELS,
                "LIVE_TUNE_CH_KP must be a valid 1-based channel index.");
  static_assert(LIVE_TUNE_CH_KD >= 1 && LIVE_TUNE_CH_KD <= cp::radio::MAX_CHANNELS,
                "LIVE_TUNE_CH_KD must be a valid 1-based channel index.");

  const uint16_t kp_ch = channels[LIVE_TUNE_CH_KP - 1];
  const uint16_t kd_ch = channels[LIVE_TUNE_CH_KD - 1];

  s_kp_mult = multiplierFor(kp_ch, s_kp_neutral_us);
  s_kd_mult = multiplierFor(kd_ch, s_kd_neutral_us);

  // Save gesture: yaw hard left, disarmed, held for kSaveHoldTicks.
  const uint16_t yaw_us = channels[3];  // ch4, 0-based
  if (!armed && yaw_us > 0 && yaw_us < kYawSaveThreshUs) {
    if (s_save_hold_ticks < kSaveHoldTicks) {
      ++s_save_hold_ticks;
    }
    if (s_save_hold_ticks >= kSaveHoldTicks && !s_save_latched) {
      commitAndSave();
      // Recenter so the still-deflected knobs no longer scale the baked
      // gains, and the multipliers return to 1.0.
      s_kp_neutral_us = kp_ch;
      s_kd_neutral_us = kd_ch;
      s_kp_mult = 1.0f;
      s_kd_mult = 1.0f;
      s_save_latched = true;
    }
  } else {
    s_save_hold_ticks = 0;
    s_save_latched    = false;
  }
#else
  (void)channels;
  (void)armed;
  s_kp_mult = 1.0f;
  s_kd_mult = 1.0f;
#endif
}

float kpMultiplier() {
  return s_kp_mult;
}

float kdMultiplier() {
  return s_kd_mult;
}

}  // namespace cp::params::live

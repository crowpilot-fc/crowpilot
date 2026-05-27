// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/estimation/DynamicNotch.h"

#include "src/Config.h"

#if ENABLE_DYNAMIC_NOTCH

#include "src/airframes/Airframe.h"
#include "src/estimation/Attitude.h"
#include "src/hal/Hal.h"

namespace cp::estimation::dynnotch {

namespace {

// Slew the current value toward target by at most max_step.
float slewToward(float current, float target, float max_step) {
  const float delta = target - current;
  if (delta > max_step) {
    return current + max_step;
  }
  if (delta < -max_step) {
    return current - max_step;
  }
  return target;
}

uint32_t s_tick    = 0;
float    s_center  = GYRO_NOTCH_CENTER_HZ;  // currently applied notch center
bool     s_engaged = false;                 // tracking a motor frequency now

}  // anonymous namespace

float computeTargetHz(const uint32_t* erpm, uint8_t count) {
  const float per_pole = 60.0f * static_cast<float>(MOTOR_POLE_PAIRS);
  float   sum_hz  = 0.0f;
  uint8_t running = 0;
  for (uint8_t i = 0; i < count; ++i) {
    if (erpm[i] > 0) {
      sum_hz += static_cast<float>(erpm[i]) / per_pole;
      ++running;
    }
  }
  if (running == 0) {
    return 0.0f;
  }
  float hz = sum_hz / static_cast<float>(running);
  if (hz < DYN_NOTCH_MIN_HZ) {
    hz = DYN_NOTCH_MIN_HZ;
  } else if (hz > DYN_NOTCH_MAX_HZ) {
    hz = DYN_NOTCH_MAX_HZ;
  }
  return hz;
}

void init() {
  s_tick    = 0;
  s_center  = GYRO_NOTCH_CENTER_HZ;
  s_engaged = false;
}

void update() {
  if (++s_tick < DYN_NOTCH_UPDATE_DIV) {
    return;
  }
  s_tick = 0;

  uint32_t erpm[cp::airframes::N_MOTORS];
  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    erpm[i] = cp::hal::out_get_erpm(i);
  }

  const float target = computeTargetHz(erpm, cp::airframes::N_MOTORS);

  if (target <= 0.0f) {
    // No telemetry or motors stopped: fall back to the fixed notch.
    if (s_engaged) {
      s_center  = GYRO_NOTCH_CENTER_HZ;
      s_engaged = false;
      cp::estimation::attitude::setGyroNotchCenter(s_center);
    }
    return;
  }

  if (!s_engaged) {
    // First running sample: snap to it so we do not slew up from the fixed
    // center (often 0) through the sub-band on the way.
    s_center  = target;
    s_engaged = true;
  } else {
    const float dt_s     = static_cast<float>(DYN_NOTCH_UPDATE_DIV) /
                           static_cast<float>(LOOP_HZ);
    const float max_step = DYN_NOTCH_MAX_SLEW_HZ_PER_S * dt_s;
    s_center             = slewToward(s_center, target, max_step);
  }
  cp::estimation::attitude::setGyroNotchCenter(s_center);
}

}  // namespace cp::estimation::dynnotch

#endif  // ENABLE_DYNAMIC_NOTCH

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include "src/Config.h"

// Shared aileron-pair mixing for the conventional plane airframes (single,
// twin, V-tail). It folds three wing options into the two aileron commands:
//
//   Differential ailerons. The down-going aileron throws less than the
//     up-going one (DIFF_AILERON_RATIO, 1.0 = symmetric), to cut adverse yaw.
//   Flaperon. A flap command droops both ailerons together as flaps.
//   Airbrake. A brake command reflexes both ailerons up together for a
//     glide-path brake. On a two-aileron wing this is the achievable analog of
//     four-surface crow (which also needs dedicated flap servos); the up-going
//     spoiler effect is the braking part. See docs/reference/config-options.md.
//
// flap_cmd and brake_cmd are 0..1. roll is the normalized -1..1 demand. The
// outputs are servo commands clamped to [0, 1] with 0.5 the geometric center.
// Pure and header-only, so it is host-testable.

namespace cp::airframes::wing {

inline float clamp01_(float x) {
  if (x > 1.0f) return 1.0f;
  if (x < 0.0f) return 0.0f;
  return x;
}

inline void aileronPair(float roll, float flap_cmd, float brake_cmd,
                        float& left, float& right) {
  float l = AILERON_TRAVEL * roll;
  float r = -AILERON_TRAVEL * roll;
  // Differential: reduce whichever aileron is deflecting down (command above
  // center), which is the adverse-yaw-producing side.
  if (l > 0.0f) l *= DIFF_AILERON_RATIO;
  if (r > 0.0f) r *= DIFF_AILERON_RATIO;
  // Flaperon droops both down, the brake reflexes both up. Common to both.
  const float common = flap_cmd * FLAPERON_TRAVEL - brake_cmd * CROW_TRAVEL;
  left  = clamp01_(0.5f + l + common);
  right = clamp01_(0.5f + r + common);
}

}  // namespace cp::airframes::wing

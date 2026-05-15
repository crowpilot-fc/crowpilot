// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// Runtime parameter registry. Every tunable registered in params.def is
// exposed as a runtime-mutable parameter with a default, clamp bounds, and
// an optional flash-persist flag. The registry is the in-RAM source of
// truth read by the control modules; Config.h supplies the defaults.

namespace cp::params {

// One enumerator per CP_PARAM row in params.def, plus PARAM_COUNT as the
// trailing size marker.
enum ParamId : uint8_t {
#define CP_PARAM(id, key, def, lo, hi, persist) id,
#include "params/params.def"
#undef CP_PARAM
  PARAM_COUNT
};

struct Param {
  const char* key;            // stable string key, see params.def
  float       value;          // current runtime value
  float       default_value;  // compile-time default from Config.h
  float       min_value;      // inclusive clamp lower bound
  float       max_value;      // inclusive clamp upper bound
  bool        persist;        // written to flash by save()
};

// Populate the registry from the Config.h defaults, then, if
// ENABLE_PARAM_PERSIST is set, mount the flash store and overlay any
// persisted values. Never halts: a storage failure leaves the registry at
// defaults and is reported over serial.
void init();

// Current value of a parameter. Cheap (array index); safe in the hot loop.
float get(ParamId id);

// Set a parameter. The value is clamped to [min_value, max_value]. Returns
// true if the stored value changed. A changed persistent parameter marks
// the registry dirty.
bool set(ParamId id, float value);

// Registry entry for a parameter, including bounds and the key string.
const Param& info(ParamId id);

// Look a parameter up by its string key. Returns the ParamId as an int, or
// -1 if no parameter has that key.
int findByKey(const char* key);

// Write every persistent parameter to flash. Returns false if persistence
// is disabled or the flash store is unavailable. Clears the dirty flag on
// success.
bool save();

// Restore persistent parameters from flash, overlaying the current
// registry. Returns false if persistence is disabled, the store is
// unavailable, or no saved file exists.
bool load();

// True when a persistent parameter has changed since the last save() or
// load(). Used by the live-tune save gesture and by debug output.
bool isDirty();

}  // namespace cp::params

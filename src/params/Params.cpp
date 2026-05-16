// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/params/Params.h"

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "src/Config.h"
#include "src/storage/LittleFsStore.h"

namespace cp::params {

namespace {

// Registry. Each row is initialised straight from params.def: the default
// expression resolves to the matching Config.h constexpr, which is a
// constant expression and so valid in a static initialiser.
Param s_params[PARAM_COUNT] = {
#define CP_PARAM(id, key, def, lo, hi, persist) \
  { key, (def), (def), (lo), (hi), (persist) != 0 },
#include "src/params/params.def"
#undef CP_PARAM
};

bool s_dirty = false;

// Persisted parameter file on the flash store. Plain text, one
// `key=value` line per persistent parameter, so it can be inspected and
// hand-edited if ever needed.
constexpr const char* kParamFile  = "/params.cfg";
constexpr size_t      kFileBufLen = 1024;

inline float clampValue(const Param& p, float v) {
  if (v < p.min_value) return p.min_value;
  if (v > p.max_value) return p.max_value;
  return v;
}

}  // namespace

void init() {
  // Registry already holds the Config.h defaults from the static
  // initialiser. Reassert them explicitly so a re-init (e.g. in a test
  // harness) is well defined.
  for (uint8_t i = 0; i < PARAM_COUNT; ++i) {
    s_params[i].value = s_params[i].default_value;
  }
  s_dirty = false;

#if ENABLE_PARAM_PERSIST
  if (cp::storage::begin()) {
    if (load()) {
      if (Serial) {
        Serial.println("Params loaded from flash.");
      }
    } else {
      if (Serial) {
        Serial.println("Params at defaults (no saved file).");
      }
    }
  } else {
    if (Serial) {
      Serial.println("WARN: param flash store unavailable. Defaults only.");
    }
  }
#endif
}

float get(ParamId id) {
  return s_params[id].value;
}

bool set(ParamId id, float value) {
  Param& p = s_params[id];
  const float clamped = clampValue(p, value);
  if (clamped == p.value) {
    return false;
  }
  p.value = clamped;
  if (p.persist) {
    s_dirty = true;
  }
  return true;
}

const Param& info(ParamId id) {
  return s_params[id];
}

int findByKey(const char* key) {
  for (uint8_t i = 0; i < PARAM_COUNT; ++i) {
    if (strcmp(s_params[i].key, key) == 0) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool save() {
#if ENABLE_PARAM_PERSIST
  char buf[kFileBufLen];
  size_t len = 0;
  for (uint8_t i = 0; i < PARAM_COUNT; ++i) {
    const Param& p = s_params[i];
    if (!p.persist) {
      continue;
    }
    char value_str[20];
    dtostrf(p.value, 0, 6, value_str);
    const int written = snprintf(buf + len, kFileBufLen - len, "%s=%s\n",
                                 p.key, value_str);
    if (written <= 0 || static_cast<size_t>(written) >= kFileBufLen - len) {
      // Buffer exhausted. Should not happen for the v1 registry size;
      // bail rather than write a truncated file.
      return false;
    }
    len += static_cast<size_t>(written);
  }
  if (!cp::storage::writeText(kParamFile, buf, len)) {
    return false;
  }
  s_dirty = false;
  return true;
#else
  return false;
#endif
}

bool load() {
#if ENABLE_PARAM_PERSIST
  char buf[kFileBufLen];
  const int n = cp::storage::readText(kParamFile, buf, kFileBufLen);
  if (n <= 0) {
    return false;
  }

  // Walk the buffer line by line. The parse is in-place: each line is
  // null-terminated at its newline and the '=' is split into a key and a
  // value substring.
  char* cursor = buf;
  while (*cursor != '\0') {
    char* line = cursor;
    char* nl = strchr(cursor, '\n');
    if (nl != nullptr) {
      *nl = '\0';
      cursor = nl + 1;
    } else {
      cursor = line + strlen(line);
    }

    char* eq = strchr(line, '=');
    if (eq == nullptr) {
      continue;  // blank or malformed line, skip
    }
    *eq = '\0';
    const char* key = line;
    const char* value_str = eq + 1;

    const int id = findByKey(key);
    if (id < 0) {
      continue;  // unknown key, e.g. a parameter removed in a later build
    }
    const float value = static_cast<float>(strtod(value_str, nullptr));
    set(static_cast<ParamId>(id), value);
  }
  // A freshly loaded registry is by definition in sync with flash.
  s_dirty = false;
  return true;
#else
  return false;
#endif
}

bool isDirty() {
  return s_dirty;
}

}  // namespace cp::params

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/cli/Cli.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <Arduino.h>

#include "src/Config.h"
#include "src/actuators/Output.h"
#include "src/params/Params.h"

namespace cp::cli {

#if ENABLE_CONFIG_CLI

namespace {

// Longest meaningful line is "cp set <key> <value>", well under this.
// A line that overruns the buffer is discarded with cp err toolong.
constexpr uint8_t kLineMax = 96;

// Decimal places for every float printed. Five resolves the smallest
// gain step the registry holds without trailing-noise digits.
constexpr int kFloatDigits = 5;

char    s_line[kLineMax] = {};
uint8_t s_len            = 0;
bool    s_overflow       = false;
bool    s_stream         = false;

void reply(const char* msg) {
  Serial.print("cp ");
  Serial.println(msg);
}

void emitParam(int idx) {
  const cp::params::Param& p =
      cp::params::info(static_cast<cp::params::ParamId>(idx));
  Serial.print("cp param ");
  Serial.print(idx);
  Serial.print(' ');
  Serial.print(p.key);
  Serial.print(' ');
  Serial.print(p.value, kFloatDigits);
  Serial.print(' ');
  Serial.print(p.default_value, kFloatDigits);
  Serial.print(' ');
  Serial.print(p.min_value, kFloatDigits);
  Serial.print(' ');
  Serial.print(p.max_value, kFloatDigits);
  Serial.print(' ');
  Serial.println(p.persist ? 1 : 0);
}

void doHandshake() {
  Serial.print("cp fw ");
  Serial.print(CROWPILOT_VERSION);
  Serial.print(" params ");
  Serial.println(static_cast<int>(cp::params::PARAM_COUNT));
}

void doList() {
  const int count = static_cast<int>(cp::params::PARAM_COUNT);
  for (int i = 0; i < count; ++i) {
    emitParam(i);
  }
  Serial.println("cp end");
}

void doSet() {
  const char* key    = strtok(nullptr, " ");
  const char* valstr = strtok(nullptr, " ");
  if (key == nullptr || valstr == nullptr) {
    reply("err noargs");
    return;
  }
  const int id = cp::params::findByKey(key);
  if (id < 0) {
    reply("err nokey");
    return;
  }
  char*       endp = nullptr;
  const float value = strtof(valstr, &endp);
  if (endp == valstr) {
    reply("err badval");
    return;
  }
  const cp::params::ParamId pid = static_cast<cp::params::ParamId>(id);
  cp::params::set(pid, value);
  Serial.print("cp ok set ");
  Serial.print(key);
  Serial.print(' ');
  Serial.println(cp::params::get(pid), kFloatDigits);
}

void doSave() {
  if (cp::actuators::arm_state() == cp::actuators::ArmState::ARMED) {
    reply("err armed");
    return;
  }
  reply(cp::params::save() ? "ok saved" : "err persist");
}

void doLoad() {
  reply(cp::params::load() ? "ok loaded" : "err persist");
}

void doDefaults() {
  const int count = static_cast<int>(cp::params::PARAM_COUNT);
  for (int i = 0; i < count; ++i) {
    const cp::params::ParamId pid = static_cast<cp::params::ParamId>(i);
    cp::params::set(pid, cp::params::info(pid).default_value);
  }
  reply("ok defaults");
}

void doStream() {
  const char* arg = strtok(nullptr, " ");
  if (arg == nullptr) {
    reply("err noargs");
    return;
  }
  if (strcmp(arg, "on") == 0) {
    s_stream = true;
    reply("ok stream on");
  } else if (strcmp(arg, "off") == 0) {
    s_stream = false;
    reply("ok stream off");
  } else {
    reply("err badval");
  }
}

void handleLine(char* line) {
  const char* prefix = strtok(line, " ");
  if (prefix == nullptr || strcmp(prefix, "cp") != 0) {
    reply("err badcmd");
    return;
  }
  const char* verb = strtok(nullptr, " ");
  if (verb == nullptr) {
    doHandshake();
  } else if (strcmp(verb, "list") == 0) {
    doList();
  } else if (strcmp(verb, "set") == 0) {
    doSet();
  } else if (strcmp(verb, "save") == 0) {
    doSave();
  } else if (strcmp(verb, "load") == 0) {
    doLoad();
  } else if (strcmp(verb, "defaults") == 0) {
    doDefaults();
  } else if (strcmp(verb, "stream") == 0) {
    doStream();
  } else {
    reply("err badcmd");
  }
}

}  // anonymous namespace

#endif  // ENABLE_CONFIG_CLI

void init() {
#if ENABLE_CONFIG_CLI
  s_len      = 0;
  s_overflow = false;
  s_stream   = false;
#endif
}

void poll() {
#if ENABLE_CONFIG_CLI
  if (!Serial) {
    return;
  }
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c < 0) {
      break;
    }
    if (c == '\n' || c == '\r') {
      if (s_overflow) {
        reply("err toolong");
      } else if (s_len > 0) {
        s_line[s_len] = '\0';
        handleLine(s_line);
      }
      s_len      = 0;
      s_overflow = false;
    } else if (s_len < kLineMax - 1) {
      s_line[s_len++] = static_cast<char>(c);
    } else {
      s_overflow = true;
    }
  }
#endif
}

bool streaming() {
#if ENABLE_CONFIG_CLI
  return s_stream;
#else
  return false;
#endif
}

}  // namespace cp::cli

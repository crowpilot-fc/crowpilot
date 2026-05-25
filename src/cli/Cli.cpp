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

#if ENABLE_ONBOARD_WIFI
#include "src/comms/OnboardWifi.h"
#endif

namespace cp::cli {

#if ENABLE_CONFIG_CLI

namespace {

// The configurator runs on the USB CDC serial and, on native builds with
// ENABLE_COMPANION_CLI, on the companion UART. SerialUSB and SerialUART
// share the Arduino Stream base, so a channel holds a Stream pointer. The
// SITL host shim's Serial is not a Stream, so there the alias is the shim
// class and only the USB channel exists.
#if BUILD_TARGET == BUILD_TARGET_NATIVE
using CliStream = Stream;
#else
using CliStream = SerialClass;
#endif

constexpr unsigned long kCompanionBaud = 115200;

// Longest meaningful line is "cp set <key> <value>", well under this.
// A line that overruns the buffer is discarded with cp err toolong.
constexpr uint8_t kLineMax = 96;

// Decimal places for every float printed.
constexpr int kFloatDigits = 5;

// One command channel: an input/output stream plus its own line-assembly
// state, so the two channels do not interleave each other's bytes.
struct Channel {
  CliStream* io;
  char       line[kLineMax];
  uint8_t    len;
  bool       overflow;
};

// USB, the companion UART, and (on a WiFi board) the onboard WiFi bridge.
Channel s_chan[3];
uint8_t s_chan_count = 0;

// The channel stream that last issued "cp stream on", or nullptr. The
// telemetry line goes here. One consumer at a time: a "stream on" from the
// other channel moves the sink.
CliStream* s_stream_out = nullptr;

// Output stream for the command currently being dispatched.
CliStream* s_io = nullptr;

void reply(const char* msg) {
  s_io->print("cp ");
  s_io->println(msg);
}

void emitParam(int idx) {
  const cp::params::Param& p =
      cp::params::info(static_cast<cp::params::ParamId>(idx));
  s_io->print("cp param ");
  s_io->print(idx);
  s_io->print(' ');
  s_io->print(p.key);
  s_io->print(' ');
  s_io->print(p.value, kFloatDigits);
  s_io->print(' ');
  s_io->print(p.default_value, kFloatDigits);
  s_io->print(' ');
  s_io->print(p.min_value, kFloatDigits);
  s_io->print(' ');
  s_io->print(p.max_value, kFloatDigits);
  s_io->print(' ');
  s_io->println(p.persist ? 1 : 0);
}

void doHandshake() {
  s_io->print("cp fw ");
  s_io->print(CROWPILOT_VERSION);
  s_io->print(" params ");
  s_io->println(static_cast<int>(cp::params::PARAM_COUNT));
}

void doList() {
  const int count = static_cast<int>(cp::params::PARAM_COUNT);
  for (int i = 0; i < count; ++i) {
    emitParam(i);
  }
  s_io->println("cp end");
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
  s_io->print("cp ok set ");
  s_io->print(key);
  s_io->print(' ');
  s_io->println(cp::params::get(pid), kFloatDigits);
}

void doSave() {
  if (cp::actuators::arm_state() == cp::actuators::ArmState::ARMED) {
    reply("err armed");
    return;
  }
  reply(cp::params::save() ? "ok saved" : "err persist");
}

void doLoad() {
  // Reloading flash params swaps every control gain at once. Rejected
  // while armed, like cp save and cp boot.
  if (cp::actuators::arm_state() == cp::actuators::ArmState::ARMED) {
    reply("err armed");
    return;
  }
  reply(cp::params::load() ? "ok loaded" : "err persist");
}

void doDefaults() {
  // Resetting to defaults swaps every control gain at once. Rejected
  // while armed.
  if (cp::actuators::arm_state() == cp::actuators::ArmState::ARMED) {
    reply("err armed");
    return;
  }
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
    s_stream_out = s_io;
    reply("ok stream on");
  } else if (strcmp(arg, "off") == 0) {
    if (s_stream_out == s_io) {
      s_stream_out = nullptr;
    }
    reply("ok stream off");
  } else {
    reply("err badval");
  }
}

void doBoot() {
  // Reboot into the RP2350 BOOTSEL bootloader so the configurator can
  // flash over USB. Rejected while armed: a reboot mid-flight drops the
  // aircraft. Rejected on the companion UART: a wireless reboot-to-
  // bootloader is not a v1.1 capability. flush() lets the reply land
  // before the reset.
  if (s_io != s_chan[0].io) {
    reply("err usbonly");
    return;
  }
  if (cp::actuators::arm_state() == cp::actuators::ArmState::ARMED) {
    reply("err armed");
    return;
  }
  reply("ok boot");
  s_io->flush();
  rp2040.rebootToBootloader();
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
  } else if (strcmp(verb, "boot") == 0) {
    doBoot();
  } else {
    reply("err badcmd");
  }
}

// Drain one channel's pending bytes, dispatching each completed line.
void pollChannel(Channel& ch) {
  while (ch.io->available() > 0) {
    const int c = ch.io->read();
    if (c < 0) {
      break;
    }
    if (c == '\n' || c == '\r') {
      if (ch.overflow) {
        s_io = ch.io;
        reply("err toolong");
      } else if (ch.len > 0) {
        ch.line[ch.len] = '\0';
        s_io            = ch.io;
        handleLine(ch.line);
      }
      ch.len      = 0;
      ch.overflow = false;
    } else if (ch.len < kLineMax - 1) {
      ch.line[ch.len++] = static_cast<char>(c);
    } else {
      ch.overflow = true;
    }
  }
}

}  // anonymous namespace

#endif  // ENABLE_CONFIG_CLI

void init() {
#if ENABLE_CONFIG_CLI
  s_chan[0].io       = &Serial;
  s_chan[0].len      = 0;
  s_chan[0].overflow = false;
  s_chan_count       = 1;

#if BUILD_TARGET == BUILD_TARGET_NATIVE && ENABLE_COMPANION_CLI
  // The companion UART is UART1 (Serial2 on the arduino-pico core), routed
  // to the board's companion pins. An ESP WiFi module bridges it to a
  // phone. Harmless when nothing is attached: an idle UART, no bytes.
  Serial2.setTX(PIN_COMPANION_TX);
  Serial2.setRX(PIN_COMPANION_RX);
  Serial2.begin(kCompanionBaud);
  s_chan[1].io       = &Serial2;
  s_chan[1].len      = 0;
  s_chan[1].overflow = false;
  s_chan_count       = 2;
#endif

#if ENABLE_ONBOARD_WIFI
  // Onboard WiFi (boards with an integrated radio). The bridge stream is
  // serviced on core1; the CLI reads and writes it like any other channel.
  s_chan[s_chan_count].io       = cp::comms::onboard_wifi::cli_stream();
  s_chan[s_chan_count].len      = 0;
  s_chan[s_chan_count].overflow = false;
  ++s_chan_count;
#endif

  s_stream_out = nullptr;
#endif
}

void poll() {
#if ENABLE_CONFIG_CLI
  for (uint8_t i = 0; i < s_chan_count; ++i) {
    if (s_chan[i].io != nullptr) {
      pollChannel(s_chan[i]);
    }
  }
#endif
}

bool streaming() {
#if ENABLE_CONFIG_CLI
  return s_stream_out != nullptr;
#else
  return false;
#endif
}

void emit_telemetry(const char* line) {
#if ENABLE_CONFIG_CLI
  if (s_stream_out != nullptr && line != nullptr) {
    s_stream_out->print(line);
    s_stream_out->print('\n');
  }
#else
  (void)line;
#endif
}

}  // namespace cp::cli

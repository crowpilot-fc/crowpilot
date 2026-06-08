// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/cli/Cli.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <Arduino.h>

#include "src/Config.h"
#include "src/actuators/Output.h"
#include "src/hal/Hal.h"
#include "src/params/Params.h"

#if BUILD_TARGET == BUILD_TARGET_NATIVE
#include <Wire.h>
#include <initializer_list>

#include "src/hal/I2c.h"
#endif

#if ENABLE_ONBOARD_WIFI
#include "src/comms/OnboardWifi.h"
#endif

#if BOARD_HAS_ESP_FLASH && BUILD_TARGET == BUILD_TARGET_NATIVE
#include "src/comms/EspFlash.h"
  #define CP_HAS_ESP_FLASH 1
#else
  #define CP_HAS_ESP_FLASH 0
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

void doEsp() {
#if CP_HAS_ESP_FLASH
  // ESP companion passthrough. Two sub-verbs:
  //   cp esp flash  - drop the ESP into its UART ROM bootloader and bridge
  //                   USB CDC <-> companion UART so host esptool can program
  //                   the chip. Returns to the prompt after a 60 s lull.
  //   cp esp reset  - pulse EN to restart the ESP into its application.
  // USB-only (the bridge takes over USB CDC) and refused while armed (the
  // ESP companion is not safety-critical in v1, but holding USB hostage and
  // freezing the link mid-flight is a poor failure mode).
  const char* sub = strtok(nullptr, " ");
  if (sub == nullptr) {
    reply("err badcmd");
    return;
  }
  if (s_io != s_chan[0].io) {
    reply("err usbonly");
    return;
  }
  if (cp::actuators::arm_state() == cp::actuators::ArmState::ARMED) {
    reply("err armed");
    return;
  }
  if (strcmp(sub, "flash") == 0) {
    reply("ok esp flash");
    s_io->flush();
    cp::comms::esp_flash::enter_bootloader();
    cp::comms::esp_flash::run_bridge(60000);
    cp::comms::esp_flash::reset_into_app();
    reply("ok esp done");
  } else if (strcmp(sub, "reset") == 0) {
    cp::comms::esp_flash::reset_into_app();
    reply("ok esp reset");
  } else {
    reply("err badcmd");
  }
#else
  reply("err unsupported");
#endif  // CP_HAS_ESP_FLASH
}

// Bench diagnostic. Probes I2C addresses 0x76 and 0x77 (the two common
// barometer addresses) and dumps chip-ID candidates plus the BMP280
// calibration block and the data registers. Used when the configured
// baro driver does not recognise the chip on the bus, or when a
// breakout is suspected of being a counterfeit. Output is informational
// only, formatting belongs in the CLI layer rather than in the HAL.
void doScan() {
#if BUILD_TARGET == BUILD_TARGET_NATIVE
  cp::hal::i2c::ensureInit();
  reply("scan begin");
  for (uint8_t addr : {0x76, 0x77}) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) {
      s_io->print("scan 0x");
      s_io->print(addr, HEX);
      s_io->println(" no ACK");
      continue;
    }
    for (uint8_t reg : {0x00, 0xD0, 0x0D}) {
      Wire.beginTransmission(addr);
      Wire.write(reg);
      if (Wire.endTransmission(false) != 0) continue;
      if (Wire.requestFrom(addr, (uint8_t)1, (uint8_t)true) != 1) continue;
      uint8_t v = (uint8_t)Wire.read();
      s_io->print("scan 0x");
      s_io->print(addr, HEX);
      s_io->print(" reg 0x");
      s_io->print(reg, HEX);
      s_io->print(" = 0x");
      s_io->println(v, HEX);
    }
    // BMP280 calibration block at 0x88..0x9F (24 bytes). Useful for
    // detecting counterfeit chips that ship with bad NVM.
    Wire.beginTransmission(addr);
    Wire.write(0x88);
    if (Wire.endTransmission(false) == 0 &&
        Wire.requestFrom(addr, (uint8_t)24, (uint8_t)true) == 24) {
      s_io->print("scan 0x");
      s_io->print(addr, HEX);
      s_io->print(" calib 0x88..0x9F:");
      for (int i = 0; i < 24; ++i) {
        uint8_t b = (uint8_t)Wire.read();
        s_io->print(' ');
        if (b < 0x10) s_io->print('0');
        s_io->print(b, HEX);
      }
      s_io->println();
    }
    // BMP280 data registers 0xF7..0xFC (pressure + temperature raw).
    Wire.beginTransmission(addr);
    Wire.write(0xF7);
    if (Wire.endTransmission(false) == 0 &&
        Wire.requestFrom(addr, (uint8_t)6, (uint8_t)true) == 6) {
      s_io->print("scan 0x");
      s_io->print(addr, HEX);
      s_io->print(" data 0xF7..0xFC:");
      for (int i = 0; i < 6; ++i) {
        uint8_t b = (uint8_t)Wire.read();
        s_io->print(' ');
        if (b < 0x10) s_io->print('0');
        s_io->print(b, HEX);
      }
      s_io->println();
    }
  }
  reply("ok scan");
#else
  reply("err unsupported");
#endif
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
  } else if (strcmp(verb, "esp") == 0) {
    doEsp();
  } else if (strcmp(verb, "scan") == 0) {
    doScan();
  } else {
    reply("err badcmd");
  }
}

// Drain one channel's pending bytes, dispatching each completed line. Caps
// the bytes read per tick so a flood from the companion link (e.g. the ESP
// pumping telemetry frames at line rate) cannot dominate the 1 ms loop
// budget. The cap is well above 115200 baud's per-tick byte rate (115.2 kbps
// = ~11 bytes per ms), so on a healthy link every byte still gets serviced
// within a few ticks of arrival; only a pathological burst is throttled.
// Inbound commands are short text lines, so capping at 96 bytes never
// stalls a real command in flight.
void pollChannel(Channel& ch) {
  constexpr uint8_t kMaxBytesPerTick = 96;
  uint8_t budget = kMaxBytesPerTick;
  while (budget-- > 0 && ch.io->available() > 0) {
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
  // The companion UART bridges to an ESP WiFi module that exposes the CLI
  // to a phone. Boards whose PIN_COMPANION_TX/RX are valid UART1 alt-function
  // pins use the hardware UART (Serial2 on the arduino-pico core). Boards
  // that route the companion to non-UART pins (e.g. the Waveshare Tiny,
  // whose top-edge castellations leave no free hardware UART pair) define
  // BOARD_COMPANION_USES_SERIALPIO=1 in their board profile; on those
  // boards the dispatcher uses SerialPIO, the rp2040 Arduino core's
  // PIO-based UART that runs on any GPIO. Harmless when nothing is
  // attached: an idle UART, no bytes.
  #if BOARD_COMPANION_USES_SERIALPIO
  static SerialPIO s_companion_pio(PIN_COMPANION_TX, PIN_COMPANION_RX);
  s_companion_pio.begin(kCompanionBaud);
  s_chan[1].io       = &s_companion_pio;
  #else
  Serial2.setTX(PIN_COMPANION_TX);
  Serial2.setRX(PIN_COMPANION_RX);
  Serial2.begin(kCompanionBaud);
  s_chan[1].io       = &Serial2;
  #endif
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

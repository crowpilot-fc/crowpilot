// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/hal/Hal.h"

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_NATIVE

#include <Arduino.h>

#if RX_PROTOCOL == RX_SBUS

#include <hardware/clocks.h>
#include <hardware/pio.h>

#include "src/libs/Sbus.h"
#include "sbus_rx.pio.h"

namespace cp::hal {

namespace {

// pio0 expands to a reinterpret_cast, so this is const, not constexpr.
const PIO      s_pio = pio0;
constexpr uint s_sm  = 0;

uint     s_offset = 0;

#if RX_SBUS_INVERTED
constexpr uint16_t kPolarityMask = 0x01FF;
#else
constexpr uint16_t kPolarityMask = 0x0000;
#endif

// Local copy of the current receiver state. Mutated each call to rx_poll
// as bytes flow in from the PIO FIFO and frames complete in the SBUS
// parser. Copied out to the caller's buffer at the end of rx_poll.
RxState s_state = {};

}  // anonymous namespace

bool rx_init() {
  s_state = {};
  for (uint8_t i = 0; i < 16; ++i) {
    s_state.channel_us[i] = 1500;
  }

  if (!pio_can_add_program(s_pio, &sbus_rx_inverted_program)) {
    return false;
  }
  s_offset = pio_add_program(s_pio, &sbus_rx_inverted_program);

  pio_sm_config c = sbus_rx_inverted_program_get_default_config(s_offset);
  sm_config_set_in_pins(&c, PIN_SBUS_RX);
  // push_threshold is documented in src/radio/Receiver.cpp (previous
  // location) and applies only when autopush is enabled. Manual `push
  // noblock` in sbus_rx.pio does the actual push.
  sm_config_set_in_shift(&c, /*shift_right=*/true, /*autopush=*/false, /*push_threshold=*/9);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

  const float clkdiv = static_cast<float>(clock_get_hz(clk_sys)) / (100000.0f * 8.0f);
  sm_config_set_clkdiv(&c, clkdiv);

  pio_gpio_init(s_pio, PIN_SBUS_RX);
  pio_sm_set_consecutive_pindirs(s_pio, s_sm, PIN_SBUS_RX, 1, /*is_out=*/false);
  pio_sm_init(s_pio, s_sm, s_offset, &c);
  pio_sm_set_enabled(s_pio, s_sm, true);

  cp::libs::sbus::reset();
  return true;
}

void rx_poll(RxState& out) {
  while (!pio_sm_is_rx_fifo_empty(s_pio, s_sm)) {
    const uint32_t isr = pio_sm_get(s_pio, s_sm);

    const uint16_t raw_bits = static_cast<uint16_t>((isr >> 23) & 0x01FFU);
    const uint16_t bits     = raw_bits ^ kPolarityMask;
    const uint8_t  data     = static_cast<uint8_t>(bits & 0xFFU);
    const uint8_t  parity   = static_cast<uint8_t>((bits >> 8) & 0x01U);

    if (parity != static_cast<uint8_t>(__builtin_parity(data))) {
      continue;
    }

    cp::libs::sbus::DecodedFrame frame;
    if (!cp::libs::sbus::feed(data, frame)) {
      s_state.lost_frames_count = cp::libs::sbus::lostFrameCount();
      continue;
    }

    for (uint8_t i = 0; i < cp::libs::sbus::kNumChannels; ++i) {
      s_state.channel_us[i] = cp::libs::sbus::rawToMicroseconds(frame.channel[i]);
    }
    s_state.ch17              = frame.ch17;
    s_state.ch18              = frame.ch18;
    s_state.frame_lost_flag   = frame.frame_lost;
    s_state.failsafe_active   = frame.failsafe;
    s_state.channels_valid    = true;
    s_state.last_frame_us     = micros();
    s_state.lost_frames_count = cp::libs::sbus::lostFrameCount();
  }

  out = s_state;
}

}  // namespace cp::hal

#elif RX_PROTOCOL == RX_PPM
  #error "RX_PPM is scaffolded in v1.0. Implementation lands in a later phase."
#elif RX_PROTOCOL == RX_PWM

// Per-channel PWM RC input. The receiver feeds N independent signal wires
// (one per channel), each emitting a 1000-2000 us pulse roughly every
// 20 ms. We measure pulse widths with a CHANGE interrupt per pin: rising
// edge timestamps the start, falling edge subtracts to get the width.
//
// PIN_PWM_RX is a board-profile constexpr array. The number of channels is
// fixed at compile time by its size. Up to 8 are supported; the table of
// ISR thunks below is sized to match. Eight covers a conventional plane
// running AETR plus a head-tracker pan/tilt pair plus mode and arm
// switches, which is the full budget of a common 8-channel PWM receiver.
//
// Caveat documented in the F-14 build note and on the public wiring page:
// PWM receivers do NOT carry a per-frame failsafe bit, unlike SBUS. The
// firmware detects link loss via a per-channel last-pulse timestamp. If
// any channel has not seen a fresh edge within FS_LINK_TIMEOUT_US, the
// receiver state goes channels_valid=false and the failsafe layer takes
// over. rx_poll below implements exactly that; do not weaken it to an
// any-channel-alive test, which cannot see a single dead signal wire.
//
// The harder caveat, which no firmware can fix: a PWM receiver that holds
// its last commanded pulse widths (or drives preset ones) after RF loss is
// indistinguishable here from a live link, because there is no protocol bit
// to inspect and the pulses keep arriving on time. The only defence is on
// the receiver: prefer one that stops pulsing on RF loss, and if it cannot,
// program its own failsafe positions to throttle low, surfaces centred, and
// a deliberate arm-channel value. Bench-test this with the prop off before
// every first flight on a new receiver.

namespace cp::hal {

namespace {

constexpr uint8_t kNumPwmChannels = sizeof(PIN_PWM_RX) / sizeof(PIN_PWM_RX[0]);
static_assert(kNumPwmChannels >= 1 && kNumPwmChannels <= 8,
              "PIN_PWM_RX must have 1..8 entries");

// Sanity bounds on the measured pulse width. Anything outside this band
// is ignored as noise (a glitch on the pin, or a partial pulse from a
// dropped frame). The legal RC range is 1000-2000 us; the band here is
// wider so a slightly mistuned transmitter or a long failsafe pulse from
// some receivers still registers.
constexpr uint32_t kMinPulseUs = 800;
constexpr uint32_t kMaxPulseUs = 2200;

// ISR-touched state. volatile because the main loop reads them outside
// the noInterrupts() critical section. uint16_t pulse_width is the most
// recent valid pulse; uint32_t last_pulse_us is when it was captured.
// Pulse widths get centred to 1500 us in rx_init() rather than via the
// static initializer so the array size can change without dropping the
// centre value on the new elements.
volatile uint32_t s_pulse_start_us[kNumPwmChannels] = {};
volatile uint16_t s_pulse_width_us[kNumPwmChannels] = {};
volatile uint32_t s_last_pulse_us[kNumPwmChannels]  = {};
volatile bool     s_channel_seen[kNumPwmChannels]   = {};

RxState s_state = {};

inline void handlePwmEdge(uint8_t ch) {
  const uint32_t now = micros();
  const int level    = digitalRead(PIN_PWM_RX[ch]);
  if (level == HIGH) {
    s_pulse_start_us[ch] = now;
  } else {
    const uint32_t start = s_pulse_start_us[ch];
    if (start == 0) {
      return;  // no matching rising edge captured yet
    }
    const uint32_t width = now - start;
    if (width >= kMinPulseUs && width <= kMaxPulseUs) {
      s_pulse_width_us[ch] = static_cast<uint16_t>(width);
      s_last_pulse_us[ch]  = now;
      s_channel_seen[ch]   = true;
    }
  }
}

// One ISR thunk per channel. attachInterrupt takes a plain function
// pointer, so we cannot template the channel index in. Eight thunks cover
// every supported channel count; only the first kNumPwmChannels are wired
// up by rx_init.
void isr0() { handlePwmEdge(0); }
void isr1() { handlePwmEdge(1); }
void isr2() { handlePwmEdge(2); }
void isr3() { handlePwmEdge(3); }
void isr4() { handlePwmEdge(4); }
void isr5() { handlePwmEdge(5); }
void isr6() { handlePwmEdge(6); }
void isr7() { handlePwmEdge(7); }

void (*const kIsrs[8])() = { isr0, isr1, isr2, isr3,
                             isr4, isr5, isr6, isr7 };

}  // anonymous namespace

bool rx_init() {
  s_state = {};
  for (uint8_t i = 0; i < 16; ++i) {
    s_state.channel_us[i] = 1500;
  }
  for (uint8_t i = 0; i < kNumPwmChannels; ++i) {
    s_pulse_width_us[i] = 1500;
    pinMode(PIN_PWM_RX[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_PWM_RX[i]), kIsrs[i], CHANGE);
  }
  return true;
}

void rx_poll(RxState& out) {
  const uint32_t now = micros();

  // Per-channel widths and ages copied out under interrupts-off so the main
  // loop sees a consistent snapshot. The block is short, no I/O, no
  // allocations. Ages are computed here rather than stored, because an
  // unsigned (now - then) subtraction is correct across the ~71.6 minute
  // micros() rollover while a bare timestamp comparison is not.
  uint16_t width_us[kNumPwmChannels];
  uint32_t age_us[kNumPwmChannels];
  bool     seen[kNumPwmChannels];

  noInterrupts();
  for (uint8_t i = 0; i < kNumPwmChannels; ++i) {
    width_us[i] = s_pulse_width_us[i];
    seen[i]     = s_channel_seen[i];
    age_us[i]   = now - s_last_pulse_us[i];
  }
  interrupts();

  // EVERY wired channel must have produced a pulse and must still be fresh.
  //
  // The obvious cheaper rule, "valid if any channel is alive", is unsafe on
  // a PWM receiver: one channel that keeps pulsing would mask a throttle,
  // elevator, mode, or arm input whose wire has come off, indefinitely and
  // silently. On RF loss a PWM receiver stops (or freezes) every channel
  // together, so requiring all of them costs nothing in the case this is
  // actually guarding against, and catches the single-wire failure too.
  //
  // Consequence worth knowing: on this rule every channel the board profile
  // lists in PIN_PWM_RX must actually be wired to the receiver. A profile
  // entry left unconnected holds the aircraft in failsafe forever.
  bool     all_fresh = true;
  uint32_t worst_age = 0;
  for (uint8_t i = 0; i < kNumPwmChannels; ++i) {
    s_state.channel_us[i] = width_us[i];
    // A channel that has never pulsed is treated as infinitely stale.
    const uint32_t age = seen[i] ? age_us[i] : 0xFFFFFFFFu;
    if (age > FS_LINK_TIMEOUT_US) {
      all_fresh = false;
    }
    if (age > worst_age) {
      worst_age = age;
    }
  }

  // Role channels above kNumPwmChannels (CHANNEL_TRANSITION on a plane
  // build, for example) are not wired and stay at the boot default of
  // 1500 us. They are deliberately excluded from the freshness rule above;
  // the static_assert in Config.h is what ensures no channel the active
  // build actually needs falls into that gap.

  // Report the LIMITING channel's timestamp, not the freshest, so any
  // downstream age check sees the worst case. Capped just past the timeout
  // so the never-seen case cannot wrap into a fresh-looking value.
  constexpr uint32_t kStaleCapUs = FS_LINK_TIMEOUT_US + 1;
  s_state.channels_valid    = all_fresh;
  s_state.last_frame_us     = now - (worst_age > kStaleCapUs ? kStaleCapUs : worst_age);
  s_state.frame_lost_flag   = false;
  s_state.failsafe_active   = false;  // PWM has no per-frame failsafe flag
  s_state.lost_frames_count = 0;
  s_state.ch17              = false;
  s_state.ch18              = false;

  out = s_state;
}

}  // namespace cp::hal

#elif RX_PROTOCOL == RX_CRSF

#include "src/libs/Crsf.h"

namespace cp::hal {

namespace {

// CRSF is a 420 kbaud, 8N1, non-inverted UART, so it uses a hardware UART
// rather than the inverted-SBUS PIO. UART0 (Serial1 on the arduino-pico
// core) receives on PIN_SBUS_RX, the same pin the receiver signal lands on.
// The transmit pin (GP0) is reserved for CRSF telemetry back to the
// receiver, unused in v1.
constexpr uint32_t kCrsfBaud  = 420000;
constexpr uint8_t  kCrsfTxPin = 0;

RxState s_state = {};

}  // anonymous namespace

bool rx_init() {
  s_state = {};
  for (uint8_t i = 0; i < 16; ++i) {
    s_state.channel_us[i] = 1500;
  }
  Serial1.setRX(PIN_SBUS_RX);
  Serial1.setTX(kCrsfTxPin);
  Serial1.begin(kCrsfBaud);
  cp::libs::crsf::reset();
  return true;
}

void rx_poll(RxState& out) {
  while (Serial1.available() > 0) {
    const int b = Serial1.read();
    if (b < 0) {
      break;
    }
    cp::libs::crsf::DecodedFrame frame;
    if (!cp::libs::crsf::feed(static_cast<uint8_t>(b), frame)) {
      s_state.lost_frames_count = cp::libs::crsf::lostFrameCount();
      continue;
    }
    for (uint8_t i = 0; i < cp::libs::crsf::kNumChannels; ++i) {
      s_state.channel_us[i] = cp::libs::crsf::rawToMicroseconds(frame.channel[i]);
    }
    s_state.ch17            = false;
    s_state.ch18            = false;
    s_state.frame_lost_flag = false;
    // CRSF has no per-frame failsafe bit in the RC frame. Link loss is caught
    // by the firmware's link-timeout failsafe when fresh frames stop arriving.
    s_state.failsafe_active   = false;
    s_state.channels_valid    = true;
    s_state.last_frame_us     = micros();
    s_state.lost_frames_count = cp::libs::crsf::lostFrameCount();
  }
  out = s_state;
}

}  // namespace cp::hal

#else
  #error "Unknown RX_PROTOCOL. Pick RX_SBUS for v1.0."
#endif

#endif  // BUILD_TARGET == BUILD_TARGET_NATIVE

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
  #error "RX_PWM is scaffolded in v1.0. Implementation lands in a later phase."
#elif RX_PROTOCOL == RX_CRSF
  #error "RX_CRSF is scaffolded in v1.0. Implementation lands in a later phase."
#else
  #error "Unknown RX_PROTOCOL. Pick RX_SBUS for v1.0."
#endif

#endif  // BUILD_TARGET == BUILD_TARGET_NATIVE

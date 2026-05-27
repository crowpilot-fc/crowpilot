// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/hal/Hal.h"

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_NATIVE

#include <Arduino.h>
#include <Servo.h>

#include "src/airframes/Airframe.h"

#if MOTOR_PROTOCOL_IS_DSHOT
#include <hardware/clocks.h>
#include <hardware/pio.h>

#include "src/libs/Dshot.h"
#if ENABLE_DSHOT_BIDIR
#include "dshot_bidir.pio.h"
#include "dshot_bidir_rx.pio.h"
#else
#include "dshot.pio.h"
#endif
#endif

// Native actuator output. See docs/developer-guide/algorithms.md.
//
// MOTOR_PROTOCOL selects how the motor pins are driven.
//
//   PWM         the Servo library drives the motor pins at standard 1000
//               to 2000 us on its own 50 Hz schedule, the same path as
//               the servos. out_commit_motors is a no-op.
//   ONESHOT125  out_set_motor_us buffers a 125 to 250 us pulse width and
//               out_commit_motors emits every motor together as one
//               synchronous burst.
//   DSHOT300/600  out_set_motor_us converts the pulse width to a DShot
//               throttle value and buffers the 16-bit frame.
//               out_commit_motors clocks each motor's frame out of its own
//               PIO state machine. One state machine per motor on PIO1
//               (PIO0 carries the SBUS receiver). The line idles low
//               between frames, so before the first commit and whenever
//               disarmed the motors see the DShot motor-stop command.
//
// With ENABLE_DSHOT_BIDIR the output is inverted (idle high) and carries an
// inverted CRC, which makes the ESC reply with its electrical RPM after each
// frame. The transmit side is the dshot_bidir program on PIO1. The receive
// side oversamples the line with the dshot_bidir_rx program on PIO2, one state
// machine per motor, and out_commit_motors reads the reply window and decodes
// it through src/libs/Dshot. The receive timing is bench-tuned: see the
// BENCH-TUNING banner on the bidir capture below.
//
// Servos always use the Servo library, which maintains its own 50 Hz PWM
// schedule, so out_set_servo_us applies the new width immediately.

namespace cp::hal {

namespace {

// Motor index to GPIO pin. The index order matches the airframe motor
// constants (MOTOR_RIGHT is 0, MOTOR_LEFT is 1).
#if AIRFRAME == AIRFRAME_QUAD_X
// A quad has no servos, so the two servo pins carry motors 3 and 4. Order
// matches MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT, MOTOR_REAR_RIGHT,
// MOTOR_REAR_LEFT. Wire each ESC to the pin for its position.
const uint8_t kMotorPins[cp::airframes::N_MOTORS] = {
  PIN_MOTOR_RIGHT,
  PIN_MOTOR_LEFT,
  PIN_SERVOS[0],
  PIN_SERVOS[1],
};
#else
const uint8_t kMotorPins[cp::airframes::N_MOTORS] = {
  PIN_MOTOR_RIGHT,
  PIN_MOTOR_LEFT,
};
#endif

Servo s_servos[cp::airframes::N_SERVO_SLOTS];

#if MOTOR_PROTOCOL == MOTOR_PROTOCOL_PWM
Servo s_motors[cp::airframes::N_MOTORS];

#elif MOTOR_PROTOCOL == MOTOR_PROTOCOL_ONESHOT125
uint16_t s_motor_us[cp::airframes::N_MOTORS] = {};

#else  // DShot300 or DShot600.
// One PIO state machine per motor. The RP2350 has four state machines per
// PIO block, so the v1 airframes (two motors, or four on the quad) fit on
// PIO1.
static_assert(cp::airframes::N_MOTORS <= 4,
              "DShot uses one PIO1 state machine per motor, max four.");

// pio1 expands to a reinterpret_cast, so this is const, not constexpr.
const PIO s_pio = pio1;
uint      s_dshot_offset = 0;
uint16_t  s_motor_frame[cp::airframes::N_MOTORS] = {};

// The transmit program and frame builder differ for bidirectional DShot
// (inverted line, inverted CRC) versus plain DShot.
#if ENABLE_DSHOT_BIDIR
#define CP_DSHOT_TX_PROGRAM      dshot_bidir_program
#define CP_DSHOT_TX_GET_CONFIG   dshot_bidir_program_get_default_config

// pio2 is free: PIO0 is SBUS, PIO1 is the DShot transmit. One receive sampler
// per motor reads that motor's pin.
const PIO s_pio_rx     = pio2;
uint      s_rx_offset  = 0;
uint32_t  s_erpm[cp::airframes::N_MOTORS] = {};

// ===========================================================================
// BENCH-TUNING REQUIRED: bidirectional-DShot receive timing
// ===========================================================================
// These constants set when and how fast the ESC reply is sampled. They cannot
// be verified in simulation or by inspection. They are first-cut estimates
// from the published bidirectional-DShot description and MUST be checked on
// the bench with a logic analyzer and a real telemetry-capable ESC before the
// eRPM this path produces can be trusted. The likely tuning candidates are the
// telemetry bit rate ratio, the turnaround delay, and the samples per bit.

// The reply is sent at 5/4 of the DShot bit rate (the published ratio).
constexpr uint32_t kTelemBitrateHz = DSHOT_BITRATE_HZ * 5u / 4u;
// Oversample factor for the line sampler.
constexpr uint32_t kSamplesPerBit  = 4;
constexpr uint32_t kRxSampleHz     = kTelemBitrateHz * kSamplesPerBit;
// Microseconds to let the transmit frame finish (16 bits) before releasing
// the line, then for the ESC to start its reply.
constexpr uint32_t kTxFrameUs      = (16u * 1000000u) / DSHOT_BITRATE_HZ + 4u;
constexpr uint32_t kTurnaroundUs   = 30;
// The reply is 21 bits. Sample a little longer to catch the whole thing.
constexpr uint32_t kResponseUs     = (24u * 1000000u) / kTelemBitrateHz + 4u;

uint16_t buildMotorFrame(uint16_t throttle) {
  return cp::libs::dshot::buildFrameBidir(throttle);
}

// Reconstruct the 21-bit GCR frame from the oversampled reply bits and decode
// it to an eRPM, or return -1 on no edge or a decode failure. BENCH-TUNED: the
// run-length to bit conversion assumes kSamplesPerBit samples per telemetry
// bit and a clean single burst.
int32_t reconstructErpm(const uint32_t* words, uint8_t word_count) {
  // Flatten the captured words (MSB first within each, oldest word first) into
  // a single run-length walk. The line idles high, so the reply begins at the
  // first high-to-low edge.
  bool    started   = false;
  bool    cur_level = true;   // idle high
  uint32_t run      = 0;
  uint32_t gcr      = 0;
  uint32_t bits     = 0;

  for (uint8_t w = 0; w < word_count; ++w) {
    for (int b = 31; b >= 0; --b) {
      const bool level = (words[w] >> b) & 1u;
      if (!started) {
        if (level == false) {     // first falling edge: reply start
          started   = true;
          cur_level = false;
          run       = 1;
        }
        continue;
      }
      if (level == cur_level) {
        ++run;
      } else {
        // End of a run: emit round(run / kSamplesPerBit) bits of cur_level.
        const uint32_t n = (run + kSamplesPerBit / 2) / kSamplesPerBit;
        for (uint32_t k = 0; k < n && bits < 21; ++k) {
          gcr = (gcr << 1) | (cur_level ? 1u : 0u);
          ++bits;
        }
        cur_level = level;
        run       = 1;
      }
    }
  }
  if (!started || bits < 20) {
    return -1;
  }
  const uint32_t decoded = cp::libs::dshot::gcrDifferentialDecode(gcr) & 0xFFFFFu;
  return cp::libs::dshot::decodeErpm(decoded);
}

// Send a frame, release the line, sample the reply, decode, restore output.
// BENCH-TUNED timing. Sequential per motor, so it blocks for roughly
// (kTxFrameUs + kTurnaroundUs + kResponseUs) per motor.
void captureMotorTelemetry(uint8_t i) {
  const uint sm  = i;
  const uint pin = kMotorPins[i];

  // Let the transmit frame clock out, then hand the pin to the ESC.
  delayMicroseconds(kTxFrameUs);
  pio_sm_set_consecutive_pindirs(s_pio, sm, pin, 1, /*is_out=*/false);

  // Drop anything the sampler captured during transmit, then wait the reply.
  pio_sm_clear_fifos(s_pio_rx, sm);
  delayMicroseconds(kTurnaroundUs + kResponseUs);

  uint32_t words[8];
  uint8_t  n = 0;
  while (n < 8 && !pio_sm_is_rx_fifo_empty(s_pio_rx, sm)) {
    words[n++] = pio_sm_get(s_pio_rx, sm);
  }

  // Hand the pin back to the transmit machine for the next frame.
  pio_sm_set_consecutive_pindirs(s_pio, sm, pin, 1, /*is_out=*/true);

  const int32_t erpm = reconstructErpm(words, n);
  if (erpm >= 0) {
    s_erpm[i] = static_cast<uint32_t>(erpm);
  }
}
#else
#define CP_DSHOT_TX_PROGRAM      dshot_program
#define CP_DSHOT_TX_GET_CONFIG   dshot_program_get_default_config

uint16_t buildMotorFrame(uint16_t throttle) {
  return cp::libs::dshot::buildFrame(throttle, false);
}
#endif
#endif

}  // anonymous namespace

void out_init() {
#if MOTOR_PROTOCOL == MOTOR_PROTOCOL_PWM
  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    s_motors[i].attach(kMotorPins[i]);
    s_motors[i].writeMicroseconds(ESC_DISARM_PULSE_US);
  }

#elif MOTOR_PROTOCOL == MOTOR_PROTOCOL_ONESHOT125
  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    pinMode(kMotorPins[i], OUTPUT);
    digitalWriteFast(kMotorPins[i], LOW);
    s_motor_us[i] = ESC_DISARM_PULSE_US;
  }

#else  // DShot300 or DShot600.
  s_dshot_offset = pio_add_program(s_pio, &CP_DSHOT_TX_PROGRAM);

  // Eight PIO cycles per DShot bit. clkdiv divides the system clock down to
  // that cycle rate. DShot300 -> 2.4 MHz, DShot600 -> 4.8 MHz.
  const float clkdiv = static_cast<float>(clock_get_hz(clk_sys)) /
                       (static_cast<float>(DSHOT_BITRATE_HZ) * 8.0f);

  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    const uint sm  = i;
    const uint pin = kMotorPins[i];

    pio_gpio_init(s_pio, pin);
    pio_sm_set_consecutive_pindirs(s_pio, sm, pin, 1, /*is_out=*/true);

    pio_sm_config c = CP_DSHOT_TX_GET_CONFIG(s_dshot_offset);
    sm_config_set_sideset_pins(&c, pin);
    // Frame in the top 16 bits of the FIFO word, shifted out MSB first,
    // autopull refilling after each 16-bit frame.
    sm_config_set_out_shift(&c, /*shift_right=*/false, /*autopull=*/true,
                            /*pull_threshold=*/16);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, clkdiv);

    pio_sm_init(s_pio, sm, s_dshot_offset, &c);
    pio_sm_set_enabled(s_pio, sm, true);

    s_motor_frame[i] = buildMotorFrame(cp::libs::dshot::kCmdMotorStop);
  }

#if ENABLE_DSHOT_BIDIR
  // Receive sampler on PIO2, one state machine per motor reading that motor's
  // pin. It free-runs at the bench-tuned oversample rate; out_commit_motors
  // drains the words captured during the reply window. The pin's output is
  // owned by PIO1; PIO can read the pad input regardless, so no gpio_init here.
  s_rx_offset = pio_add_program(s_pio_rx, &dshot_bidir_rx_program);
  const float rx_clkdiv = static_cast<float>(clock_get_hz(clk_sys)) /
                          static_cast<float>(kRxSampleHz);
  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    const uint sm  = i;
    const uint pin = kMotorPins[i];
    pio_sm_config rc = dshot_bidir_rx_program_get_default_config(s_rx_offset);
    sm_config_set_in_pins(&rc, pin);
    sm_config_set_in_shift(&rc, /*shift_right=*/false, /*autopush=*/true,
                           /*push_threshold=*/32);
    sm_config_set_fifo_join(&rc, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&rc, rx_clkdiv);
    pio_sm_init(s_pio_rx, sm, s_rx_offset, &rc);
    pio_sm_set_enabled(s_pio_rx, sm, true);
    s_erpm[i] = 0;
  }
#endif
#endif

  for (uint8_t i = 0; i < cp::airframes::N_SERVOS; ++i) {
    s_servos[i].attach(PIN_SERVOS[i]);
  }
}

void out_set_motor_us(uint8_t idx, uint16_t pulse_us) {
  if (idx < cp::airframes::N_MOTORS) {
#if MOTOR_PROTOCOL == MOTOR_PROTOCOL_PWM
    s_motors[idx].writeMicroseconds(pulse_us);

#elif MOTOR_PROTOCOL == MOTOR_PROTOCOL_ONESHOT125
    s_motor_us[idx] = pulse_us;

#else  // DShot300 or DShot600.
    const uint16_t throttle = cp::libs::dshot::microsecondsToThrottle(
        pulse_us, ESC_IDLE_PULSE_US, ESC_MAX_PULSE_US);
    s_motor_frame[idx] = buildMotorFrame(throttle);
#endif
  }
}

void out_set_servo_us(uint8_t idx, uint16_t pulse_us) {
  if (idx < cp::airframes::N_SERVOS) {
    s_servos[idx].writeMicroseconds(pulse_us);
  }
}

void out_commit_motors() {
#if MOTOR_PROTOCOL == MOTOR_PROTOCOL_PWM
  // Standard PWM: the Servo library already drives the motor pins on its
  // own 50 Hz schedule, so there is nothing to commit.

#elif MOTOR_PROTOCOL == MOTOR_PROTOCOL_ONESHOT125
  // OneShot125: raise every motor pin together, then lower each one when
  // its own buffered pulse width has elapsed. The burst blocks for at most
  // the longest pulse, about 250 microseconds.
  bool lowered[cp::airframes::N_MOTORS] = {};
  uint8_t remaining = cp::airframes::N_MOTORS;

  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    digitalWriteFast(kMotorPins[i], HIGH);
  }
  const uint32_t start = micros();

  while (remaining > 0) {
    const uint32_t elapsed = micros() - start;
    for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
      if (!lowered[i] && elapsed >= s_motor_us[i]) {
        digitalWriteFast(kMotorPins[i], LOW);
        lowered[i] = true;
        --remaining;
      }
    }
  }

#else  // DShot300 or DShot600.
  // Push each motor's frame to its state machine, left-justified into the
  // 32-bit FIFO word so the 16 frame bits shift out MSB first. The FIFO is
  // empty between 1 kHz ticks (a frame clocks out in under 30 us even at
  // DShot300), so the guarded put never blocks the flight loop.
  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    const uint sm = i;
    if (!pio_sm_is_tx_fifo_full(s_pio, sm)) {
      pio_sm_put(s_pio, sm, static_cast<uint32_t>(s_motor_frame[i]) << 16);
    }
  }
#if ENABLE_DSHOT_BIDIR
  // After each frame the ESC replies with eRPM. Capture and decode it per
  // motor. BENCH-TUNED timing; see the banner above.
  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    captureMotorTelemetry(i);
  }
#endif
#endif
}

uint32_t out_get_erpm(uint8_t idx) {
#if MOTOR_PROTOCOL_IS_DSHOT && ENABLE_DSHOT_BIDIR
  return (idx < cp::airframes::N_MOTORS) ? s_erpm[idx] : 0;
#else
  // No eRPM telemetry without bidirectional DShot.
  (void)idx;
  return 0;
#endif
}

}  // namespace cp::hal

#endif  // BUILD_TARGET == BUILD_TARGET_NATIVE

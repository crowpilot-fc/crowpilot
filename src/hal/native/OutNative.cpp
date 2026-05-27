// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/hal/Hal.h"

#include "src/Config.h"

#if BUILD_TARGET == BUILD_TARGET_NATIVE

#include <Arduino.h>
#include <Servo.h>

#include "src/airframes/Airframe.h"

// True when MOTOR_PROTOCOL selects one of the DShot rates.
#define CP_MOTOR_IS_DSHOT                          \
  (MOTOR_PROTOCOL == MOTOR_PROTOCOL_DSHOT300 ||    \
   MOTOR_PROTOCOL == MOTOR_PROTOCOL_DSHOT600)

#if CP_MOTOR_IS_DSHOT
#include <hardware/clocks.h>
#include <hardware/pio.h>

#include "src/libs/Dshot.h"
#include "dshot.pio.h"
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
  s_dshot_offset = pio_add_program(s_pio, &dshot_program);

  // Eight PIO cycles per DShot bit. clkdiv divides the system clock down to
  // that cycle rate. DShot300 -> 2.4 MHz, DShot600 -> 4.8 MHz.
  const float clkdiv = static_cast<float>(clock_get_hz(clk_sys)) /
                       (static_cast<float>(DSHOT_BITRATE_HZ) * 8.0f);

  for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
    const uint sm  = i;
    const uint pin = kMotorPins[i];

    pio_gpio_init(s_pio, pin);
    pio_sm_set_consecutive_pindirs(s_pio, sm, pin, 1, /*is_out=*/true);

    pio_sm_config c = dshot_program_get_default_config(s_dshot_offset);
    sm_config_set_sideset_pins(&c, pin);
    // Frame in the top 16 bits of the FIFO word, shifted out MSB first,
    // autopull refilling after each 16-bit frame.
    sm_config_set_out_shift(&c, /*shift_right=*/false, /*autopull=*/true,
                            /*pull_threshold=*/16);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, clkdiv);

    pio_sm_init(s_pio, sm, s_dshot_offset, &c);
    pio_sm_set_enabled(s_pio, sm, true);

    s_motor_frame[i] =
        cp::libs::dshot::buildFrame(cp::libs::dshot::kCmdMotorStop, false);
  }
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
    s_motor_frame[idx] = cp::libs::dshot::buildFrame(throttle, false);
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
#endif
}

uint32_t out_get_erpm(uint8_t /*idx*/) {
  // No eRPM telemetry on the plain (non-bidirectional) output path. The
  // bidirectional DShot receive path fills this in.
  return 0;
}

}  // namespace cp::hal

#endif  // BUILD_TARGET == BUILD_TARGET_NATIVE

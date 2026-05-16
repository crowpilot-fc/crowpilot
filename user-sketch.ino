// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

// CrowPilot user extension sketch: Caribou scale-feature example.
//
// CrowPilot handles flight (stabilization, mixing, actuator output). This
// sketch handles the scale features of the Caribou twin-engine cargo
// plane: sequenced retractable landing gear, three-position flaps, a cargo
// bay door, and navigation lights.
//
// Enable the hook with ENABLE_USER_HOOK = 1 in src/Config.h. With it off
// (the default) user_setup() and user_tick() still compile but never run.
//
// Transmitter channels used by this sketch. Channels 7 and 8 are reserved
// for the plane stabilizer (CHANNEL_STAB, CHANNEL_ALT_HOLD), so the scale
// features start at channel 9:
//   ch9   gear up/down switch
//   ch10  flap switch (three-position: up / half / full)
//   ch11  cargo bay door switch
//   ch12  navigation lights switch
//
// Note: ch9 and ch10 are also the default live-tune channels
// (LIVE_TUNE_CH_KP and LIVE_TUNE_CH_KD in Config.h). A build that runs
// both this hook and live tuning should remap one side or the other.
//
// Pins. The Caribou airframe claims the motor, servo, I2C, SD, SBUS, LED,
// and companion pins. The pins below are free on the Waveshare RP2350-Tiny
// for this airframe. Writes to a claimed pin are ignored with a warning.

#include <Servo.h>

#include "user_hook/SensorApi.h"

// --- Pin assignments (free GPIO on the Caribou airframe) -----------------

constexpr uint8_t GP_RETRACT_NOSE  = 22;
constexpr uint8_t GP_RETRACT_LEFT  = 26;
constexpr uint8_t GP_RETRACT_RIGHT = 27;
constexpr uint8_t GP_FLAP_SERVO    = 28;
constexpr uint8_t GP_BAY_SERVO     = 15;
constexpr uint8_t GP_NAV_LIGHTS    = 2;

// --- Transmitter channels ------------------------------------------------

constexpr uint8_t CH_GEAR   = 9;
constexpr uint8_t CH_FLAP   = 10;
constexpr uint8_t CH_BAY    = 11;
constexpr uint8_t CH_LIGHTS = 12;

// --- Servo pulse widths --------------------------------------------------

constexpr uint16_t FLAP_UP_US   = 1000;
constexpr uint16_t FLAP_HALF_US = 1500;
constexpr uint16_t FLAP_FULL_US = 2000;
constexpr uint16_t BAY_CLOSED_US = 1100;
constexpr uint16_t BAY_OPEN_US   = 1900;

// --- Retract sequencing --------------------------------------------------

// Electric retracts are driven on/off: HIGH lowers the leg, LOW raises it.
// The three legs deploy in sequence with a delay so the load on the power
// system is staggered and the sequence looks like scale gear.
enum RetractState { GEAR_UP, GEAR_LOWERING, GEAR_DOWN, GEAR_RAISING };

constexpr uint32_t LEG_DELAY_MS = 500;  // delay between successive legs

// --- User-owned state ----------------------------------------------------

static Servo s_flap_servo;
static Servo s_bay_servo;

static RetractState s_retract_state = GEAR_UP;
static uint32_t     s_state_start_ms = 0;

// --- Helpers -------------------------------------------------------------

static void setAllRetracts(bool down) {
  const uint8_t level = down ? HIGH : LOW;
  digitalWrite(GP_RETRACT_NOSE,  level);
  digitalWrite(GP_RETRACT_LEFT,  level);
  digitalWrite(GP_RETRACT_RIGHT, level);
}

// --- Entry points --------------------------------------------------------

void user_setup() {
  pinMode(GP_RETRACT_NOSE,  OUTPUT);
  pinMode(GP_RETRACT_LEFT,  OUTPUT);
  pinMode(GP_RETRACT_RIGHT, OUTPUT);
  pinMode(GP_NAV_LIGHTS,    OUTPUT);

  s_flap_servo.attach(GP_FLAP_SERVO);
  s_bay_servo.attach(GP_BAY_SERVO);

  // Safe ground state: gear up, lights off, flaps up, bay closed.
  setAllRetracts(false);
  digitalWrite(GP_NAV_LIGHTS, LOW);
  s_flap_servo.writeMicroseconds(FLAP_UP_US);
  s_bay_servo.writeMicroseconds(BAY_CLOSED_US);
  s_retract_state = GEAR_UP;
}

void user_tick() {
  // Navigation lights: simple on/off from a two-position switch.
  digitalWrite(GP_NAV_LIGHTS,
               cp::user::channel(CH_LIGHTS) > 1500 ? HIGH : LOW);

  // Flaps: a three-position switch maps to three servo positions.
  const uint16_t flap_sw = cp::user::channel(CH_FLAP);
  if (flap_sw < 1300) {
    s_flap_servo.writeMicroseconds(FLAP_UP_US);
  } else if (flap_sw > 1700) {
    s_flap_servo.writeMicroseconds(FLAP_FULL_US);
  } else {
    s_flap_servo.writeMicroseconds(FLAP_HALF_US);
  }

  // Cargo bay door: a two-position switch opens or closes it.
  s_bay_servo.writeMicroseconds(
      cp::user::channel(CH_BAY) > 1500 ? BAY_OPEN_US : BAY_CLOSED_US);

  // Landing gear: sequenced retract. The nose leg moves first, then the
  // left leg after LEG_DELAY_MS, then the right leg after 2 * LEG_DELAY_MS.
  const bool want_down = cp::user::channel(CH_GEAR) > 1500;

  switch (s_retract_state) {
    case GEAR_UP:
      if (want_down) {
        s_retract_state = GEAR_LOWERING;
        s_state_start_ms = millis();
      }
      break;
    case GEAR_DOWN:
      if (!want_down) {
        s_retract_state = GEAR_RAISING;
        s_state_start_ms = millis();
      }
      break;
    case GEAR_LOWERING:
    case GEAR_RAISING: {
      const uint32_t elapsed = millis() - s_state_start_ms;
      const bool target_down = (s_retract_state == GEAR_LOWERING);
      const uint8_t level = target_down ? HIGH : LOW;
      digitalWrite(GP_RETRACT_NOSE, level);
      if (elapsed > LEG_DELAY_MS) {
        digitalWrite(GP_RETRACT_LEFT, level);
      }
      if (elapsed > 2 * LEG_DELAY_MS) {
        digitalWrite(GP_RETRACT_RIGHT, level);
        s_retract_state = target_down ? GEAR_DOWN : GEAR_UP;
      }
      break;
    }
  }
}

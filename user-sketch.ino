// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

// CrowPilot user extension sketch: DHC-4 Caribou scale features.
//
// CrowPilot's core mixer drives the four primary surfaces and the two
// ESCs. This sketch drives everything else on the Caribou: the flap
// pair, the three retracts (two wing main gear plus the nose), the two
// cargo bay doors, and the nav blink LED. Pin and channel assignments
// match docs/reference/caribou-wiring.svg.
//
// The nose wheel is steered mechanically off the rudder servo. The
// firmware mixer drives the rudder on GP9, and the nose-wheel servo is
// wired to that same signal, so this sketch does not drive a steering
// output and GP3 is free.
//
// Enable the hook with ENABLE_USER_HOOK = 1 in src/Config.h. With it
// off (the default) user_setup() and user_tick() still compile but
// never run.
//
// Transmitter channels (TX side):
//   ch1, 2, 3, 4   AETR primaries  (firmware mixer, not this sketch)
//   ch5, 6         throttle 2 and aileron 2  (firmware-side)
//   ch7            landing gear switch
//   ch8            flap 1 switch  (three-position: up / half / full)
//   ch9            cargo bay door switch
//   ch10           flap 2 switch  (three-position: up / half / full)
//
// Pins below are firmware-free on the WeAct RP2350A_V10 for the Caribou
// airframe. Writes to a firmware-claimed pin are ignored with a warning.

#include <Servo.h>

#include "src/user_hook/SensorApi.h"

// --- Pin assignments (Caribou aux, per caribou-wiring.svg) ----------------

constexpr uint8_t GP_FLAP_1   = 12;  // both wings' flap 1 (Y-harness)
constexpr uint8_t GP_FLAP_2   = 13;  // both wings' flap 2 (Y-harness)
constexpr uint8_t GP_RETRACTS = 22;  // all three retracts share this signal
constexpr uint8_t GP_BAY_1    = 15;  // cargo bay door 1
constexpr uint8_t GP_BAY_2    = 28;  // cargo bay door 2
constexpr uint8_t GP_LED2     =  2;  // nav blink, red left wing / green right wing
// GP3 is free. The nose wheel is steered mechanically off the rudder servo.

// --- Transmitter channels -------------------------------------------------

constexpr uint8_t CH_GEAR   = 7;
constexpr uint8_t CH_FLAP_1 = 8;   // flap 1 three-position switch
constexpr uint8_t CH_BAY    = 9;
constexpr uint8_t CH_FLAP_2 = 10;  // flap 2 three-position switch, independent

// --- Servo pulse widths ---------------------------------------------------

constexpr uint16_t SERVO_UP_US    = 1000;  // retracts up, flaps up, bay closed
constexpr uint16_t SERVO_MID_US   = 1500;  // flap half
constexpr uint16_t SERVO_DOWN_US  = 2000;  // retracts down, flaps full, bay open
constexpr uint16_t SWITCH_HIGH_US = 1700;  // three-position switch upper threshold
constexpr uint16_t SWITCH_LOW_US  = 1300;  // three-position switch lower threshold

// --- LED2 blink ----------------------------------------------------------

constexpr uint32_t LED_HALF_PERIOD_MS = 500;  // 1 Hz square wave

// --- User-owned state ----------------------------------------------------

static Servo s_flap1;
static Servo s_flap2;
static Servo s_retracts;
static Servo s_bay1;
static Servo s_bay2;

static uint32_t s_led_last_toggle_ms = 0;
static bool     s_led_state          = false;

// --- Helpers -------------------------------------------------------------

// Map a three-position switch channel to the up, half, or full flap setting.
static uint16_t flapPosition(uint16_t ch_us) {
  if (ch_us < SWITCH_LOW_US) {
    return SERVO_UP_US;
  }
  if (ch_us > SWITCH_HIGH_US) {
    return SERVO_DOWN_US;
  }
  return SERVO_MID_US;
}

// --- Entry points --------------------------------------------------------

void user_setup() {
  s_flap1.attach(GP_FLAP_1);
  s_flap2.attach(GP_FLAP_2);
  s_retracts.attach(GP_RETRACTS);
  s_bay1.attach(GP_BAY_1);
  s_bay2.attach(GP_BAY_2);
  pinMode(GP_LED2, OUTPUT);

  // Safe ground state: gear up, flaps up, bay closed, nav LED off. The
  // pilot raises the gear after takeoff and lowers it before landing, so
  // everything starts in the cleanest possible config.
  s_retracts.writeMicroseconds(SERVO_UP_US);
  s_flap1.writeMicroseconds(SERVO_UP_US);
  s_flap2.writeMicroseconds(SERVO_UP_US);
  s_bay1.writeMicroseconds(SERVO_UP_US);
  s_bay2.writeMicroseconds(SERVO_UP_US);
  digitalWrite(GP_LED2, LOW);

  s_led_state          = false;
  s_led_last_toggle_ms = millis();
}

void user_tick() {
  // Landing gear. ch7 high lowers all three retracts together.
  const bool gear_down = cp::user::channel(CH_GEAR) > 1500;
  s_retracts.writeMicroseconds(gear_down ? SERVO_DOWN_US : SERVO_UP_US);

  // Flaps. Flap 1 and flap 2 are independent, each a three-position switch
  // (up, half, full) on its own channel. Each output drives that flap on
  // both wings, since the left and right servos are tied together. To move
  // the two flaps together, send one flap switch on both channels at the
  // transmitter.
  s_flap1.writeMicroseconds(flapPosition(cp::user::channel(CH_FLAP_1)));
  s_flap2.writeMicroseconds(flapPosition(cp::user::channel(CH_FLAP_2)));

  // Cargo bay doors. Both doors share the ch9 switch.
  const uint16_t bay_us =
      cp::user::channel(CH_BAY) > 1500 ? SERVO_DOWN_US : SERVO_UP_US;
  s_bay1.writeMicroseconds(bay_us);
  s_bay2.writeMicroseconds(bay_us);

  // Nav LED. 1 Hz square wave from a free-running counter so the strobe
  // is visible day and night, on both wings together.
  const uint32_t now_ms = millis();
  if ((now_ms - s_led_last_toggle_ms) >= LED_HALF_PERIOD_MS) {
    s_led_state          = !s_led_state;
    s_led_last_toggle_ms = now_ms;
    digitalWrite(GP_LED2, s_led_state ? HIGH : LOW);
  }
}

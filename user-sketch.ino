// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

// CrowPilot user extension sketch: DHC-4 Caribou scale features.
//
// CrowPilot's core mixer drives the four primary surfaces and the two
// ESCs. This sketch drives everything else on the Caribou: the flap
// pair, the three retracts (two wing main gear plus the nose), the two
// cargo bay doors, the nose wheel steering servo, and the nav blink
// LED. Pin and channel assignments match docs/reference/caribou-wiring.svg.
//
// Enable the hook with ENABLE_USER_HOOK = 1 in src/Config.h. With it
// off (the default) user_setup() and user_tick() still compile but
// never run.
//
// Transmitter channels (TX side):
//   ch1, 2, 3, 4   AETR primaries  (firmware mixer, not this sketch)
//   ch5, 6         throttle 2 and aileron 2  (firmware-side)
//   ch7            landing gear switch
//   ch8            flap switch  (three-position: up / half / full)
//   ch9            cargo bay door switch
//
// Channel 4 (rudder) is also read here for nose-wheel steering.
//
// Nose-wheel steering follows the rudder stick when the gear is down,
// and centres when the gear is up. The pilot can taxi but the wheel
// does not flap around mid-flight.
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
constexpr uint8_t GP_STEERING =  3;  // nose wheel steering
constexpr uint8_t GP_LED2     =  2;  // nav blink, red left wing / green right wing

// --- Transmitter channels -------------------------------------------------

constexpr uint8_t CH_RUDDER = 4;  // rudder stick, used for nose steering
constexpr uint8_t CH_GEAR   = 7;
constexpr uint8_t CH_FLAP   = 8;
constexpr uint8_t CH_BAY    = 9;

// --- Servo pulse widths ---------------------------------------------------

constexpr uint16_t SERVO_UP_US    = 1000;  // retracts up, flaps up, bay closed
constexpr uint16_t SERVO_MID_US   = 1500;  // centred (steering, flap half)
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
static Servo s_steering;

static uint32_t s_led_last_toggle_ms = 0;
static bool     s_led_state          = false;

// --- Entry points --------------------------------------------------------

void user_setup() {
  s_flap1.attach(GP_FLAP_1);
  s_flap2.attach(GP_FLAP_2);
  s_retracts.attach(GP_RETRACTS);
  s_bay1.attach(GP_BAY_1);
  s_bay2.attach(GP_BAY_2);
  s_steering.attach(GP_STEERING);
  pinMode(GP_LED2, OUTPUT);

  // Safe ground state: gear up, flaps up, bay closed, steering centred,
  // nav LED off. The pilot raises the gear after takeoff and lowers it
  // before landing; everything starts in the cleanest possible config.
  s_retracts.writeMicroseconds(SERVO_UP_US);
  s_flap1.writeMicroseconds(SERVO_UP_US);
  s_flap2.writeMicroseconds(SERVO_UP_US);
  s_bay1.writeMicroseconds(SERVO_UP_US);
  s_bay2.writeMicroseconds(SERVO_UP_US);
  s_steering.writeMicroseconds(SERVO_MID_US);
  digitalWrite(GP_LED2, LOW);

  s_led_state          = false;
  s_led_last_toggle_ms = millis();
}

void user_tick() {
  // Landing gear. ch7 high lowers all three retracts together.
  const bool gear_down = cp::user::channel(CH_GEAR) > 1500;
  s_retracts.writeMicroseconds(gear_down ? SERVO_DOWN_US : SERVO_UP_US);

  // Flaps. Three-position switch on ch8 maps to up, half, full. Both
  // flap servos on each wing share the channel via the Y-harness, so
  // flap1 and flap2 outputs always carry the same pulse width.
  const uint16_t flap_sw = cp::user::channel(CH_FLAP);
  uint16_t flap_us;
  if (flap_sw < SWITCH_LOW_US) {
    flap_us = SERVO_UP_US;
  } else if (flap_sw > SWITCH_HIGH_US) {
    flap_us = SERVO_DOWN_US;
  } else {
    flap_us = SERVO_MID_US;
  }
  s_flap1.writeMicroseconds(flap_us);
  s_flap2.writeMicroseconds(flap_us);

  // Cargo bay doors. Both doors share the ch9 switch.
  const uint16_t bay_us =
      cp::user::channel(CH_BAY) > 1500 ? SERVO_DOWN_US : SERVO_UP_US;
  s_bay1.writeMicroseconds(bay_us);
  s_bay2.writeMicroseconds(bay_us);

  // Nose wheel steering. Follow the rudder stick on the ground, centre
  // it in flight. The rule is: steering is live only when the gear is
  // commanded down. In the air the gear is up and the wheel parks.
  const uint16_t steering_us =
      gear_down ? cp::user::channel(CH_RUDDER) : SERVO_MID_US;
  s_steering.writeMicroseconds(steering_us);

  // Nav LED. 1 Hz square wave from a free-running counter so the strobe
  // is visible day and night, on both wings together.
  const uint32_t now_ms = millis();
  if ((now_ms - s_led_last_toggle_ms) >= LED_HALF_PERIOD_MS) {
    s_led_state          = !s_led_state;
    s_led_last_toggle_ms = now_ms;
    digitalWrite(GP_LED2, s_led_state ? HIGH : LOW);
  }
}

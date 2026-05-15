// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

#include "actuators/Output.h"
#include "estimation/Attitude.h"
#include "failsafe/Failsafe.h"
#include "modes/FlightMode.h"
#include "radio/Receiver.h"
#include "sensors/Barometer.h"
#include "sensors/Imu.h"
#include "user_hook/PinGuard.h"

// Read-only sensor and flight-state API for user-sketch.ino. Every accessor
// returns a value or a small struct by copy; user code cannot reach into or
// mutate live flight state. Include this header at the top of
// user-sketch.ino. It also installs the pin-claim guard (see the bottom of
// this file).

namespace cp::user {

// IMU reading in engineering units.
struct Imu {
  float gx_dps;
  float gy_dps;
  float gz_dps;
  float ax_g;
  float ay_g;
  float az_g;
  float temp_c;
};

// Barometer reading. `present` is false on BARO_NONE builds and on a
// chip-init failure.
struct Baro {
  float pressure_pa;
  float temperature_c;
  float altitude_m;
  bool  present;
};

// Latest IMU sample.
inline Imu imu() {
  const auto& s = cp::sensors::imu::latest();
  return Imu{s.gx_dps, s.gy_dps, s.gz_dps, s.ax_g, s.ay_g, s.az_g, s.temp_c};
}

// Latest barometer sample.
inline Baro baro() {
  const auto& s = cp::sensors::baro::latest();
  return Baro{s.pressure_pa, s.temperature_c, s.altitude_m,
              cp::sensors::baro::is_present()};
}

// Receiver channel pulse width in microseconds. `ch_1based` is 1..16.
// Out-of-range channels return a safe centered 1500.
inline uint16_t channel(uint8_t ch_1based) {
  if (ch_1based < 1 || ch_1based > cp::radio::MAX_CHANNELS) {
    return 1500;
  }
  return cp::radio::state().channel_us[ch_1based - 1];
}

// Transition fader, 0.0 forward to 1.0 hover.
inline float fader() {
  return cp::modes::fader();
}

// True while the motors are armed.
inline bool armed() {
  return cp::actuators::arm_state() == cp::actuators::ArmState::ARMED;
}

// True while the lost-link failsafe override is active.
inline bool failsafeActive() {
  return cp::failsafe::state().active;
}

// Attitude estimate in the forward-flight Euler convention, degrees.
inline cp::estimation::attitude::Euler attitudeForward() {
  return cp::estimation::attitude::eulerForwardFlight();
}

}  // namespace cp::user

// Pin-claim guard. From this point on in the user-sketch translation unit,
// the Arduino pin calls redirect to the guarded entry points: writes to a
// firmware-claimed pin are ignored with a one-time warning, free pins pass
// through. Firmware code under src/ never includes this header and so is
// never redirected. The macros sit after every #include above so the
// included headers see the real Arduino functions.
#define pinMode(pin, mode)      cp::user_hook::pins::guardPinMode((pin), (mode))
#define digitalWrite(pin, val)  cp::user_hook::pins::guardDigitalWrite((pin), (val))
#define analogWrite(pin, val)   cp::user_hook::pins::guardAnalogWrite((pin), (val))

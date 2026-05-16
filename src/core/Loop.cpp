// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "core/Loop.h"

#include <Arduino.h>

#include "Config.h"
#include "actuators/EscCalibrate.h"
#include "actuators/Output.h"
#include "airframes/Airframe.h"
#include "control/DesiredState.h"
#include "control/Pid.h"
#include "estimation/Attitude.h"
#include "failsafe/Failsafe.h"
#include "hal/Led.h"
#include "modes/FlightMode.h"
#include "params/LiveTune.h"
#include "params/Params.h"
#include "radio/Receiver.h"
#include "sensors/Barometer.h"
#include "sensors/Imu.h"
#include "sensors/ImuCalibrate.h"
#include "telemetry/SdLogger.h"
#include "user_hook/UserHook.h"

#if AIRFRAME != AIRFRAME_TAILSITTER_BICOPTER
#include "control/PlaneStab.h"
#endif

// The 1 kHz cooperative super-loop, per PROJECT_OVERVIEW section 4.
// init() brings every module up in dependency order and halts on a
// fatal sensor failure before any motor output. tick() runs one
// iteration: sense, estimate, command, stabilize, mix, output, then
// regulates to the loop period.

namespace cp::core {

namespace {

constexpr uint32_t kLoopPeriodUs = 1000000UL / LOOP_HZ;

uint32_t s_last_tick_us   = 0;
uint32_t s_loop_period_us = 0;
uint32_t s_tick_count     = 0;

}  // anonymous namespace

void init() {
  Serial.begin(115200);
  cp::hal::led_init();

  if (Serial) {
    Serial.print("CrowPilot boot. LOOP_HZ=");
    Serial.print(LOOP_HZ);
    Serial.print(", period_us=");
    Serial.println(kLoopPeriodUs);
  }

  // The parameter registry seeds the control gains, so it comes up
  // before any control module reads a gain.
  cp::params::init();

  // The IMU is mandatory. A missing IMU halts here, with a fast LED
  // blink, before any motor output is possible.
  if (!cp::sensors::imu::init()) {
    if (Serial) {
      Serial.println("ERROR: IMU init failed. Halting before motor output.");
    }
    cp::hal::haltWithFastBlink();
  }
  if (Serial) {
    Serial.println("IMU OK");
  }

  // The receiver is mandatory.
  if (!cp::radio::init()) {
    if (Serial) {
      Serial.println("ERROR: Receiver init failed (PIO program could not load).");
    }
    cp::hal::haltWithFastBlink();
  }
  if (Serial) {
    Serial.println("Receiver OK (PIO SM 0 active)");
  }

  cp::failsafe::init();

  // The barometer is optional. A BARO_NONE build or a chip failure
  // leaves the firmware running without altitude.
  if (!cp::sensors::baro::init()) {
    if (Serial) {
      Serial.println("WARN: Barometer init failed. Continuing without altitude.");
    }
  } else if (Serial) {
    Serial.println(cp::sensors::baro::is_present()
                       ? "Barometer OK"
                       : "Barometer disabled (BARO_NONE)");
  }

  cp::estimation::attitude::init();
  cp::control::desired::init();
  cp::control::pid::init();
  cp::airframes::init();
  cp::actuators::init();
  if (Serial) {
    Serial.println("Actuators OK (NOT_ARMED)");
  }

  // One-shot bench routines. Each does not return. The actuator stage
  // is up before ESC calibration so the motor pins are configured.
#if ENABLE_ESC_CALIBRATION
  cp::actuators::esc_calibrate::run();
#endif
#if ENABLE_IMU_CALIBRATION
  cp::sensors::imu_calibrate::run();
#endif

  cp::modes::init();
  cp::params::live::init();
  cp::telemetry::init();
  if (Serial) {
    if (cp::telemetry::is_active()) {
      Serial.print("Logger OK -> ");
      Serial.println(cp::telemetry::current_filename());
    } else {
      Serial.println("Logger inactive (no SD card, init failed, or disabled).");
    }
  }
  cp::user_hook::init();
#if AIRFRAME != AIRFRAME_TAILSITTER_BICOPTER
  cp::control::plane_stab::init();
#endif
}

void tick() {
  const uint32_t tick_start_us = micros();

  // Measure the loop period: the interval since the previous tick.
  // Zero on the first tick, before a delta has been captured.
  if (s_last_tick_us != 0) {
    s_loop_period_us = tick_start_us - s_last_tick_us;
  }
  s_last_tick_us = tick_start_us;
  const float dt_s = static_cast<float>(s_loop_period_us) * 1.0e-6f;

  // Receiver, then the failsafe override. Downstream consumers read the
  // failsafe-effective channels, never the raw receiver channels.
  cp::radio::poll();
  cp::failsafe::update();
  const uint16_t* channels = cp::failsafe::channels().ch_us;

  // Transition fader.
  cp::modes::update(channels, dt_s);
  const float fader = cp::modes::fader();

  const bool armed =
      cp::actuators::arm_state() == cp::actuators::ArmState::ARMED;

  // Live tuning reads the raw receiver channels.
  cp::params::live::update(cp::radio::state().channel_us, armed);

  // Sensors. The barometer self-rate-limits internally.
  const bool imu_ok = cp::sensors::imu::read();
  cp::sensors::baro::read();

  // Attitude estimate. A failed IMU read leaves the estimate to coast
  // on the previous tick rather than feeding it a stale sample.
  if (imu_ok) {
    const cp::sensors::imu::Sample& s = cp::sensors::imu::latest();
    cp::estimation::attitude::update(s.gx_dps, s.gy_dps, s.gz_dps,
                                     s.ax_g, s.ay_g, s.az_g, dt_s);
  }

  // Pilot setpoints.
  cp::control::desired::update(channels);
  const cp::control::desired::State& desired =
      cp::control::desired::current();

#if AIRFRAME == AIRFRAME_TAILSITTER_BICOPTER
  // Tailsitter: the quaternion-error PID stabilizer feeds the mixer.
  constexpr float kFlyingThrottleMin =
      static_cast<float>(ARM_THROTTLE_MAX_US - RC_MIN_US) /
      static_cast<float>(RC_MAX_US - RC_MIN_US);

  const cp::estimation::attitude::Euler error =
      cp::estimation::attitude::errorToReference(fader, desired.roll_deg,
                                                 desired.pitch_deg);
  const bool flying = armed && desired.throttle > kFlyingThrottleMin;

  cp::control::pid::update(error, cp::estimation::attitude::bodyRates(),
                           desired.yaw_rate_dps, fader, flying, dt_s);

  const cp::control::pid::Output& pid = cp::control::pid::output();
  cp::airframes::update(desired.throttle,
                        pid.roll, pid.pitch, pid.yaw,
                        desired.roll_passthru, desired.pitch_passthru,
                        desired.yaw_passthru, fader);
#else
  // Fixed-wing: the plane stabilizer feeds the plane mixer, which reads
  // the stabilizer output directly.
  const cp::sensors::baro::Sample& baro = cp::sensors::baro::latest();
  cp::control::plane_stab::update(
      cp::estimation::attitude::eulerForwardFlight(),
      cp::estimation::attitude::bodyRates(),
      desired,
      baro.altitude_m,
      cp::sensors::baro::is_present() && baro.valid,
      channels,
      dt_s);
  cp::airframes::update(desired.throttle, 0.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, fader);
#endif

  // Actuator output: arm logic, OneShot125 emit, servo PWM.
  cp::actuators::update(cp::airframes::output(), desired.throttle,
                        desired.ch5_us);

  // User extension and telemetry. Each is internally rate-limited.
  cp::user_hook::tick();
  cp::telemetry::tick();

#if ENABLE_DEBUG_PRINTS
  if (Serial && (s_tick_count % DEBUG_PRINT_INTERVAL_TICKS) == 0) {
    const cp::estimation::attitude::Euler att =
        cp::estimation::attitude::eulerForwardFlight();
    Serial.print("dt_us=");  Serial.print(s_loop_period_us);
    Serial.print(" fader="); Serial.print(fader, 2);
    Serial.print(" roll=");  Serial.print(att.roll_deg, 1);
    Serial.print(" pitch="); Serial.print(att.pitch_deg, 1);
    Serial.print(" armed="); Serial.println(armed ? 1 : 0);
  }
#endif

  cp::hal::led_tick(tick_start_us);
  ++s_tick_count;

  // Regulate to the loop period. An overrun simply starts the next
  // tick late; the period measurement records it.
  while ((micros() - tick_start_us) < kLoopPeriodUs) {
    // Spin until the next tick is due.
  }
}

uint32_t last_loop_period_us() {
  return s_loop_period_us;
}

}  // namespace cp::core

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/core/Loop.h"

#include <Arduino.h>

#include "src/Config.h"
#include "src/actuators/EscCalibrate.h"
#include "src/actuators/Output.h"
#include "src/airframes/Airframe.h"
#include "src/control/DesiredState.h"
#include "src/control/Pid.h"
#include "src/estimation/Attitude.h"
#include "src/failsafe/Failsafe.h"
#include "src/hal/Led.h"
#include "src/modes/FlightMode.h"
#include "src/params/LiveTune.h"
#include "src/params/Params.h"
#include "src/radio/Receiver.h"
#include "src/sensors/Barometer.h"
#include "src/sensors/Imu.h"
#include "src/sensors/ImuCalibrate.h"
#include "src/telemetry/SdLogger.h"
#include "src/user_hook/UserHook.h"

#if AIRFRAME != AIRFRAME_TAILSITTER_BICOPTER
#include "src/control/PlaneStab.h"
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

// Loop-period statistics, accumulated for the 1 Hz DEV report.
uint32_t s_period_sum   = 0;
uint32_t s_period_max   = 0;
uint32_t s_period_count = 0;

const char* modeName(cp::modes::FlightMode m) {
  switch (m) {
    case cp::modes::FlightMode::HOVER:         return "hover";
    case cp::modes::FlightMode::TRANSITIONING: return "transitioning";
    case cp::modes::FlightMode::FORWARD:       return "forward";
  }
  return "?";
}

// Serial debug output. Each block is gated on a compile-time flag in
// Config.h and on a live USB serial connection, and is rate-limited.
// Called once per tick.
void debugOutput() {
  if (!Serial) {
    return;
  }

#if DEBUG_PRINT_DEV
  // Loop-period statistics, accumulated each tick, reported at 1 Hz.
  s_period_sum += s_loop_period_us;
  if (s_loop_period_us > s_period_max) {
    s_period_max = s_loop_period_us;
  }
  ++s_period_count;
  if (s_period_count >= DEBUG_LOOP_REPORT_TICKS) {
    Serial.print("loop_period_us avg=");
    Serial.print(s_period_sum / s_period_count);
    Serial.print(" max=");
    Serial.print(s_period_max);
    Serial.print(" (over ");
    Serial.print(s_period_count);
    Serial.println(" iter)");
    s_period_sum   = 0;
    s_period_max   = 0;
    s_period_count = 0;
  }
  if ((s_tick_count % DEBUG_DEV_INTERVAL_TICKS) == 0) {
    const cp::estimation::attitude::Euler att =
        cp::estimation::attitude::eulerForwardFlight();
    const cp::control::pid::Output& pid = cp::control::pid::output();
    Serial.print("DEV imu=");
    Serial.print(cp::sensors::imu::is_healthy() ? "OK" : "FAULT");
    Serial.print(" rpy=(");  Serial.print(att.roll_deg, 1);
    Serial.print(",");       Serial.print(att.pitch_deg, 1);
    Serial.print(",");       Serial.print(att.yaw_deg, 1);
    Serial.print(") fs=");   Serial.print(cp::failsafe::state().active ? 1 : 0);
    Serial.print(" fader="); Serial.print(cp::modes::fader(), 2);
    Serial.print(" mode=");  Serial.print(modeName(cp::modes::mode()));
    Serial.print(" armed=");
    Serial.print(cp::actuators::arm_state() == cp::actuators::ArmState::ARMED
                     ? 1 : 0);
    Serial.print(" pid=(");  Serial.print(pid.roll, 2);
    Serial.print(",");       Serial.print(pid.pitch, 2);
    Serial.print(",");       Serial.print(pid.yaw, 2);
    Serial.print(") baro=");
    Serial.print(cp::sensors::baro::is_present() ? "OK" : "off");
    Serial.print(" tlm=");
    Serial.println(cp::telemetry::is_active()
                       ? cp::telemetry::current_filename() : "off");
  }
#endif

#if DEBUG_PRINT_IMU
  if ((s_tick_count % DEBUG_STREAM_INTERVAL_TICKS) == 0) {
    const cp::sensors::imu::Sample& s = cp::sensors::imu::latest();
    Serial.print("imu a=("); Serial.print(s.ax_g, 2);
    Serial.print(",");       Serial.print(s.ay_g, 2);
    Serial.print(",");       Serial.print(s.az_g, 2);
    Serial.print(")g g=(");  Serial.print(s.gx_dps, 1);
    Serial.print(",");       Serial.print(s.gy_dps, 1);
    Serial.print(",");       Serial.print(s.gz_dps, 1);
    Serial.print(")dps T="); Serial.print(s.temp_c, 1);
    Serial.println("C");
  }
#endif

#if DEBUG_PRINT_RX
  if ((s_tick_count % DEBUG_STREAM_INTERVAL_TICKS) == 0) {
    const cp::radio::State& rx = cp::radio::state();
    Serial.print("rx ch=[");
    for (uint8_t i = 0; i < 6; ++i) {
      Serial.print(rx.channel_us[i]);
      if (i < 5) {
        Serial.print(",");
      }
    }
    Serial.print("] valid="); Serial.print(rx.channels_valid ? 1 : 0);
    Serial.print(" fs=");     Serial.print(rx.failsafe_active ? 1 : 0);
    Serial.print(" fl=");     Serial.print(rx.frame_lost_flag ? 1 : 0);
    Serial.print(" lost=");   Serial.println(rx.lost_frames_count);
  }
#endif

#if DEBUG_PRINT_FAILSAFE
  if ((s_tick_count % DEBUG_STREAM_INTERVAL_TICKS) == 0) {
    const cp::failsafe::State&    fs = cp::failsafe::state();
    const cp::failsafe::Channels& ch = cp::failsafe::channels();
    Serial.print("fs active=");  Serial.print(fs.active ? 1 : 0);
    Serial.print(" link_to=");   Serial.print(fs.link_timeout ? 1 : 0);
    Serial.print(" oor=");       Serial.print(fs.channel_out_of_range ? 1 : 0);
    Serial.print(" rx_fs=");     Serial.print(fs.rx_protocol_failsafe ? 1 : 0);
    Serial.print(" rx_fl=");     Serial.print(fs.rx_frame_lost ? 1 : 0);
    Serial.print(" eff=[");
    for (uint8_t i = 0; i < 6; ++i) {
      Serial.print(ch.ch_us[i]);
      if (i < 5) {
        Serial.print(",");
      }
    }
    Serial.println("]");
  }
#endif

#if DEBUG_PRINT_ATTITUDE
  if ((s_tick_count % DEBUG_STREAM_INTERVAL_TICKS) == 0) {
    const cp::estimation::attitude::Euler att =
        cp::estimation::attitude::eulerForwardFlight();
    const cp::estimation::attitude::Quaternion& q =
        cp::estimation::attitude::quaternion();
    Serial.print("rpy=(");   Serial.print(att.roll_deg, 1);
    Serial.print(",");       Serial.print(att.pitch_deg, 1);
    Serial.print(",");       Serial.print(att.yaw_deg, 1);
    Serial.print(")deg q=(");Serial.print(q.w, 4);
    Serial.print(",");       Serial.print(q.x, 4);
    Serial.print(",");       Serial.print(q.y, 4);
    Serial.print(",");       Serial.print(q.z, 4);
    Serial.println(")");
  }
#endif

#if DEBUG_PRINT_MODE
  if ((s_tick_count % DEBUG_STREAM_INTERVAL_TICKS) == 0) {
    Serial.print("mode fader="); Serial.print(cp::modes::fader(), 2);
    Serial.print(" mode=");      Serial.print(modeName(cp::modes::mode()));
    Serial.print(" ch6=");
    Serial.println(cp::failsafe::channels().ch_us[CHANNEL_TRANSITION - 1]);
  }
#endif

#if DEBUG_PRINT_MIXER
  if ((s_tick_count % DEBUG_STREAM_INTERVAL_TICKS) == 0) {
    const cp::airframes::Output& mix = cp::airframes::output();
    Serial.print("mix m=[");
    for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
      Serial.print(mix.motor[i], 2);
      if (i + 1 < cp::airframes::N_MOTORS) {
        Serial.print(",");
      }
    }
    Serial.print("] s=[");
    for (uint8_t i = 0; i < cp::airframes::N_SERVOS; ++i) {
      Serial.print(mix.servo[i], 2);
      if (i + 1 < cp::airframes::N_SERVOS) {
        Serial.print(",");
      }
    }
    Serial.println("]");
  }
#endif
}

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

  debugOutput();

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

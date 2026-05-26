// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/core/Loop.h"

#include <Arduino.h>
#include <stdio.h>

#include "src/Config.h"
#include "src/actuators/EscCalibrate.h"
#include "src/actuators/Output.h"
#include "src/airframes/Airframe.h"
#include "src/cli/Cli.h"
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

#if AIRFRAME != AIRFRAME_TAILSITTER_BICOPTER && AIRFRAME != AIRFRAME_QUAD_X
#include "src/control/PlaneStab.h"
#endif

// NATIVE and SITL are implemented (src/hal/native/ and src/hal/sim/).
// HIL is still scaffolded, so catch it here with a clear message rather
// than an opaque linker error from undefined cp::hal symbols.
#if BUILD_TARGET == BUILD_TARGET_HIL
#error "The HIL HAL is not implemented. Build NATIVE or SITL."
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
uint32_t s_loop_period_us = kLoopPeriodUs;
uint32_t s_tick_count     = 0;

// Count of ticks whose work exceeded the loop period, so the regulator
// could not pad them to the nominal rate.
uint32_t s_overrun_count = 0;

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
    Serial.print(" overruns=");
    Serial.print(s_overrun_count);
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
    const cp::failsafe::State&      fs  = cp::failsafe::state();
    const uint16_t*                 ch  = cp::failsafe::channels().ch_us;
    const cp::airframes::Output&    mix = cp::airframes::output();
    Serial.print("DEV imu=");
    Serial.print(cp::sensors::imu::is_healthy() ? "OK" : "FAULT");
    Serial.print(" rpy=(");   Serial.print(att.roll_deg, 1);
    Serial.print(",");        Serial.print(att.pitch_deg, 1);
    Serial.print(",");        Serial.print(att.yaw_deg, 1);
    Serial.print(") fs=");    Serial.print(fs.active ? 1 : 0);
    Serial.print("/to=");     Serial.print(fs.link_timeout ? 1 : 0);
    Serial.print("/rxfs=");   Serial.print(fs.rx_protocol_failsafe ? 1 : 0);
    Serial.print(" ch=[");
    for (uint8_t i = 0; i < 6; ++i) {
      Serial.print(ch[i]);
      if (i < 5) {
        Serial.print(",");
      }
    }
    Serial.print("] fader="); Serial.print(cp::modes::fader(), 2);
    Serial.print(" mode=");   Serial.print(modeName(cp::modes::mode()));
    Serial.print(" armed=");
    Serial.print(cp::actuators::arm_state() == cp::actuators::ArmState::ARMED
                     ? 1 : 0);
    Serial.print(" pid=(");   Serial.print(pid.roll, 2);
    Serial.print(",");        Serial.print(pid.pitch, 2);
    Serial.print(",");        Serial.print(pid.yaw, 2);
    Serial.print(") mix=[m");
    for (uint8_t i = 0; i < cp::airframes::N_MOTORS; ++i) {
      Serial.print(mix.motor[i], 2);
      if (i + 1 < cp::airframes::N_MOTORS) {
        Serial.print(",");
      }
    }
    Serial.print("|s");
    for (uint8_t i = 0; i < cp::airframes::N_SERVOS; ++i) {
      Serial.print(mix.servo[i], 2);
      if (i + 1 < cp::airframes::N_SERVOS) {
        Serial.print(",");
      }
    }
    Serial.print("] baro=");
    Serial.print(cp::sensors::baro::is_present() ? "OK" : "off");
    Serial.print(" alt=");    Serial.print(cp::sensors::baro::latest().altitude_m, 1);
    Serial.print(" lost=");   Serial.print(cp::radio::state().lost_frames_count);
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
    Serial.print(" ch15=");
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

#if ENABLE_CONFIG_CLI
  // Configurator live telemetry. Emitted only while a channel has issued
  // "cp stream on", and sent to whichever channel asked (USB or the
  // companion UART). Fields, space-separated:
  // roll pitch yaw armed failsafe mode loop_us ch1..ch6
  // alt_m yawrate_dps ax_g ay_g az_g.
  if (cp::cli::streaming() &&
      (s_tick_count % DEBUG_STREAM_INTERVAL_TICKS) == 0) {
    const cp::estimation::attitude::Euler att =
        cp::estimation::attitude::eulerForwardFlight();
    const cp::failsafe::State& fs = cp::failsafe::state();
    const uint16_t*            ch = cp::failsafe::channels().ch_us;
    const bool armed =
        cp::actuators::arm_state() == cp::actuators::ArmState::ARMED;
    const cp::sensors::baro::Sample& baro = cp::sensors::baro::latest();
    const cp::sensors::imu::Sample&  imu  = cp::sensors::imu::latest();
    char roll[12], pitch[12], yaw[12], alt[12], gz[12], ax[12], ay[12], az[12];
    dtostrf(att.roll_deg,    0, 1, roll);
    dtostrf(att.pitch_deg,   0, 1, pitch);
    dtostrf(att.yaw_deg,     0, 1, yaw);
    dtostrf(baro.altitude_m, 0, 1, alt);   // metres, relative to ground baseline
    dtostrf(imu.gz_dps,      0, 1, gz);    // yaw rate, for the turn coordinator
    dtostrf(imu.ax_g,        0, 2, ax);    // accel x/y/z for slip and G-load
    dtostrf(imu.ay_g,        0, 2, ay);
    dtostrf(imu.az_g,        0, 2, az);
    char line[220];
    snprintf(line, sizeof(line),
             "cp tlm %s %s %s %d %d %s %lu %d %d %d %d %d %d %s %s %s %s %s",
             roll, pitch, yaw, armed ? 1 : 0, fs.active ? 1 : 0,
             modeName(cp::modes::mode()),
             static_cast<unsigned long>(s_loop_period_us),
             ch[0], ch[1], ch[2], ch[3], ch[4], ch[5],
             alt, gz, ax, ay, az);
    cp::cli::emit_telemetry(line);
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
#if ENABLE_CONFIG_CLI
  cp::cli::init();
#endif
#if AIRFRAME != AIRFRAME_TAILSITTER_BICOPTER && AIRFRAME != AIRFRAME_QUAD_X
  cp::control::plane_stab::init();
#endif
}

void tick() {
  const uint32_t tick_start_us = micros();

  // Measure the loop period: the interval since the previous tick. The
  // first tick keeps the nominal seed value, before a delta exists.
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
#elif AIRFRAME == AIRFRAME_QUAD_X
  // Quadcopter: self-leveling angle mode. The stick angle setpoints minus the
  // measured attitude form the roll and pitch error for the angle PID; yaw is
  // a rate loop. fader = 1.0 selects the hover gain set, the right regime for
  // a quad. Rate (acro) mode is a later addition.
  constexpr float kFlyingThrottleMin =
      static_cast<float>(ARM_THROTTLE_MAX_US - RC_MIN_US) /
      static_cast<float>(RC_MAX_US - RC_MIN_US);

  const cp::estimation::attitude::Euler att =
      cp::estimation::attitude::eulerForwardFlight();
  const cp::estimation::attitude::Euler error{
      desired.roll_deg - att.roll_deg,
      desired.pitch_deg - att.pitch_deg,
      0.0f};

  const bool flying = armed && desired.throttle > kFlyingThrottleMin;

  cp::control::pid::update(error, cp::estimation::attitude::bodyRates(),
                           desired.yaw_rate_dps, 1.0f, flying, dt_s);

  const cp::control::pid::Output& pid = cp::control::pid::output();
  cp::airframes::update(desired.throttle,
                        pid.roll, pid.pitch, pid.yaw,
                        0.0f, 0.0f, 0.0f, 1.0f);
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
                        desired.arm_us);

  // User extension and telemetry. Each is internally rate-limited.
  cp::user_hook::tick();
  cp::telemetry::tick();

  debugOutput();

#if ENABLE_CONFIG_CLI
  cp::cli::poll();
#endif

  cp::hal::led_tick(tick_start_us);
  ++s_tick_count;

  // Regulate to the loop period. If the tick's work already exceeded the
  // period the regulator cannot pad it: count the overrun, and the next
  // tick simply starts late.
  if ((micros() - tick_start_us) >= kLoopPeriodUs) {
    ++s_overrun_count;
  }
  while ((micros() - tick_start_us) < kLoopPeriodUs) {
    // Spin until the next tick is due.
  }
}

uint32_t last_loop_period_us() {
  return s_loop_period_us;
}

uint32_t loop_overrun_count() {
  return s_overrun_count;
}

}  // namespace cp::core

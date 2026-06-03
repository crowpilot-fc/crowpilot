// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/telemetry/SdLogger.h"

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "src/Config.h"
#include "src/actuators/Output.h"
#include "src/control/DesiredState.h"
#include "src/control/Pid.h"
#include "src/core/Loop.h"
#include "src/estimation/Attitude.h"
#include "src/failsafe/Failsafe.h"
#include "src/modes/FlightMode.h"
#include "src/radio/Receiver.h"
#include "src/sensors/Barometer.h"
#include "src/sensors/Imu.h"

namespace cp::telemetry {

namespace {

// 109-byte record per TELEMETRY_FORMAT.md §3. Schema version 0x01.
// static_assert below catches any compiler that adds padding despite
// the pragma pack.
#pragma pack(push, 1)
struct TelemetryRecord {
  uint8_t  schema_version;
  uint64_t t_us;
  uint32_t loop_period_us;
  int16_t  gyro_x, gyro_y, gyro_z;
  int16_t  accel_x, accel_y, accel_z;
  int16_t  imu_temp_raw;
  int32_t  pressure_pa;
  int32_t  baro_temp_cc;
  int32_t  altitude_cm;
  float    quat_w, quat_x, quat_y, quat_z;
  float    roll_fwd_deg, pitch_fwd_deg, yaw_fwd_deg;
  float    roll_hover_deg, pitch_hover_deg, yaw_hover_deg;
  uint16_t rc_ch1_us, rc_ch2_us, rc_ch3_us, rc_ch4_us, rc_ch5_us, rc_ch6_us;
  float    fader;
  uint16_t motor1_us, motor2_us;
  uint16_t servo_left_us, servo_right_us;
  int8_t   pid_roll_q7, pid_pitch_q7, pid_yaw_q7;
  uint8_t  reserved;
  uint8_t  flight_mode;
  uint8_t  status_flags;
};
#pragma pack(pop)

static_assert(sizeof(TelemetryRecord) == 109,
              "TelemetryRecord must be exactly 109 bytes per TELEMETRY_FORMAT.md §3.");

constexpr uint8_t  kSchemaVersion   = 0x01;
constexpr uint16_t kMaxLogIndex     = 9999;

// Status-flag bit positions per TELEMETRY_FORMAT.md §4.3.
constexpr uint8_t kStatusArmed             = 0x01;
constexpr uint8_t kStatusFailsafeActive    = 0x02;
constexpr uint8_t kStatusThrottleCut       = 0x04;
constexpr uint8_t kStatusImuFault          = 0x08;
constexpr uint8_t kStatusBaroFault         = 0x10;
constexpr uint8_t kStatusRxFault           = 0x20;
constexpr uint8_t kStatusImuLossDegraded   = 0x80;

File     s_file;
bool     s_active           = false;
char     s_filename[16]     = {0};
uint32_t s_bytes_written    = 0;
uint32_t s_tick_count       = 0;

// Monotonic wide-time tracking. micros() is uint32_t on the Arduino HAL
// and wraps every ~71 minutes. The log schema declares t_us as uint64.
// On every record we compute the unsigned 32-bit delta from the last
// micros() reading and accumulate it into a 64-bit counter. The 32-bit
// delta is correct across the wrap as long as we sample more often than
// the period (~71 minutes), which the 100 Hz logger trivially does.
uint32_t s_last_micros      = 0;
uint64_t s_wide_us          = 0;

// Scan the SD root for `LOGnnnn.BIN` and return the next free index.
// Returns 1 if no log files exist yet, or kMaxLogIndex + 1 if the
// numbering is exhausted (caller treats that as halt-logging).
uint16_t findNextLogIndex() {
  uint16_t max_seen = 0;
  bool     any_found = false;

  File root = SD.open("/");
  if (!root) {
    return 1;
  }
  while (true) {
    File f = root.openNextFile();
    if (!f) break;
    const char* name = f.name();
    // Match LOGnnnn.BIN with exactly four digits.
    if (strncasecmp(name, "LOG", 3) == 0) {
      const char* d = name + 3;
      if (isdigit(static_cast<unsigned char>(d[0])) &&
          isdigit(static_cast<unsigned char>(d[1])) &&
          isdigit(static_cast<unsigned char>(d[2])) &&
          isdigit(static_cast<unsigned char>(d[3])) &&
          strncasecmp(d + 4, ".BIN", 4) == 0) {
        const uint16_t idx =
            static_cast<uint16_t>((d[0] - '0') * 1000 +
                                  (d[1] - '0') * 100 +
                                  (d[2] - '0') * 10 +
                                  (d[3] - '0'));
        if (idx > max_seen) {
          max_seen = idx;
        }
        any_found = true;
      }
    }
    f.close();
  }
  root.close();

  return any_found ? static_cast<uint16_t>(max_seen + 1)
                   : static_cast<uint16_t>(1);
}

bool openLogFile() {
  const uint16_t idx = findNextLogIndex();
  if (idx > kMaxLogIndex) {
    // Numbering exhausted. Tell the user once and stay inactive.
    if (Serial) {
      Serial.println("ERROR: log file index exhausted. Delete old LOGnnnn.BIN files.");
    }
    return false;
  }
  snprintf(s_filename, sizeof(s_filename), "LOG%04u.BIN", static_cast<unsigned>(idx));
  s_file = SD.open(s_filename, FILE_WRITE);
  if (!s_file) {
    return false;
  }
  s_bytes_written = 0;
  return true;
}

inline int8_t toQ7(float x) {
  if (x >  1.0f) x =  1.0f;
  if (x < -1.0f) x = -1.0f;
  return static_cast<int8_t>(x * 127.0f);
}

inline uint8_t flightModeCode(cp::modes::FlightMode m) {
  switch (m) {
    case cp::modes::FlightMode::HOVER:         return 0;
    case cp::modes::FlightMode::FORWARD:       return 1;
    case cp::modes::FlightMode::TRANSITIONING: return 2;
  }
  return 2;
}

uint8_t computeStatusFlags() {
  uint8_t flags = 0;
  if (cp::actuators::arm_state() == cp::actuators::ArmState::ARMED) {
    flags |= kStatusArmed;
  }
  const auto& fs = cp::failsafe::state();
  if (fs.active) {
    flags |= kStatusFailsafeActive;
  }
  const auto& d = cp::control::desired::current();
  if (d.arm_us > 1500) {
    flags |= kStatusThrottleCut;
  }
  if (!cp::sensors::imu::is_healthy()) {
    flags |= kStatusImuFault;
  }
  if (cp::core::imu_loss_degraded()) {
    flags |= kStatusImuLossDegraded;
  }
  if (cp::sensors::baro::is_present() && !cp::sensors::baro::is_healthy()) {
    flags |= kStatusBaroFault;
  }
  if (fs.rx_frame_lost || fs.link_timeout) {
    flags |= kStatusRxFault;
  }
  return flags;
}

void buildRecord(TelemetryRecord& r) {
  r.schema_version = kSchemaVersion;
  // t_us is uint64 per the schema. micros() is uint32 and wraps every
  // ~71 minutes. Compute the unsigned 32-bit delta from the previous
  // sample and accumulate into a 64-bit counter so the value is
  // monotonic across a wrap. Logger is rate-limited well above the wrap
  // period so the delta is always well below 2^32.
  const uint32_t now_us = micros();
  s_wide_us += static_cast<uint32_t>(now_us - s_last_micros);
  s_last_micros = now_us;
  r.t_us = s_wide_us;
  r.loop_period_us  = cp::core::last_loop_period_us();

  const auto& imu = cp::sensors::imu::latest();
  r.gyro_x       = imu.raw_gx;
  r.gyro_y       = imu.raw_gy;
  r.gyro_z       = imu.raw_gz;
  r.accel_x      = imu.raw_ax;
  r.accel_y      = imu.raw_ay;
  r.accel_z      = imu.raw_az;
  r.imu_temp_raw = imu.raw_temp;

  const auto& baro = cp::sensors::baro::latest();
  r.pressure_pa  = static_cast<int32_t>(baro.pressure_pa);
  r.baro_temp_cc = static_cast<int32_t>(baro.temperature_c * 100.0f);
  r.altitude_cm  = static_cast<int32_t>(baro.altitude_m * 100.0f);

  const auto& q = cp::estimation::attitude::quaternion();
  r.quat_w = q.w;
  r.quat_x = q.x;
  r.quat_y = q.y;
  r.quat_z = q.z;

  const auto fwd = cp::estimation::attitude::eulerForwardFlight();
  r.roll_fwd_deg  = fwd.roll_deg;
  r.pitch_fwd_deg = fwd.pitch_deg;
  r.yaw_fwd_deg   = fwd.yaw_deg;

  const auto hv = cp::estimation::attitude::eulerHover();
  r.roll_hover_deg  = hv.roll_deg;
  r.pitch_hover_deg = hv.pitch_deg;
  r.yaw_hover_deg   = hv.yaw_deg;

  const auto& rx = cp::radio::state();
  r.rc_ch1_us = rx.channel_us[0];
  r.rc_ch2_us = rx.channel_us[1];
  r.rc_ch3_us = rx.channel_us[2];
  r.rc_ch4_us = rx.channel_us[3];
  r.rc_ch5_us = rx.channel_us[4];
  r.rc_ch6_us = rx.channel_us[5];

  r.fader = cp::modes::fader();

  const auto& pulses = cp::actuators::latest_pulses();
  // Motors 0 and 1 are the first two on every airframe (MOTOR_RIGHT and
  // MOTOR_LEFT where those are named).
  r.motor1_us = pulses.motor_us[0];
  r.motor2_us = pulses.motor_us[1];
  if (cp::airframes::N_SERVOS >= 2) {
    r.servo_left_us  = pulses.servo_us[cp::airframes::SERVO_LEFT];
    r.servo_right_us = pulses.servo_us[cp::airframes::SERVO_RIGHT];
  } else {
    // Servo-less airframe (multirotor): log the rear motors in the servo
    // fields so a quad's four motors are all captured. The indices are held
    // in variables so a two-motor airframe's dead branch stays in bounds.
    const uint8_t i2 = (cp::airframes::N_MOTORS > 2) ? 2 : 0;
    const uint8_t i3 = (cp::airframes::N_MOTORS > 3) ? 3 : 0;
    r.servo_left_us  = pulses.motor_us[i2];
    r.servo_right_us = pulses.motor_us[i3];
  }

  const auto& pid = cp::control::pid::output();
  r.pid_roll_q7  = toQ7(pid.roll);
  r.pid_pitch_q7 = toQ7(pid.pitch);
  r.pid_yaw_q7   = toQ7(pid.yaw);

  r.reserved     = 0;
  r.flight_mode  = flightModeCode(cp::modes::mode());
  r.status_flags = computeStatusFlags();
}

}  // anonymous namespace

void init() {
  s_active        = false;
  s_filename[0]   = '\0';
  s_bytes_written = 0;
  s_tick_count    = 0;
  s_last_micros   = micros();
  s_wide_us       = 0;

#if !ENABLE_TELEMETRY_LOG
  return;
#else
  // SD bus init. The Pi Pico Arduino core's `SD.h` configures SPI0
  // internally based on the CS pin argument.
  if (!SD.begin(PIN_SD_CS)) {
    if (Serial) {
      Serial.println("Logger: SD.begin failed. No card or unsupported card.");
    }
    return;
  }
  if (!openLogFile()) {
    if (Serial) {
      Serial.println("Logger: open failed. Inactive.");
    }
    return;
  }
  s_active = true;
#endif
}

void tick() {
#if !ENABLE_TELEMETRY_LOG
  return;
#else
  if (!s_active) {
    return;
  }
  if (++s_tick_count < TELEMETRY_LOG_INTERVAL_TICKS) {
    return;
  }
  s_tick_count = 0;

  TelemetryRecord r;
  buildRecord(r);

  const size_t written =
      s_file.write(reinterpret_cast<const uint8_t*>(&r), sizeof(r));
  if (written != sizeof(r)) {
    if (Serial) {
      Serial.println("Logger: short write. Closing and rotating.");
    }
    s_file.close();
    if (!openLogFile()) {
      s_active = false;
    }
    return;
  }
  s_bytes_written += static_cast<uint32_t>(written);

  if (s_bytes_written >= TELEMETRY_LOG_MAX_BYTES) {
    s_file.close();
    if (!openLogFile()) {
      s_active = false;
    }
  }
#endif
}

bool is_active() {
  return s_active;
}

const char* current_filename() {
  return s_filename;
}

}  // namespace cp::telemetry

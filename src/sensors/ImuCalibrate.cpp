// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/sensors/ImuCalibrate.h"

#include <Arduino.h>

#include "src/Config.h"
#include "src/hal/Hal.h"
#include "src/hal/Led.h"

namespace cp::sensors::imu_calibrate {

namespace {

// One g of gravity, read on the vertical accelerometer axis when the
// airframe is held belly-down and level. It is real signal, not bias,
// and is removed from the z-axis offset estimate.
constexpr float kGravityZ_g = 1.0f;

// Bound on consecutive failed reads before the routine gives up, so a
// dead IMU does not spin forever in silence.
constexpr uint32_t kMaxConsecutiveFailures = 1000;

// Wait up to this long for the USB serial monitor to attach, so the
// printed result is not lost.
constexpr uint32_t kSerialWaitMs = 5000;

// Busy-wait for the requested microseconds. The project standard
// forbids delay() everywhere, init code included, so the calibration
// pacing spins on micros() like the carried chip drivers do.
void busyWaitUs(uint32_t us) {
  const uint32_t t0 = micros();
  while ((micros() - t0) < us) {
    // spin
  }
}

// Halt forever after the calibration prints. The tight spin pattern this
// used to use starved USB CDC polling, so the printed bias values never
// left the FC. Delegate to haltWithFastBlink, which uses delay() so the
// rp2040 core can drain CDC and the calibration result actually reaches
// the host. The repeat message also lets a bench operator who attaches a
// monitor late catch the bias output.
[[noreturn]] void halt() {
  cp::hal::haltWithFastBlink("calibration complete - paste the printed lines into src/Config.h");
}

}  // anonymous namespace

void run() {
  // Brief settle so USB CDC enumeration completes before the first print.
  // Loop.cpp set Serial.ignoreFlowControl(true) earlier, so writes buffer
  // even when no host is attached and drain when one arrives. Do not gate
  // prints on `if (Serial)` here, that drops bytes that the CDC layer
  // could have delivered to a host attaching later.
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < kSerialWaitMs) {
  }

  Serial.println();
  Serial.println("CrowPilot IMU bias calibration.");
  Serial.println("Hold the airframe stationary and level, belly down.");

  if (!cp::hal::imu_init()) {
    Serial.println("IMU init failed. Cannot calibrate.");
    halt();
  }

  float sum_ax = 0.0f;
  float sum_ay = 0.0f;
  float sum_az = 0.0f;
  float sum_gx = 0.0f;
  float sum_gy = 0.0f;
  float sum_gz = 0.0f;

  uint32_t collected = 0;
  uint32_t consecutive_failures = 0;

  while (collected < IMU_CALIB_SAMPLE_COUNT) {
    cp::hal::ImuSample s;
    if (!cp::hal::imu_read(s)) {
      if (++consecutive_failures >= kMaxConsecutiveFailures) {
        Serial.println("IMU read failed repeatedly. Calibration aborted.");
        halt();
      }
      continue;
    }
    consecutive_failures = 0;

    sum_ax += s.ax_g;
    sum_ay += s.ay_g;
    sum_az += s.az_g;
    sum_gx += s.gx_dps;
    sum_gy += s.gy_dps;
    sum_gz += s.gz_dps;
    ++collected;

    busyWaitUs(1000);  // Spread the batch over roughly two seconds.
  }

  const float n = static_cast<float>(IMU_CALIB_SAMPLE_COUNT);

  // The three gyro axes and the two horizontal accelerometer axes read
  // zero at rest, so their mean is the bias directly. The vertical
  // accelerometer axis carries one g of gravity, removed here.
  const float gyro_bias_x  = sum_gx / n;
  const float gyro_bias_y  = sum_gy / n;
  const float gyro_bias_z  = sum_gz / n;
  const float accel_bias_x = sum_ax / n;
  const float accel_bias_y = sum_ay / n;
  const float accel_bias_z = sum_az / n - kGravityZ_g;

  // Print the bias output once per second for 30 seconds so a host that
  // attaches the serial monitor at any time within that window catches
  // the values. Without this repeat, the first print would land in the
  // CDC TX buffer and be lost the moment the panic loop's PANIC lines
  // overwrote the ring buffer. The calibration routine is one-shot
  // bench-only and only runs when ENABLE_IMU_CALIBRATION is set, so this
  // 30 s delay is not on the normal flight boot path.
  for (int rep = 0; rep < 30; ++rep) {
    Serial.println();
    Serial.println("Calibration complete. Paste these into src/Config.h,");
    Serial.println("set ENABLE_IMU_CALIBRATION back to 0, and reflash:");
    Serial.println();
    Serial.print("constexpr float GYRO_BIAS_X = "); Serial.print(gyro_bias_x, 6);  Serial.println("f;");
    Serial.print("constexpr float GYRO_BIAS_Y = "); Serial.print(gyro_bias_y, 6);  Serial.println("f;");
    Serial.print("constexpr float GYRO_BIAS_Z = "); Serial.print(gyro_bias_z, 6);  Serial.println("f;");
    Serial.print("constexpr float ACC_BIAS_X  = "); Serial.print(accel_bias_x, 6); Serial.println("f;");
    Serial.print("constexpr float ACC_BIAS_Y  = "); Serial.print(accel_bias_y, 6); Serial.println("f;");
    Serial.print("constexpr float ACC_BIAS_Z  = "); Serial.print(accel_bias_z, 6); Serial.println("f;");
    Serial.println();
    delay(1000);
  }

  halt();
}

}  // namespace cp::sensors::imu_calibrate

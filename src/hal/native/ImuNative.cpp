// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "hal/Hal.h"

#include "Config.h"

#if BUILD_TARGET == BUILD_TARGET_NATIVE

#include <Arduino.h>

#include "hal/I2c.h"

#if   IMU_TYPE == IMU_MPU6500
  #include "libs/Mpu6500.h"
  namespace imu_chip = cp::libs::mpu6500;
#elif IMU_TYPE == IMU_MPU6050
  #include "libs/Mpu6050.h"
  namespace imu_chip = cp::libs::mpu6050;
#else
  #error "IMU_TYPE must be IMU_MPU6500 or IMU_MPU6050."
#endif

namespace cp::hal {

namespace {

float s_inv_gyro_sens  = 0.0f;
float s_inv_accel_sens = 0.0f;

}  // anonymous namespace

bool imu_init() {
  cp::hal::i2c0::ensureInit();
  if (!imu_chip::init(IMU_I2C_ADDR, IMU_GYRO_RANGE, IMU_ACCEL_RANGE)) {
    return false;
  }
  s_inv_gyro_sens  = 1.0f / imu_chip::gyroSensitivityLsbPerDps();
  s_inv_accel_sens = 1.0f / imu_chip::accelSensitivityLsbPerG();
  return true;
}

bool imu_read(ImuSample& out) {
  imu_chip::RawSample raw;
  if (!imu_chip::readBurst(raw)) {
    return false;
  }
  out.raw_ax   = raw.ax;
  out.raw_ay   = raw.ay;
  out.raw_az   = raw.az;
  out.raw_gx   = raw.gx;
  out.raw_gy   = raw.gy;
  out.raw_gz   = raw.gz;
  out.raw_temp = raw.temp;

  out.ax_g   = static_cast<float>(raw.ax) * s_inv_accel_sens;
  out.ay_g   = static_cast<float>(raw.ay) * s_inv_accel_sens;
  out.az_g   = static_cast<float>(raw.az) * s_inv_accel_sens;
  out.gx_dps = static_cast<float>(raw.gx) * s_inv_gyro_sens;
  out.gy_dps = static_cast<float>(raw.gy) * s_inv_gyro_sens;
  out.gz_dps = static_cast<float>(raw.gz) * s_inv_gyro_sens;
  out.temp_c = imu_chip::tempRawToCelsius(raw.temp);
  return true;
}

}  // namespace cp::hal

#endif  // BUILD_TARGET == BUILD_TARGET_NATIVE

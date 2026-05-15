// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>

// Hardware Abstraction Layer. The split between the firmware logic (every
// sensor/control module under `src/`) and the per-target I/O lives here.
// Two implementations:
//
//   src/hal/native/*.cpp - real hardware. Wire, Servo, PIO, OneShot125
//     bit-bang, the Bosch and InvenSense chip drivers. Used when
//     BUILD_TARGET = BUILD_TARGET_NATIVE.
//
//   src/hal/sim/*.cpp - SCRC over UDP (SITL) or USB CDC (HIL). Populated
//     in Phase 12.7. Used when BUILD_TARGET = BUILD_TARGET_HIL or
//     BUILD_TARGET = BUILD_TARGET_SITL.
//
// The HAL API is transport-agnostic. The firmware logic is identical
// across all three build targets; only the HAL implementation differs.
//
// See SITL_INTEGRATION.md for the broader integration context and
// crowtalk/decisions/ for the locked-in protocol decisions that shape
// the sim side.

namespace cp::hal {

// --- IMU ---------------------------------------------------------------

// One IMU sample. Both raw (int16 register values) and scaled
// (engineering units) forms. Native fills both; sim fills the scaled
// fields and may leave the raw fields zero or reverse-scaled. The
// telemetry logger reads the raw fields, so sim implementations should
// reverse-scale to keep the log format identical across builds.
struct ImuSample {
  float   ax_g;
  float   ay_g;
  float   az_g;
  float   gx_dps;
  float   gy_dps;
  float   gz_dps;
  float   temp_c;
  int16_t raw_ax;
  int16_t raw_ay;
  int16_t raw_az;
  int16_t raw_gx;
  int16_t raw_gy;
  int16_t raw_gz;
  int16_t raw_temp;
};

bool imu_init();
bool imu_read(ImuSample& out);

// --- Barometer ---------------------------------------------------------

struct BaroSample {
  float pressure_pa;
  float temperature_c;
};

// True if a barometer is configured AND its init succeeded. False for
// BARO_NONE builds and for chip-init failures. Sim builds report true.
bool baro_present();

bool baro_init();
bool baro_read(BaroSample& out);

// --- Receiver ----------------------------------------------------------

struct RxState {
  uint16_t channel_us[16];
  bool     ch17;
  bool     ch18;
  bool     channels_valid;
  bool     failsafe_active;
  bool     frame_lost_flag;
  uint32_t lost_frames_count;
  uint32_t last_frame_us;
};

bool rx_init();
void rx_poll(RxState& out);

// --- Actuator output ---------------------------------------------------

// Two-phase emit. set_motor_us buffers per-motor pulse widths. set_servo_us
// applies servo PWM immediately (the underlying Servo library updates the
// hardware duty cycle on its own 20 ms schedule). commit_motors then
// emits the synchronous OneShot125 burst on all motor pins.
void     out_init();
void     out_set_motor_us(uint8_t idx, uint16_t pulse_us);
void     out_set_servo_us(uint8_t idx, uint16_t pulse_us);
void     out_commit_motors();

}  // namespace cp::hal

<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# First bench test

The smoke test before propellers go anywhere near the airframe. Work through these steps in order. Do not skip ahead.

> **Read [SAFETY.md](../../SAFETY.md) and [DISCLAIMER.md](../../DISCLAIMER.md) in full before any motor spins. Propellers can cause serious injury or death. This bench procedure is the minimum, not the maximum.**

## Stage 0. Before any power-on

- Propellers REMOVED from the motors.
- ESC battery DISCONNECTED.
- FC and airframe on a stable bench. No clutter within propeller-strike radius (even though props are off).
- Transmitter ON, bound to the receiver, **ch5 HIGH (throttle cut), throttle stick at minimum**.
- Multimeter or oscilloscope available.
- Fire extinguisher rated for electrical and lithium fires within reach.

## Stage 1. USB-only power

Connect the FC to USB only. No ESCs, no battery.

1. Open the Arduino IDE Serial Monitor at 115200 baud.
2. Expected boot output:
   ```
   CrowPilot boot. LOOP_HZ=1000, period_us=1000
   IMU OK
   Receiver OK (PIO SM 0 active)
   Barometer OK
   Actuators OK (NOT_ARMED)
   Logger OK -> LOG0001.BIN
   ```
3. The 1 Hz `loop_period_us avg=1000 max=...` line should appear repeatedly.
4. The 200 ms `DEV ...` line should appear, with `imu=OK`, `fs=1/to=1/rxfs=0` (failsafe active since no transmitter context), `armed=0`, `pid=(0.00,0.00,0.00)`, `mix=[m0.30,0.30|s0.50,0.50]`, `baro=OK`, `lost=0`.
5. The onboard LED on GP25 should blink at 1 Hz heartbeat.

**Red flags.**

- `ERROR: IMU init failed`. Check I2C wiring (GP4 SDA, GP5 SCL), sensor power, and `IMU_I2C_ADDR` in `Config.h`.
- `ERROR: Receiver init failed`. PIO program could not load; rebuild and reflash. If the error persists, file an issue.
- `WARN: Barometer init failed`. Non-fatal. Check baro wiring or set `BARO_TYPE = BARO_NONE`.
- Fast LED blink (~5 Hz) and no further serial output. Fatal init failure; check the last serial line for the cause.

## Stage 2. Sensor sanity

With the FC level and stationary on the bench:

1. Set `DEBUG_PRINT_DEV = 0` and `DEBUG_PRINT_IMU = 1` in `Config.h`. Reflash.
2. Expected output: 10 Hz lines like `imu a=(+0.01, -0.02, +1.00)g g=(+0.10, -0.50, +0.30)dps T=24.5C`.
3. Static accel reads close to `(0, 0, +1.00) g` for a level FC.
4. Static gyro reads close to `(0, 0, 0) dps`, within a couple of dps.
5. Temperature reads within a few degrees of room temperature.
6. Tilt the FC about each axis. The corresponding accel and gyro values respond sensibly.

If static gyro reads farther than a few dps from zero, run the IMU bias calibration:

1. Set `ENABLE_IMU_CALIBRATION = 1`. Reflash.
2. Keep the FC level and stationary for ~12 seconds.
3. Copy the six `constexpr float ..._BIAS_* = ...f;` lines from the serial monitor into `src/Config.h`.
4. Set `ENABLE_IMU_CALIBRATION = 0`. Reflash.
5. Repeat Stage 2 step 1 to 6 above. Gyro values should now sit much closer to zero.

## Stage 3. Receiver mapping

With the transmitter ON, ch5 HIGH, throttle stick at minimum:

1. Set `DEBUG_PRINT_IMU = 0` and `DEBUG_PRINT_RX = 1` in `Config.h`. Reflash.
2. Expected output: 10 Hz lines like `rx ch=[1000, 1500, 1500, 1500, 2000, 1500] valid=1 fs=0 fl=0 lost=0`.
3. Move each stick and switch in turn:
   - Throttle stick. ch1 should sweep 1000 to 2000.
   - Roll stick. ch2 should sweep 1000 to 2000.
   - Pitch stick. ch3.
   - Yaw stick. ch4.
   - Arm/cut switch. ch5 should toggle 1000 (LOW) to 2000 (HIGH).
   - Transition switch. ch6.
4. Verify channel directions match your TX configuration. If a stick reads inverted, fix it on the TX side (preferred) or in `Config.h`.
5. Confirm centered sticks read 1500 microseconds, plus or minus a few.

## Stage 4. Failsafe verification

With the transmitter on and channels reading correctly:

1. Set `DEBUG_PRINT_RX = 0` and `DEBUG_PRINT_FAILSAFE = 1`. Reflash.
2. Expected output: `fs active=0 link_to=0 oor=0 rx_fs=0 rx_fl=0 eff=[1000, 1500, 1500, 1500, 2000, 1500]`.
3. Power off the transmitter. Within ~200 ms the print should switch to `fs active=1 link_to=1 ... rx_fs=1` and effective channels should snap to the failsafe defaults `eff=[1300, 1500, 1500, 1500, 1000, 2000]`.
4. Power the transmitter back on. Within a second `fs active=0` returns.

Failsafe behavior in flight is "controlled fall": throttle pinned to ~30 percent, sticks centered, fader pinned to hover. The FC does NOT cut throttle to zero on link loss; that would slam motors off and drop the aircraft.

## Stage 5. Attitude estimate

With the FC level and stationary:

1. Set `DEBUG_PRINT_FAILSAFE = 0` and `DEBUG_PRINT_ATTITUDE = 1`. Reflash.
2. Expected output: `rpy=(+0.5, -1.2, +0.3)deg q=(+0.9999, +0.0044, ...)`.
3. RPY values converge near zero within 1 to 2 seconds (the Madgwick filter takes a moment to pull the quaternion onto gravity).
4. Tilt the FC about each axis. Roll, pitch, yaw should track the motion.
5. Yaw drifts slowly when stationary. This is expected; v1 has no magnetometer fusion.
6. Quaternion magnitude (sqrt of squared sum of q0..q3) should stay at 1.0 ± 0.0001.

## Stage 6. Fader and mode

With the transmitter on:

1. Set `DEBUG_PRINT_ATTITUDE = 0` and `DEBUG_PRINT_MODE = 1`. Reflash.
2. With ch6 in the hover position (toward 1900 microseconds), expect `mode fader=1.00 mode=hover ch6=1900`.
3. Flip ch6 to forward (toward 1100 microseconds). The fader slews to 0.00 over 3 seconds: `fader=0.50 mode=transitioning ch6=1100` at the halfway point, then `fader=0.00 mode=forward ch6=1100`.
4. Flip back to hover. Slews 0.00 to 1.00 over 3 seconds.
5. Mid-stick ch6 (around 1500 microseconds) settles at fader=0.50 and mode=transitioning.
6. Power off the transmitter. ch6 pins to 2000 (via failsafe), fader returns to 1.00 hover.

## Stage 7. Bench mixer check (motors OFF)

This stage runs with motors connected to ESCs but NO PROPELLERS. ESC battery still disconnected.

1. Set `DEBUG_PRINT_MODE = 0` and `DEBUG_PRINT_MIXER = 1`. Reflash.
2. Power the FC by USB. NOT_ARMED state. Mixer output should show `mix m=[~throttle, ~throttle] s=[~0.50, ~0.50]` with throttle and servos at safe defaults.
3. Power transmitter on, ch5 LOW (arm switch enabled), throttle stick at minimum. The FC should auto-arm.
4. Verify mixer responses to stick deflections in hover and forward modes. No motor or servo movement at this stage (no battery).

## Stage 8. ESC connected (no motors yet)

Now connect ESC battery. Motors stay disconnected from the ESC.

1. With ch5 HIGH (throttle cut), verify ESC stays silent (no arm tone).
2. Flick ch5 LOW with throttle at minimum. ESC should beep its arm tone.
3. Push throttle stick up slowly. ESC follows the pulse width internally (no motor connected, so nothing spins).
4. Flick ch5 HIGH. ESC returns to silent.
5. Disconnect ESC battery.

## Stage 9. Motors connected, propellers OFF

Now connect motors to ESCs. **Propellers stay OFF.**

1. Tether the airframe so it cannot tip over or jump.
2. Power FC, then ESC battery.
3. Verify NOT_ARMED state. Motor pulses on the scope show ~120 microseconds (below valid range).
4. Arm: ch5 LOW, throttle stick at minimum. Motors should idle (visible spin or twitch at idle pulse).
5. Throttle up slowly. Verify motor direction matches the airframe geometry (right motor clockwise, left counterclockwise when viewed from behind, for the Eclipson reference build).
6. Verify each stick affects the correct motor differential or servo deflection in both hover and forward modes per [docs/user-guide/tuning.md](../user-guide/tuning.md).
7. ch5 HIGH immediately disarms. Motor pulses snap back to ~120 microseconds.
8. Power off TX during ARMED+low-throttle. Failsafe activates within ~200 ms. Motors hold at the failsafe-throttle value (~30 percent), not zero.

## Stage 10. Propellers on, tethered

Crosses into v1 first-flight territory. See [SAFETY.md](../../SAFETY.md) "First motor spin with propellers installed" for the procedure. This stage is **out of scope for the bench-verification queue**; it is acceptance flight testing.

## When to escalate

If any stage fails and the cause is not in the red-flag list:

1. Pull the SD card and decode the latest log via `python tools/decode_log.py LOG0001.BIN > log0001.csv`.
2. Inspect the CSV for the anomalous tick.
3. Open a GitHub Issue with the relevant excerpt of the CSV and a description of the bench setup.

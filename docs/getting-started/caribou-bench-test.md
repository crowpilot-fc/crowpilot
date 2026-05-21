<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Caribou bench test

The pre-flight bench procedure for the DHC-4 Caribou twin-engine plane.
Follow it in order. Each phase ends with a pass/fail bar. A failure means
**stop and fix**, not work around. The whole sequence runs with **propellers
off** until phase 11.

The reference wiring for every step is [`docs/reference/caribou-wiring.svg`](../reference/caribou-wiring.svg).
The board pin map is [`docs/reference/caribou-pinmap.svg`](../reference/caribou-pinmap.svg).

Read [Safety](../safety.md) before applying any power.

---

## 0. What you need on the bench

- The wired airframe or breadboard equivalent (WeAct RP2350A, ESP32-C3
  companion, MPU6500, BMP388, microSD, SBUS receiver, and all the servos
  the airframe carries).
- A 6S LiPo. For early phases, USB power alone is enough.
- A multimeter.
- The transmitter, with channels configured (see phase 2).
- A laptop running a serial monitor at 115200 baud, plus a phone for
  the wireless companion.
- An SD card formatted FAT32, inserted in the FC.
- A solid bench restraint or tether the aircraft cannot escape from in
  phase 11.

---

## 1. Pre-power inspection

Eyes only. No power yet.

- [ ] Every connector matches the wiring diagram. Spot-check the MPX
  pinout on both wings, the 3-pin headers on the body, the XT60s, and
  the ground tap from each XT60 to the common ground bus.
- [ ] No reversed I2C pair, no swapped ESC signal, no servo wired with
  +5V on the signal pin.
- [ ] Battery is **disconnected**.
- [ ] Propellers are **off both motors** and out of reach.

**Pass:** no obvious miswire, battery out, props off.

---

## 2. Transmitter channel mapping

The firmware reads RC channels by index. Set up the TX so each channel
carries the value the firmware expects.

Firmware defaults in [`Config.h`](https://github.com/crowpilot-fc/crowpilot/blob/main/src/Config.h):

| Channel | Constant | Used by |
|---|---|---|
| 1 | `CHANNEL_THROTTLE` | Plane stabilizer / mixer |
| 2 | `CHANNEL_ROLL` | Plane stabilizer / mixer |
| 3 | `CHANNEL_PITCH` | Plane stabilizer / mixer |
| 4 | `CHANNEL_YAW` | Plane stabilizer / mixer |
| 5 | `CHANNEL_ARM` | Arm switch |
| 6 | `CHANNEL_TRANSITION` | Tailsitter fader (unused on a plane) |
| 7 | `CHANNEL_STAB` | Passthrough (HIGH) vs stabilized (LOW) |
| 8 | `CHANNEL_ALT_HOLD` | Barometric altitude hold |

Aux channels are read by `user-sketch.ino`; assign them to TX switches
for the landing gear, flaps, bay doors, nose-wheel steering, and the
LED2 nav-blink master.

**Important if your TX is in AETR mode** (channels 1-4 = Aileron,
Elevator, Throttle, Rudder), the firmware's TAER defaults will see your
throttle on channel 3 and your roll stick on channel 1. Either:

- Set the TX output map to TAER (throttle on channel 1), or
- Edit the `CHANNEL_*` defines in `Config.h` to match your TX (roll on
  1, pitch on 2, throttle on 3, yaw on 4) and reflash.

Do this before any further phase. The whole procedure assumes the
firmware's expectation of "channel N carries function X" lines up with
the TX.

**Pass:** the TX is mapped so the firmware sees the right value on the
right channel, and the spare TX switches are assigned to ARM, STAB,
ALT_HOLD, gear, flap, bay door, and steering.

---

## 3. USB-only smoke test

- [ ] Connect the FC over USB. Open the serial monitor at 115200 baud.
- [ ] Confirm the boot banner: `CrowPilot boot. LOOP_HZ=1000, period_us=1000`.
- [ ] Confirm each module reports OK:
  - `IMU OK`
  - `Receiver OK (PIO SM 0 active)`
  - `Barometer OK` (or `disabled (BARO_NONE)` if intentional)
  - `Actuators OK (NOT_ARMED)`
  - `Logger OK -> LOG0001.BIN` (if the SD card is in)
- [ ] Verify the onboard LED (GP25) is blinking at 1 Hz (boot heartbeat).
- [ ] Verify the external status LED on GP14 lights.

If any module reports failure or the boot halts with a fast LED blink
(panic), stop and debug. Most failures here are wiring (I2C swap, SD
card not seated) or wrong board profile (`BOARD` in `Config.h`).

**Pass:** clean boot, every module OK, the FC stays running.

---

## 4. Receiver and channel verification

Still USB-only. Power on the transmitter and bind the receiver.

- [ ] In `Config.h`, set `DEBUG_PRINT_RX 1` and reflash, **or** issue
  `cp stream on` over the serial monitor to receive `cp tlm` lines.
- [ ] Move each stick and switch in turn. Confirm only the expected
  channel value changes, and that it sweeps the full RC range
  (1000 µs to 2000 µs, centered at 1500 µs).
- [ ] Throttle stick at idle reads close to `RC_MIN_US` (1000 µs).
- [ ] Throttle stick at full reads close to `RC_MAX_US` (2000 µs).
- [ ] Arm switch in the disarm position reads above the channel
  midpoint; in the arm position reads below.

**Pass:** every TX input maps to the expected channel value, full range,
no twitching when the sticks are still.

---

## 5. Battery power-on

Disconnect USB. With the propellers still off:

- [ ] Plug both XT60s into the battery. Confirm both ESCs beep their
  startup sequence.
- [ ] Verify with the multimeter:
  - **5 V on Rail A** (the body bus from ESC-A, on the right MPX).
  - **5 V on Rail B** (the nose bus from ESC-B, on the left MPX).
  - **Common ground** is continuous: probe between any two ground
    points (XT60 negative, MCU GND, a servo GND). Reading should be
    near 0 Ω.
- [ ] The FC comes up under battery power (LEDs blink, ESP boots).
- [ ] No ESC beeps repeatedly: that means it is not seeing a valid
  throttle signal. Recheck wiring and channel mapping.

**Pass:** both 5 V rails present, common ground continuous, FC + ESP
boot cleanly under battery power.

---

## 6. Wireless companion

With the battery on:

- [ ] Phone WiFi: confirm `CrowPilot-XXXX` access point appears, where
  `XXXX` is the last 4 hex of the ESP MAC.
- [ ] Join the AP using the password set in `esp-companion.ino`
  (default `crowpilot`).
- [ ] Open `http://192.168.4.1`. The page should load with the
  CrowPilot UI, status pill turning to **connected**.
- [ ] Telemetry strip updates live (roll/pitch/yaw move when you tilt
  the airframe).
- [ ] Parameter list populates; move one slider and confirm the new
  value sticks (the slider value matches after release).

If the page loads but parameters never arrive, the UART link to the FC
is wrong (TX/RX swapped, common-ground missing, or wrong baud). If the
AP itself does not appear, the ESP is not powered or not booting.

**Pass:** phone connects, web UI loads, params and telemetry round-trip.

---

## 7. Surface direction check  (THE critical phase)

**The single most common maiden-crash cause is a reversed control
surface.** Spend time here.

Set `CHANNEL_STAB` HIGH on the TX (passthrough). With the airframe held
level and the battery on:

- [ ] Roll stick **right** → right aileron goes **up**, left aileron
  goes **down** (the surfaces command a right roll).
- [ ] Pitch stick **back** → elevators go **up** (the surfaces command
  a pitch up).
- [ ] Yaw stick **right** → rudder goes **right** (commands a right yaw).

If any direction is wrong: **reverse the channel on the TX**, not in
`Config.h`. Then re-test the entire phase.

Now set `CHANNEL_STAB` LOW (stabilized). Hold the airframe level, then
tilt it:

- [ ] Tilt the **right wing down** by hand. The surfaces should command
  to **roll left** (right aileron down, left aileron up): the
  stabilizer is opposing the disturbance.
- [ ] Tilt the **nose up**. The elevators should command **nose down**.
- [ ] Yaw the airframe by hand. The rudder should oppose the yaw.

A surface that moves *with* the tilt instead of against it is a sign
flip in the stabilizer for that axis. Disarm everything, fix in
`Config.h` (`STAB_OUTPUT_SCALE` per-axis sign, or the relevant gain),
and re-test.

**Pass:** every surface moves the right way in both passthrough and
stabilized, on every axis.

---

## 8. Throttle range and ESC arming

Propellers **still off**.

- [ ] Throttle stick at idle. Both ESCs sit silent, motors not
  spinning. The disarm pulse (`ESC_DISARM_PULSE_US`, 1000 µs in PWM
  mode) is below the ESC's throttle-arm threshold.
- [ ] Move the arm switch and follow phase 9 to actually arm.
- [ ] Once armed, raise the throttle slowly. Both motors spin up
  smoothly and symmetrically. Confirm rotation direction by eye and
  by the airflow tape on a single blade.
- [ ] Throttle to roughly mid: both motors hold a steady RPM.
- [ ] Throttle to full briefly: both motors run cleanly.
- [ ] Disarm. Motors stop within one tick.

If a motor turns the wrong way, swap any two of the three ESC-to-motor
phases. If one motor spins notably slower, that ESC may be on a
different setting (low-voltage cutoff, throttle calibration). Address
ESC-by-ESC.

**Pass:** both motors spin up symmetrically across the full throttle
range. Disarm stops them.

---

## 9. Arm sequence

Per the safety logic in `Output.cpp`:

- [ ] Power-cycle the board with the **arm switch already in the arm
  position**. Try to arm by lowering the throttle. The motors must
  **not** arm: the new arm-edge guard refuses to arm until the switch
  has been seen disarmed at least once since boot.
- [ ] Move the arm switch to the disarm position (no motor change).
- [ ] Move the arm switch to the arm position with the throttle stick
  at idle. Motors arm; throttle now affects them.
- [ ] Move the arm switch to disarm at any time. Motors stop within
  one tick.
- [ ] Try to arm with the throttle above idle (`ARM_THROTTLE_MAX_US`,
  1050 µs default). The motors must **not** arm.

**Pass:** the arm-edge guard works, throttle gate works, disarm is
unconditional.

---

## 10. Failsafe drill

Propellers **off**. Arm the FC, throttle at idle.

- [ ] Power off the transmitter. Within 100 ms (`FS_LINK_TIMEOUT_US`):
  - Motor pulses go to `FS_CH1_THROTTLE` (1300 µs by default, a low
    descending throttle).
  - Surfaces drive to the failsafe defaults (centered sticks).
  - `cp tlm` reports `fs=1` if streaming.
- [ ] Power the transmitter back on. Failsafe should clear within
  a frame or two, the FC returns to normal pilot inputs.
- [ ] With the TX on, deliberately push a channel out of the SBUS
  valid range (some TX setups can do this). The out-of-range check
  in `Failsafe.cpp` (`[880, 2159]`) should also trip failsafe.

**Pass:** TX off trips failsafe in under 100 ms, TX back on clears it,
out-of-range channels trip failsafe.

---

## 11. Auxiliary channels  (user-sketch)

With the user-sketch built and `ENABLE_USER_HOOK 1`:

- [ ] **Landing gear** switch toggles the retracts. If the sequenced
  retract logic is on, the nose leg moves first, then the wing legs
  with a delay.
- [ ] **Flap** channel moves the flap servos to up / half / down.
  Both flap1 servos (and both flap2 servos) move together.
- [ ] **Bay door** channel opens and closes both bay door servos.
- [ ] **Nose-wheel steering** stick moves the nose wheel left and
  right (or follows the rudder, depending on the sketch's logic).
- [ ] **LED2 nav blink** is blinking on both wings (red on the left
  wing, green on the right wing, blinking in sync if the GPIO is
  shared).
- [ ] **LED1 white** is steady on both wings whenever the battery is
  on.

If a servo binds or stalls, immediately reduce its travel in the
user-sketch or mechanically. A stalled servo on Rail A pulls amps from
the 5 A BEC that should be feeding the rest of the body.

**Pass:** every aux actuator moves correctly, no stall, no current sag.

---

## 12. Stabilizer behaviour test

Arm, hold the airframe level, throttle just enough to get past the
ESC's idle. Then:

- [ ] With `CHANNEL_STAB` LOW (stabilized) and sticks centered, give
  the airframe a sharp roll or pitch disturbance by hand. The
  surfaces should oppose it and the disturbance should damp out in
  one to two oscillation periods.
- [ ] Persistent oscillation on an axis means the D gain is too low or
  the P gain is too high for that axis.
- [ ] Sluggish recovery means the P gain is too low.
- [ ] Switch to passthrough (`CHANNEL_STAB` HIGH). The same disturbance
  should *not* damp: it should keep rotating because the FC is now
  only relaying stick inputs.

Capture the difference between the two modes in a log (next phase)
before tuning further.

**Pass:** stabilized recovers cleanly from a hand disturbance,
passthrough does not interfere with manual control.

---

## 13. Telemetry log

- [ ] Arm, run a 30-second bench session moving sticks across their
  full range.
- [ ] Disarm. Pull the SD card.
- [ ] Decode the log on the host:

  ```bash
  python tools/decode_log.py LOG0001.BIN > log0001.csv
  ```

- [ ] Confirm: `loop_period_us` stays near 1000, no `LOG_OVERFLOW`
  flag, attitude tracks the airframe's actual motion, throttle and
  arm flags transition cleanly.
- [ ] If the `overruns` counter on the DEV line is nonzero and
  growing, the loop is taking too long: investigate (SD write
  latency, debug print overhead).

**Pass:** the log is complete, clean, and tells the story of the
bench session.

---

## 14. Final pre-flight checks

The whole table from above, but condensed into a single go/no-go gate.
Run this in the last minute before powering up at the field.

- [ ] Battery charged. Voltage above the per-cell threshold.
- [ ] Propellers tight and balanced.
- [ ] All control linkages tight. No play, no binding through full
  travel.
- [ ] Centre of mass within the airframe's design envelope. Re-trim
  if anything has moved since the last flight.
- [ ] Arm sequence verified at the field (you may need to power-cycle
  the FC if the switch position changed since the bench).
- [ ] Failsafe trips when the TX is turned off and recovers when it
  is turned back on, at the field, with the actual receiver
  placement.
- [ ] Stabilizer is on (`CHANNEL_STAB` LOW) for the first flight, not
  passthrough.
- [ ] SD card is in and the logger reported OK at boot.
- [ ] You have a recovery plan if the failsafe descent ends somewhere
  awkward.

If any item is no-go, **do not fly**. Fix on the bench and re-run from
the relevant phase.

---

## Red flags that mean stop, not work around

- Motors do not stop instantly on disarm.
- Any surface moves *with* a hand disturbance instead of against it
  in stabilized mode.
- Loop period averages above 1100 µs or the overrun counter grows.
- Failsafe does not engage within 100 ms of the TX powering off.
- Either 5 V rail sags below ~4.7 V under bench load.
- The ESP companion shows the wrong parameter values vs the serial
  monitor.
- The IMU goes unhealthy under a small bench disturbance.

These are not tuning items. They mean a wiring, code, or hardware
problem that has to be fixed before flight.

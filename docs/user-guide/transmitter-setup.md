<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Transmitter setup

CrowPilot reads SBUS. The four flight controls sit on the AETR primaries, the arm and stabilizer switches sit on high channels clear of the primaries, and the rest of the Caribou functions are aux channels handled by the [user sketch](user-sketch.md). This page covers how to map them on your transmitter. Verify every channel on the bench per [caribou-bench-test.md](../getting-started/caribou-bench-test.md) before flying.

## Channel mapping

The reference layout matches the DHC-4 Caribou first-flight aircraft. Map your transmitter to match the channel that each `CHANNEL_*` constant in `Config.h` reads.

### Flight controls (firmware mixer)

| Channel | Function | Control | Range |
|---|---|---|---|
| ch1 | Roll | Aileron stick | 1000 to 2000 us, centered 1500 |
| ch2 | Pitch | Elevator stick | 1000 to 2000 us, centered 1500 |
| ch3 | Throttle | Throttle stick | 1000 (min) to 2000 (max) us |
| ch4 | Yaw | Rudder stick | 1000 to 2000 us, centered 1500 |

This is the standard AETR order (Aileron, Elevator, Throttle, Rudder).

### Mode and arm switches (firmware)

| Channel | Function | Control | Range |
|---|---|---|---|
| ch7 | Flight mode | Two or three position switch | LOW angle (self-level), MID horizon, HIGH manual passthrough |
| ch8 | Arm / cut | Two-position switch | 1000 (LOW, armed) / 2000 (HIGH, disarmed) |

The arm switch on ch8 is mandatory. The flight-mode switch on ch7 picks the stabilizer mode by switch position: LOW is angle (self-leveling), the middle is horizon (self-levels near center stick, full control at full stick), and HIGH is full manual passthrough. A two-position switch reaches only angle and manual, which matches the older stabilized-or-manual behavior. The default position-to-mode mapping is in `Config.h` (`PLANE_MODE_SW_LOW/MID/HIGH`) and can be remapped, for example to put rate mode in the middle. These two channels used to sit at ch14 and ch16, above every Caribou function, so that they could not collide with a primary or aux assignment. That only works on SBUS and CRSF, which always carry 16 channels. A PWM receiver has exactly as many channels as it has signal wires, and any role above that count reads a parked 1500 us forever, which pins the flight mode with no way to reach manual in the air. They now sit at ch7 and ch8, inside an 8-channel budget. You must still assign both explicitly on the transmitter, and on a PWM build both wires must physically reach the flight controller. A `static_assert` in `Config.h` refuses to build if either role lands above the receiver's channel count.

`CHANNEL_ALT_HOLD` (ch13) and the live-tune knobs `LIVE_TUNE_CH_KP` (ch11) and `LIVE_TUNE_CH_KD` (ch12) are optional. Altitude hold is off by default (`ENABLE_ALT_HOLD = 0`). Leave these channels unassigned to skip them.

### Caribou aux (user sketch)

These channels are read by `user-sketch.ino`, not the firmware mixer.

| Channel | Function |
|---|---|
| ch5 | Throttle 2 (second engine) |
| ch6 | Aileron 2 (second aileron) |
| ch7 | Landing gear, both retracts |
| ch8 | Flap 1, both wings tied |
| ch9 | Bay door, both doors on one channel |
| ch10 | Flap 2, both wings tied |

Flap 1 (ch8) and flap 2 (ch10) are independent. To work them together, assign one flap switch to both channels on the transmitter.

## Throws, deadband, expo

- **Endpoints.** Set channel endpoints so the stick travels the full 1000 to 2000 us range. A stick that only reaches 1100 to 1900 costs you control authority.
- **Centering.** Roll, pitch, and yaw must read 1500 us, plus or minus a few, with the sticks centered. Trim them on the transmitter.
- **Deadband.** A small deadband on roll, pitch, and yaw is fine and reduces hand-jitter. The FC does not add its own deadband.
- **Expo.** Expo is a pilot preference. Start with none, add it later if the aircraft feels twitchy around center. Expo is applied on the transmitter, not in `Config.h`.

## Switch assignments

- **ch8 (arm/cut)** goes to a two-position switch. Pick one with a physical guard if your transmitter has one, or at least a switch you will not knock by accident. HIGH must mean disarmed.
- **ch7 (flight mode)** goes to a two or three position switch. LOW is angle (self-level), the middle is horizon, HIGH is full manual passthrough. A two-position switch reaches angle and manual. Fly the maiden in angle (LOW).

## Channel direction

If a stick or switch reads inverted on the bench (throttle up reads 1000 instead of 2000, ch8 HIGH reads armed instead of disarmed), fix it on the transmitter by reversing the channel. Do not invert it in `Config.h`. Keeping all inversion on the transmitter side keeps one source of truth.

## Verifying the setup

Set `DEBUG_PRINT_RX = 1` in `Config.h`, reflash, and open the serial monitor. Move each stick and switch in turn and confirm the channel values move as the table above describes. The browser configurator and the phone companion both show live channel values too. Full procedure in [caribou-bench-test.md](../getting-started/caribou-bench-test.md).

## Transmitter-specific notes

CrowPilot reads SBUS or CRSF, selected by `RX_PROTOCOL` in `Config.h`. RadioMaster, Jumper, FrSky, and Flysky transmitters paired with an SBUS or ELRS receiver all work. ELRS and Crossfire receivers output CRSF natively, so set `RX_PROTOCOL = RX_CRSF` and wire the receiver to the GP1 input. An ELRS receiver set to SBUS output also works, but CRSF is the better fit and is the path that later gains a telemetry uplink.

Transmitter-by-transmitter walkthroughs are a good contribution target. If you set up a specific model, consider opening a pull request adding it here.

<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Flashing CrowPilot

CrowPilot is an Arduino sketch. Flash it via Arduino IDE 2.x with the Earle Philhower Pi Pico Arduino core.

## Prerequisites

- Arduino IDE 2.x. Download from [arduino.cc/en/software](https://www.arduino.cc/en/software).
- The Raspberry Pi Pico Arduino core by Earle F. Philhower. Install via the Arduino IDE Boards Manager.
- A Waveshare RP2350-Tiny (or any supported alternate; see [docs/reference/pin-maps.md](../reference/pin-maps.md)).
- A USB-C cable.

## One-time setup

1. Open Arduino IDE 2.x.
2. Open **File → Preferences**.
3. In **Additional boards manager URLs** add `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`.
4. Click OK.
5. Open **Tools → Board → Boards Manager**.
6. Search for "Raspberry Pi Pico". Install **Raspberry Pi Pico/RP2040/RP2350 by Earle F. Philhower III**. Version 4.x or newer is required for RP2350 support.

## Clone the repository

```bash
git clone https://github.com/crowpilot-fc/crowpilot.git
cd crowpilot
```

Open `crowpilot.ino` in the Arduino IDE. The IDE will recognize the directory as a sketch.

## Configure your board

Edit `src/Config.h` and set `BOARD` to match your hardware:

```cpp
#define BOARD  BOARD_WAVESHARE_RP2350_TINY
// or
#define BOARD  BOARD_WEACT_RP2350A_V10
```

You can also adjust `IMU_TYPE` (MPU-6500 or MPU-6050) and `BARO_TYPE` (BMP388, BMP280, or BARO_NONE). `LOOP_HZ` is 1000 for v1 and should be left there: the IMU drivers configure the sensor for a 1 kHz internal sample rate to match it. The defaults (MPU-6500 preferred, BMP388 default) match the v1 reference build.

## Select the right board entry

In Arduino IDE: **Tools → Board → Raspberry Pi RP2040 Boards → Raspberry Pi Pico 2** (or **Generic RP2350** if your board is not a Pi Pico 2).

Confirm CPU speed: **Tools → CPU Speed → 150 MHz**. This matches the RP2350's stock clock rate.

## Flash the firmware

1. Hold the BOOTSEL button on the Waveshare RP2350-Tiny's adapter board. (Other RP2350 boards have a BOOTSEL button on the main board itself.)
2. Connect the USB-C cable to the FC and to your computer. Release BOOTSEL.
3. The board appears as a USB drive.
4. In Arduino IDE, click **Upload** (the right-arrow button).
5. The IDE compiles the sketch and flashes it. On success the board reboots and disconnects as a USB drive.
6. The FC starts running the firmware immediately.

## First serial output

Open **Tools → Serial Monitor** at 115200 baud. Within a second of power-on you should see:

```
CrowPilot boot. LOOP_HZ=1000, period_us=1000
IMU OK
Receiver OK (PIO SM 0 active)
Barometer OK
Actuators OK (NOT_ARMED)
Logger OK -> LOG0001.BIN
```

If `Logger inactive (no SD card, init failed, or disabled)` appears instead of `Logger OK`, either no SD card is inserted, the card is not FAT32, or `ENABLE_TELEMETRY_LOG = 0` in `Config.h`. Logging is optional; the FC flies fine without it.

If `Barometer disabled (BARO_NONE)` appears, that is correct for builds with `BARO_TYPE = BARO_NONE` and no chip wired.

After the boot banner the firmware emits a periodic development debug line (`DEBUG_PRINT_DEV = 1` by default) every 200 ms:

```
DEV imu=OK rpy=(+0.1,-0.2,+0.0) fs=1/to=1/rxfs=0 ch=[1300,1500,...] fader=1.00 mode=hover armed=0 pid=(+0.00,+0.00,+0.00) mix=[m0.30,0.30|s0.50,0.50] baro=OK alt=0.0 lost=0 tlm=LOG0001.BIN
```

Plus a 1 Hz loop period report:

```
loop_period_us avg=1000 max=1006 (over 1000 iter)
```

## What if it does not boot

- The onboard LED on GP25 blinking fast (~5 Hz) without any serial output means a fatal init failure. Check IMU and barometer wiring.
- No serial output at all and no LED activity means either the board is in BOOTSEL mode still (disconnect, reset, reconnect) or the flash did not take.
- "ERROR: IMU init failed" means the IMU is not responding on I2C. Check SDA/SCL pin assignments, sensor power, and the `IMU_I2C_ADDR` setting.
- "WARN: Barometer init failed" is non-fatal. The FC continues without altitude. Check baro wiring or set `BARO_TYPE = BARO_NONE` if you do not want a baro.

For deeper troubleshooting see [docs/getting-started/first-bench-test.md](first-bench-test.md).

## After first boot

Before powering motors:

1. Run through [docs/getting-started/first-bench-test.md](first-bench-test.md).
2. Calibrate IMU bias by setting `ENABLE_IMU_CALIBRATION = 1`, reflashing, and following the serial-monitor prompts.
3. Map your transmitter channels per [docs/user-guide/tuning.md](../user-guide/tuning.md).
4. Verify failsafe behavior by toggling the transmitter off mid-bench.

Only after all bench tests pass should you connect motors or propellers.

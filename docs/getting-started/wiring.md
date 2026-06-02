<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Wiring

Pin assignments and wiring for CrowPilot. The default first-flight build is the DHC-4 Caribou on a WeAct RP2350A_V10, shown first. The Waveshare RP2350-Tiny is the smaller alternate, shown after it. The full per-board pin reference is [docs/reference/pin-maps.md](../reference/pin-maps.md), the annotated Caribou pin diagram is [docs/reference/caribou-pinmap.svg](../reference/caribou-pinmap.svg), and a prototype board layout (module placement plus the MPX and wing connectors) is [docs/reference/caribou-board-layout.svg](../reference/caribou-board-layout.svg).

## Pin map (WeAct RP2350A_V10, DHC-4 Caribou)

This is the default build. The four control-surface servos are on GP6 to GP9. The I2C, SD, ESC, and companion pins are the same as on the Tiny.

Firmware-driven pins:

| Function | GP pin | Notes |
|---|---|---|
| I2C SDA | GP4 | I2C0. IMU and barometer. |
| I2C SCL | GP5 | I2C0. |
| Aileron servo, left | GP6 | 50 Hz servo PWM. |
| Aileron servo, right | GP7 | 50 Hz servo PWM. |
| Elevator servo | GP8 | 50 Hz servo PWM. |
| Rudder servo | GP9 | 50 Hz servo PWM. Also drives the nose-wheel servo, wired to the same signal. |
| ESC, right engine | GP10 | Motor signal. PWM, OneShot125, or DShot300/600. |
| ESC, left engine | GP11 | Motor signal. |
| SBUS receiver | GP1 | PIO inverted UART. No external inverter. |
| SD MOSI | GP19 | SPI0. |
| SD MISO | GP16 | SPI0. |
| SD CLK | GP18 | SPI0. Module pin reads CLK. |
| SD CS | GP17 | SPI0. |
| Companion UART TX | GP20 | UART1 to the ESP companion. |
| Companion UART RX | GP21 | UART1. |
| Status LED, external | GP14 | Optional, with a 470 Ω resistor. |
| Onboard LED | GP25 | Boot heartbeat. Built in (the board's LED2). |

Caribou aux pins, driven by `user-sketch.ino` rather than the firmware mixer:

| Function | GP pin | Notes |
|---|---|---|
| Nav lights, LED2 | GP2 | 1 Hz blink. |
| Flap servo 1 | GP12 | Both wings on one channel. |
| Flap servo 2 | GP13 | |
| Bay door 1 | GP15 | |
| Retracts, all three | GP22 | All three gear legs on one channel. |
| Bay door 2 | GP28 | |

GP3, GP23, GP24, GP26, GP27, and GP29 are spare. GP3 used to drive the nose-wheel steering, which is now mechanical off the rudder. On a Pico 2 W, GP23, GP24, GP25, and GP29 are taken by the on-board radio instead, and the heartbeat LED moves to the external GP14.

## Pin map (Waveshare RP2350-Tiny, alternate)

| Function | GP pin | Notes |
|---|---|---|
| I2C SDA | GP4 | I2C0. IMU and barometer share this bus. |
| I2C SCL | GP5 | I2C0. |
| Motor 1 (right) | GP10 | OneShot125 ESC signal. Bit-banged. |
| Motor 2 (left) | GP11 | OneShot125 ESC signal. |
| Servo 1 | GP12 | Standard 50 Hz servo PWM. Left elevon on the tailsitter. |
| Servo 2 | GP13 | Standard 50 Hz servo PWM. Right elevon on the tailsitter. |
| Servo 3 | GP8 | Twin-cargo plane only. Unused on the tailsitter. |
| Servo 4 | GP9 | Twin-cargo plane only. Unused on the tailsitter. |
| SBUS receiver | GP1 | PIO inverted UART. Wire the receiver SBUS line directly. No external inverter. |
| SD card MOSI | GP19 | SPI0 TX. |
| SD card MISO | GP16 | SPI0 RX. |
| SD card CLK | GP18 | SPI0 SCK. The SD module silkscreens this pin CLK. |
| SD card CS | GP17 | Standard chip-select. |
| Onboard LED (boot indicator) | GP25 | Built-in. Heartbeat blink during normal operation. Fast 5 Hz blink indicates an init failure. |
| Status LED (external, optional) | GP14 | Reserved for an external 3 mm status LED with a 470 Ω current-limiting resistor. |
| Companion UART RX | GP21 | UART1. ESP companion (v1.1) or GPS (v2). |
| Companion UART TX | GP20 | UART1. |

## Power

The reference build uses OPTO ESCs (no built-in BEC) plus a standalone 5 V switching UBEC.

- 4S LiPo battery feeds both ESCs in parallel.
- The UBEC's 5 V output feeds the FC board's VBUS pad, the receiver's VCC, and both servos.
- The RP2350's onboard 3.3 V regulator powers the I2C sensors (IMU, barometer) and the SD card module.
- Single common ground star-grounded at the UBEC negative.

If the ESC has an integrated BEC instead of being OPTO, omit the external UBEC. Do not run two BECs in parallel; that is a documented failure mode.

## Receiver wiring

Wire the receiver signal line directly to GP1 either way. Power the receiver from the 5 V UBEC rail and ground to the FC common ground. Pick the protocol with `RX_PROTOCOL` in `Config.h`.

**SBUS** (`RX_SBUS`). The RP2350 hardware UART cannot invert the SBUS signal in silicon, so CrowPilot handles inversion in a PIO state machine. No transistor inverter is required. CrowPilot reads inverted SBUS only: the PIO is fixed to the inverted-SBUS line polarity (idle low, start bit high), which is what FrSky and most SBUS receivers output. `RX_SBUS_INVERTED` flips only the captured data bits, not the start-bit detection, so a receiver emitting non-inverted SBUS will not decode. Set such a receiver to inverted SBUS, or use CRSF.

**CRSF** (`RX_CRSF`), for Crossfire and ELRS. A plain 420 kbaud UART on GP1, not inverted, so no PIO and no inverter. This is the native output of ELRS receivers and the recommended path for them.

## I2C bus

The IMU and the barometer both sit on the I2C0 bus. They share two signal lines: every module's SDA-type pin goes to GP4, and every module's SCL-type pin goes to GP5. Wire each module from its own table below. Find the label on your module, then run a wire to the listed flight-controller pin. A pin marked "not used" is left open (no wire).

Both modules also connect to the same 3.3 V and GND pins on the flight controller. That is correct, the bus is shared.

If you wire bare chips instead of breakout boards, add a 4.7 kΩ resistor from GP4 to 3.3 V and another from GP5 to 3.3 V. Most breakout boards already have these.

### IMU module: MPU-6500 or MPU-6050

| Pin on the IMU module | Wire it to (flight controller) |
|---|---|
| VCC | 3.3 V |
| GND | GND |
| SCL | GP5 |
| SDA | GP4 |
| EDA | not used |
| ECL | not used |
| ADO | GND (this sets I2C address 0x68, the firmware default) |
| INT | not used |
| NCS | not used |
| FSYNC | not used |

To use address 0x69 instead, wire ADO to 3.3 V and set `IMU_I2C_ADDR` to 0x69 in `src/Config.h`.

An MPU-6050 GY-521 board labels three of these pins differently: XDA, XCL, and AD0 in place of EDA, ECL, and ADO. Wire VCC, GND, SCL, SDA, and AD0 (to GND) the same way, and leave XDA, XCL, and INT not used.

### Barometer module: BMP388

| Pin on the BMP388 module | Wire it to (flight controller) |
|---|---|
| VIN | 3.3 V |
| 3Vo | not used (this is the module's 3.3 V output, not an input) |
| GND | GND |
| SCK | GP5 |
| SDO | 3.3 V (this sets I2C address 0x77, the firmware default) |
| SDI | GP4 |
| CS | 3.3 V (this selects I2C mode) |
| INT | not used |

On the BMP388 the I2C clock is the pin marked SCK and the I2C data is the pin marked SDI, not SCL and SDA. The CS pin must go to 3.3 V, or the chip stays in SPI mode and never answers on I2C. To use address 0x76 instead, wire SDO to GND and set `BARO_I2C_ADDR` to 0x76 in `src/Config.h`.

## SD card

The SD module sits on the SPI0 bus. Wire each module pin to the listed flight-controller pin. The clock pin is labeled CLK on the module, which is the same line the RP2350 pin map calls SPI SCK.

| Pin on the SD module | Wire it to (flight controller) |
|---|---|
| 3V3 | 3.3 V |
| GND | GND |
| MOSI | GP19 |
| MISO | GP16 |
| CLK | GP18 |
| CS | GP17 |

Most microSD breakouts are 3.3 V-native and do not need a level shifter.

The SD card must be FAT32 formatted. FAT16 and exFAT are not supported by the Arduino-Pico `SD.h` library used in v1.

## Servos

On the WeAct Caribou build the four control-surface servos are on GP6 (left aileron), GP7 (right aileron), GP8 (elevator), and GP9 (rudder). On the Waveshare-Tiny tailsitter build the two elevons are on GP12 and GP13, with GP8 and GP9 free for the user extension hook. All servos run standard 1 ms to 2 ms pulses at 50 Hz, handled by the Pi Pico Arduino core's `Servo.h`.

Power servos from the 5 V UBEC rail. Servo current can spike during stall, so size the UBEC for at least 3 A.

## ESCs

ESCs go on GP10 (motor 1, right) and GP11 (motor 2, left). The default `MOTOR_PROTOCOL` is standard hobby PWM: pulses fall in the 1000 to 2000 microsecond range during normal operation and sit at `ESC_DISARM_PULSE_US` (1000 microseconds, idle) when CrowPilot is in the NOT_ARMED state. Set `MOTOR_PROTOCOL_ONESHOT125` for OneShot ESCs, where the running range is 125 to 250 microseconds and the disarm pulse drops below 125.

For BLHeli_S or BLHeli_32 ESCs, set `MOTOR_PROTOCOL_DSHOT300` or `MOTOR_PROTOCOL_DSHOT600`. DShot is a digital protocol: a PIO state machine clocks a 16-bit, CRC-checked frame to each motor pin, so the same GP10 and GP11 lines carry the digital signal instead of a pulse width. DShot needs no ESC calibration and is immune to pulse-width drift. The DShot output uses PIO1, which does not collide with the SBUS receiver's PIO0. Match the rate the ESC is flashed for: DShot600 is the common choice and DShot300 is the more tolerant of long or noisy signal leads.

Bidirectional DShot (`ENABLE_DSHOT_BIDIR`) adds no wires: the ESC reports its RPM back on the same signal lead, between command frames. It needs an ESC flashed for bidirectional telemetry, and the RPM feeds the dynamic gyro notch (`ENABLE_DYNAMIC_NOTCH`). The receive timing is bench-tuned and unverified in v1, so leave both options off until you have checked the telemetry on the bench with a logic analyzer or a known-good ESC. Set `MOTOR_POLE_PAIRS` to your motor's magnet pole-pair count (7 for a typical 14-magnet outrunner) so the reported eRPM converts to the right frequency.

When disarmed, CrowPilot holds the motors stopped (the disarm pulse for PWM and OneShot, the motor-stop command for DShot). Once CrowPilot arms (`CHANNEL_ARM` LOW and the throttle stick at idle), the outputs follow the throttle command.

For ESCs that require explicit calibration (less common with BLHeli, more common with older ESCs, and never for DShot), see [docs/user-guide/tuning.md](../user-guide/tuning.md) and set `ENABLE_ESC_CALIBRATION = 1` in `src/Config.h`.

## ESP32-C3 wireless companion (optional)

The WiFi companion is optional. It connects to the flight controller's companion UART (GP20 and GP21) and serves the phone UI over the cp protocol. Skip this section if you are not using WiFi. Skip it too if you use a Raspberry Pi Pico 2 W as the flight controller, since its radio is on-board and needs no separate module.

The transmit and receive lines cross over: the flight controller's transmit pin goes to the ESP's receive pin, and the ESP's transmit pin goes to the flight controller's receive pin. Both sides are 3.3 V logic, so no level shifter is needed.

The reference ESP board is the **Seeed XIAO ESP32-C3**. Its silkscreen labels the pins `D0..D10` rather than the raw GPIO numbers. The companion link uses `D6` and `D7`, which carry GPIO21 and GPIO20 respectively (the labels and the GPIO numbers do not line up, so it is easy to wire backwards if you match digits naively).

| FC pin | XIAO silkscreen | XIAO GPIO | What it does |
|--------|-----------------|-----------|---|
| GP20 | D7 | GPIO20 (U0RXD) | FC transmits, ESP receives |
| GP21 | D6 | GPIO21 (U0TXD) | ESP transmits, FC receives |
| 5V | 5V | VBUS | powers the ESP |
| GND | GND | GND | common ground, required |

The TX and RX lines cross over by name. The XIAO `D7` pin sits at ESP GPIO20 which is the UART RX, so it pairs with the FC's TX on GP20. The XIAO `D6` pin sits at ESP GPIO21 which is the UART TX, so it pairs with the FC's RX on GP21.

The ESP-side GPIO assignment is set in `esp-companion/esp-companion.ino` and can move to any free ESP32-C3 GPIO if your ESP board is not a XIAO. Power the ESP from the 5 V rail, not the flight controller's 3.3 V regulator: WiFi transmit current peaks above what the RP2350's on-board regulator should supply, and the ESP board makes its own 3.3 V.

### Passthrough flashing (optional, no ESP USB needed)

Two extra control jumpers let the flight controller flash the ESP itself, so the ESP never needs a USB cable. The FC pulses the ESP's EN line to reset it and holds GPIO9 (the boot strap) low at reset to drop the chip into its UART ROM bootloader, then bridges USB CDC to the companion UART for the duration of the flash. The cp serial command is `cp esp flash`. This works on the WeAct, the Waveshare Tiny, and the Pico 2 W. The SmartElex NEO does not break out enough free GPIO for this and is skipped.

| FC pin | XIAO label | XIAO GPIO | What it does |
|--------|------------|-----------|---|
| GP22 | EN pad | CHIP_EN | FC drives this low to reset the ESP |
| GP3 | D9 | GPIO9 | FC holds this low at reset to enter the ROM bootloader |

On the XIAO ESP32-C3, the `EN` line is **not broken out as a normal pin**. It is a small pad or via on the board, typically near the RESET button. You may need to solder a wire to it. The `GPIO9` strap is on the `D9` pin and is easy to reach. If soldering to EN is not practical, skip both jumpers and keep using the ESP's own USB port for flashing it; the companion link (D6/D7, 5V, GND) still works without them.

With those two jumpers in place, on the host run something like `esptool.py --port <FC serial port> --before no_reset --baud 115200 write_flash 0x0 esp-companion.bin`, after sending `cp esp flash` over the same serial port. The bridge returns to the cp prompt after 60 seconds with no traffic, then resets the ESP into its application. The command is refused while armed and is USB-only (it takes the USB CDC over for the duration of the flash, so it cannot be run from the companion link itself).

If you do not wire these two extra lines, the ESP still needs one initial USB flash through its own port, but everything else (the cp uplink and the phone UI) works without them.

## Wiring checklist before first power-on

- Battery DISCONNECTED.
- Propellers REMOVED.
- All ground connections meet at a common star point (UBEC negative is a good choice).
- 5 V rail measured between UBEC output and ground reads ~5.0 V before any logic is connected.
- 3.3 V rail (when FC is USB-powered) reads ~3.3 V.
- I2C lines have continuity to the sensor breakouts.
- SBUS line traces from receiver to GP1 with no intermediate transistor.
- ESC signal lines go to GP10 and GP11; ESC battery leads are NOT connected yet.
- Servos are wired but not yet on the airframe.

When all of the above checks pass, follow [docs/getting-started/caribou-bench-test.md](caribou-bench-test.md) for the bench bring-up sequence on the Caribou. For the carried tailsitter airframe, use [first-bench-test.md](first-bench-test.md).

<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Wiring

Pin assignments and wiring for the Waveshare RP2350-Tiny reference build. For the WeAct RP2350A_V10 and other board profiles, see [docs/reference/pin-maps.md](../reference/pin-maps.md).

## Pin map (Waveshare RP2350-Tiny, canonical)

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

## SBUS receiver wiring

The RP2350 hardware UART cannot invert the SBUS signal in silicon. CrowPilot handles inversion in a PIO state machine. No transistor inverter is required.

Wire the SBUS line from the receiver directly to GP1. Power the receiver from the 5 V UBEC rail and ground to the FC common ground.

CrowPilot v1 supports inverted SBUS only. The PIO program is fixed to the inverted-SBUS line polarity (idle low, start bit high), which is what FrSky and most SBUS receivers output. `RX_SBUS_INVERTED` only flips the captured data bits, not the start-bit detection, so a receiver that emits non-inverted SBUS will not decode. If your ELRS receiver outputs non-inverted SBUS, configure it for inverted SBUS output instead.

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

Servos go on GP12 (left elevon) and GP13 (right elevon) for the Eclipson-style tailsitter. Standard 1 ms to 2 ms pulse range at 50 Hz. The Pi Pico Arduino core's `Servo.h` handles the PWM hardware internally.

A twin-cargo plane build uses four servos and additionally drives GP8 and GP9. On the tailsitter those two pins are unused and free for the user extension hook.

Power servos from the 5 V UBEC rail. Servo current can spike during stall; size the UBEC for at least 3 A.

## ESCs

ESCs go on GP10 (motor 1, right) and GP11 (motor 2, left). The default `MOTOR_PROTOCOL` is standard hobby PWM: pulses fall in the 1000 to 2000 microsecond range during normal operation and sit at `ESC_DISARM_PULSE_US` (1000 microseconds, idle) when CrowPilot is in the NOT_ARMED state. Set `MOTOR_PROTOCOL_ONESHOT125` for OneShot ESCs, where the running range is 125 to 250 microseconds and the disarm pulse drops below 125.

When disarmed, CrowPilot holds the motors at the disarm pulse so they stay stopped. Once CrowPilot arms (`CHANNEL_ARM` LOW and the throttle stick at idle), the pulses follow the throttle command.

For ESCs that require explicit calibration (less common with BLHeli, more common with older ESCs), see [docs/user-guide/tuning.md](../user-guide/tuning.md) and set `ENABLE_ESC_CALIBRATION = 1` in `src/Config.h`.

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

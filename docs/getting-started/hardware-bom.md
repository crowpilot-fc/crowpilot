<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Hardware Bill of Materials

The flight-controller parts cost about USD 20 in total. Propulsion, airframe, servos, receiver, and battery are user-supplied and priced separately; CrowPilot is happy on any small foam-board, balsa, or 3D-printed plane or bicopter.

## Flight controller and sensors

| Item | Reference part | Approximate cost | Notes |
|---|---|---|---|
| Microcontroller board | Waveshare RP2350-Tiny | $6 | ~21 × 18 mm. USB-C via detachable adapter. RP2350A, dual M33 at 150 MHz with hardware FPU, 16 MB flash. |
| IMU (preferred) | MPU-6500 breakout | $2 | I2C, configured at ±8 g. Lower gyro noise than the MPU-6050. |
| IMU (budget alternate) | MPU-6050 (GY-521 breakout) | $1 | I2C, configured at ±8 g. Slightly noisier than MPU-6500. |
| Barometer (default) | BMP388 breakout | $3 | I2C, sub-Pa noise floor. Better for v1.1 altitude hold. |
| Barometer (alternate) | BMP280 (GY-BMP280 breakout) | $2 | I2C, cheaper. |
| SD card module | 6-pin micro-SD reader | $2 | SPI0. 3.3 V logic compatible without a level shifter on most modules. |
| SD card | 4 GB or larger Class 10 microSD | $3 | FAT32 format. |
| Wiring | Wire, headers, status LED, 470 Ω resistor | $5 | Bench supplies. |

Total: roughly USD 20.

Buy a genuine MPU-6500 or MPU-6050. The IMU driver gates initialization on the chip `WHO_AM_I` value (`0x70` for the MPU-6500, `0x68` for the MPU-6050) and refuses any other ID. Many cheap breakouts sold as "MPU-6500" carry a different die, such as an MPU-9250, ICM-20602, or an unmarked clone, and will fail init with `ERROR: IMU init failed`. The strict check is deliberate: an unverified chip can have a different register map or sensitivity scale, which would silently corrupt the attitude estimate.

## Alternate microcontroller boards

| Board | Approximate cost | When to pick |
|---|---|---|
| Waveshare RP2350-Tiny | $6 | Primary in-airframe target. Smallest footprint. |
| Waveshare RP2350-Zero | $5 | Compact alternate. Permanent USB-C plus onboard WS2812 RGB LED. |
| Waveshare RP2350-One | $5 | Pi-Pico form factor. Easy to breadboard for development. |
| WeAct RP2350A_V10 | $6 | Pi-Pico form factor. Legacy alternate. |
| Raspberry Pi Pico 2 W | $7 | RP2350A with an onboard WiFi radio. Serves the companion UI itself, so no separate ESP is needed. Same flight pins as the WeAct profile. |
| SmartElex RP2350A NEO | $5 | Small castellated board, 16 GPIO. Good for a basic plane. The onboard LED is a WS2812, so use an external status LED. |

Board profiles ship for the Waveshare RP2350-Tiny, the WeAct RP2350A_V10, the Raspberry Pi Pico 2 W, and the SmartElex RP2350A NEO. The Waveshare RP2350-Zero and RP2350-One are pin-similar and supported in principle, but they do not ship a profile header yet, so they need a small `src/boards/*.h` pin map added before use.

The Seeed XIAO RP2350 is **not supported** because it only breaks out 11 GPIO, which is too few for the v1 flight controller.

WiFi is optional. A plain RP2350 board (above) is the cheapest build and has no WiFi. To add WiFi, either attach an ESP32-C3 companion to the companion UART (works on any board, keeps the radio off the flight MCU) or use the Pico 2 W, which has the radio built in.

## Propulsion, airframe, and the rest

CrowPilot does not prescribe a propulsion BOM. Pick motors, ESCs, propellers, servos, receiver, battery, UBEC, and airframe to match the aircraft you want to fly.

The current first-flight airframe is the DHC-4 Caribou, a twin-engine high-wing cargo plane. Its propulsion and power wiring are documented in [Wiring](wiring.md) and the [Caribou wiring diagram](../reference/caribou-wiring.svg). The firmware also supports a 3D-printed tailsitter VTOL bicopter as a carried alternate, and v1.2 will add a 3D-printed Gee Bee 80 mm EDF.

The propulsion example below is the tailsitter reference build, a roughly 1 kg-class Eclipson E-VTOL-1 (about USD 30 of STL files from the designer). A second supported tailsitter reference is the FliteTest Kraken Mk2 (about 2.5 kg class).

For the Eclipson 1 kg class:

- 2× motors at roughly 880 kV on 4S. SunnySky X2216 II is the reference.
- 2× ESCs rated 30 A or higher. Cyclone BLHeli_S 45A OPTO is the reference; OPTO means the ESCs have no built-in BEC, so a standalone UBEC is required.
- 2× 9 g metal-gear servos. EMAX ES08MD II class.
- 4S LiPo battery 1500 to 2200 mAh, 50 C or higher.
- 5 V / 3 A switching UBEC for the FC, RX, and servos.
- Any SBUS or CRSF (Crossfire / ELRS) receiver, selected with `RX_PROTOCOL`. No external inverter required: SBUS is read inverted in PIO, and CRSF is a plain UART.
- Propellers: 10 × 4.5 or 11 × 4.7, one CW + one CCW pair.

Scale these for heavier airframes. The mixer and PID code do not care about the specific parts; tune the gains in [docs/user-guide/tuning.md](../user-guide/tuning.md).

## What you also need on the bench

- USB-C cable for flashing.
- Soldering iron, solder, flux.
- Multimeter for power-rail and continuity checks.
- Optional but recommended: oscilloscope or logic analyzer for receiver and ESC signal verification.
- Optional: tether or test stand for tethered hover before free flight.

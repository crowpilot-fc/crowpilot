<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Pin maps

All supported board profiles. The active profile is selected at compile time via `BOARD` in `src/Config.h`. Adding a new profile is a single header in `src/boards/<board>.h`; see [docs/developer-guide/architecture.md](../developer-guide/architecture.md) for the contract.

## Waveshare RP2350-Tiny (reference build)

This is the primary v1 target. The pin layout was chosen to keep ESC and servo lines on adjacent GP pins for tidy wiring on a small breakout.

| Function | GP pin | Bus / peripheral | Notes |
|---|---|---|---|
| I2C SDA | GP4 | I2C0 | IMU + barometer shared. |
| I2C SCL | GP5 | I2C0 | |
| Motor 1 (right) | GP10 | GPIO bit-bang | OneShot125. |
| Motor 2 (left) | GP11 | GPIO bit-bang | OneShot125. |
| Servo 1 | GP12 | PWM slice (Servo lib) | 50 Hz, 1 to 2 ms. Left elevon on the tailsitter. |
| Servo 2 | GP13 | PWM slice (Servo lib) | Right elevon on the tailsitter. |
| Servo 3 | GP8 | PWM slice (Servo lib) | Twin-cargo plane only. Free on the tailsitter. |
| Servo 4 | GP9 | PWM slice (Servo lib) | Twin-cargo plane only. Free on the tailsitter. |
| SBUS receiver | GP1 | PIO SM 0 | Inverted UART in PIO. No external inverter. |
| SD MOSI | GP19 | SPI0 TX | |
| SD MISO | GP16 | SPI0 RX | |
| SD SCK | GP18 | SPI0 SCK | |
| SD CS | GP17 | GPIO | Standard chip-select. |
| Onboard LED | GP25 | GPIO | Boot heartbeat. Fast 5 Hz = init failed. |
| Status LED (external) | GP14 | GPIO | Optional 3 mm LED with 470 Ω current limit. |
| Companion UART RX | GP21 | UART1 RX | ESP companion (v1.1) or GPS (v2). |
| Companion UART TX | GP20 | UART1 TX | |

Select with `#define BOARD BOARD_WAVESHARE_RP2350_TINY` in `src/Config.h`.

## WeAct Studio RP2350A_V10

The WeAct RP2350A is functionally equivalent to the Waveshare RP2350-One and uses the Pi-Pico-form RP2350A package. The only difference from the Waveshare RP2350-Tiny profile is the servo pin choice: GP12/GP13 are not broken out on every Pi-Pico-form board, so servos move to GP6/GP7.

| Function | GP pin | Bus / peripheral | Notes |
|---|---|---|---|
| I2C SDA | GP4 | I2C0 | |
| I2C SCL | GP5 | I2C0 | |
| Motor 1 (right) | GP10 | GPIO bit-bang | OneShot125. |
| Motor 2 (left) | GP11 | GPIO bit-bang | OneShot125. |
| Servo 1 | GP6 | PWM slice (Servo lib) | Different from Waveshare profile. Left elevon on the tailsitter. |
| Servo 2 | GP7 | PWM slice (Servo lib) | Right elevon on the tailsitter. |
| Servo 3 | GP8 | PWM slice (Servo lib) | Twin-cargo plane only. Free on the tailsitter. |
| Servo 4 | GP9 | PWM slice (Servo lib) | Twin-cargo plane only. Free on the tailsitter. |
| SBUS receiver | GP1 | PIO SM 0 | |
| SD MOSI | GP19 | SPI0 TX | |
| SD MISO | GP16 | SPI0 RX | |
| SD SCK | GP18 | SPI0 SCK | |
| SD CS | GP17 | GPIO | |
| Onboard LED | GP25 | GPIO | |
| Status LED (external) | GP14 | GPIO | Optional. |
| Companion UART RX | GP21 | UART1 RX | |
| Companion UART TX | GP20 | UART1 TX | |

Select with `#define BOARD BOARD_WEACT_RP2350A_V10` in `src/Config.h`.

## RP2350-One (Waveshare, Pi-Pico form)

Use the WeAct profile. The two boards are pin-compatible.

## Adding a new board profile

To add a new RP2350-based board:

1. Create `src/boards/<board_name>.h`. Copy the structure from `src/boards/waveshare_rp2350_tiny.h`. Define every `PIN_*` constant that the firmware uses, in a `cp::boards::<board_name>` namespace, plus a `using namespace ...` block in `cp::` so module code can refer to them unqualified.
2. Add a `BOARD_*` selector to `src/Config.h` and the matching `#include` block.
3. Add the new board entry here and update [docs/getting-started/wiring.md](../getting-started/wiring.md) if the pinout differs meaningfully.

Constraints to honor:

- I2C must be on I2C0 (GP4/GP5 by default, or another I2C0-capable pair).
- SBUS receive pin must be a PIO-capable GPIO. All RP2350 GPIOs are PIO-capable, so this is a non-constraint in practice.
- SD card pins must be on SPI0.
- The onboard LED pin (GP25 on most RP2350 boards) is used as the boot indicator. If your board has a different onboard LED, override `PIN_LED_ONBOARD`.

## Pin conflict reference

Some pin choices are locked because of peripheral assignment. For reference, the RP2350 peripherals used by CrowPilot:

- **I2C0:** Used by IMU and barometer. Move to I2C1 if needed by passing a different `Wire` instance to the HAL, but the default expects I2C0.
- **SPI0:** Used by SD card. Move to SPI1 if needed similarly.
- **PIO0:** Used by the SBUS state machine on SM 0. SMs 1-3 are free.
- **UART1:** Used by the companion interface. UART0 is free (and is what the USB serial monitor uses via the `Serial` global).

If you need to free up I2C0 for a different sensor (e.g. a magnetometer in v2), you can move IMU and baro to I2C1; this is a HAL-only change.

<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Adding a board

CrowPilot supports a board through a per-board profile header. Adding a new RP2350-based board is a small, self-contained change: one header, one selector, and a verification pass.

## Why per-board profiles

Different RP2350 breakouts route the same RP2350 silicon to different physical pins, and some boards do not break out every GPIO. A per-board profile is a single header that names which GP pin each function uses. The firmware logic never refers to a raw pin number; it refers to `PIN_*` constants that the active profile defines. Selecting a board at compile time is the only thing that changes.

## The board-profile header

Create `src/boards/<board_name>.h`. Copy the structure from `src/boards/waveshare_rp2350_tiny.h`. Every profile must define this set of constants:

| Constant | Function |
|---|---|
| `PIN_I2C_SDA`, `PIN_I2C_SCL` | I2C0 bus for IMU and barometer. |
| `PIN_MOTOR_RIGHT`, `PIN_MOTOR_LEFT` | OneShot125 ESC signal pins. |
| `PIN_SERVOS[4]` | Servo PWM pins. Indices follow the airframe `SERVO_*` constants. The tailsitter uses the first two; the twin-cargo plane uses all four. |
| `PIN_SBUS_RX` | SBUS receive pin (PIO). |
| `PIN_SD_MOSI`, `PIN_SD_MISO`, `PIN_SD_SCK`, `PIN_SD_CS` | SPI0 SD card. |
| `PIN_LED_ONBOARD` | Boot indicator and panic blink. |
| `PIN_LED_STATUS` | Optional external status LED. |
| `PIN_COMPANION_TX`, `PIN_COMPANION_RX` | UART1 companion interface. |

The constants go in a `cp::boards::<board_name>` namespace. A trailing `using namespace` block hoists them into `cp::` so module code refers to them unqualified. Only one profile is included per build, so there is no collision.

Carry the SPDX header (`GPL-3.0-or-later`) and copyright line at the top, like every other source file.

## Pin map decisions

Some pins are constrained by peripheral assignment, others are free:

- **I2C** must be on an I2C0-capable pair. The HAL expects I2C0.
- **SD card** must be on SPI0.
- **SBUS** can be any GPIO; all RP2350 GPIOs are PIO-capable.
- **Motors, servos, LEDs** are free GPIO. Choose whatever your board breaks out conveniently.

For the full peripheral reference, see [reference/pin-maps.md](../reference/pin-maps.md).

## Adding the build target

1. Add a `BOARD_*` selector macro to `src/Config.h`.
2. Add the matching `#include` of your new profile header, gated on the selector.
3. Document the new board in [reference/pin-maps.md](../reference/pin-maps.md) and, if the pinout differs from the reference, in [getting-started/wiring.md](../getting-started/wiring.md).

## Testing the new profile

Before trusting a new profile, run the Phase-1-equivalent checks:

1. Flash and confirm the boot banner on the serial monitor.
2. Confirm the onboard LED blinks the 1 Hz heartbeat.
3. Work through the full [first bench test](../getting-started/first-bench-test.md). Every stage exercises a different pin group; if a pin constant is wrong, a stage will fail visibly.

Do not fly a new board profile until it has passed the full bench test.

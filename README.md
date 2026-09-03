<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# CrowPilot

CrowPilot is an experimental fixed-wing flight controller for the RP2350A.
The rebuild is intentionally limited to one receiver protocol and one safe
bring-up path:

- WeAct RP2350A V10 controller module
- SBUS receiver input
- Eight standard PWM outputs
- MPU6500 over SPI
- BMP388 over I2C
- MicroSD blackbox logging
- Optional ESP32 field companion, electrically disabled while armed

The flight firmware is being rebuilt on the Raspberry Pi Pico SDK. It has not
been flight-tested. Do not install a propeller or connect a motor capable of
producing thrust during development or initial bench testing.

The current architecture and implementation plan are maintained in
[Notion](https://app.notion.com/p/3d0f4e695ae381d5b3c9dfaea4ba3e7e).
The authoritative physical BOM, wiring schedule, and staged assembly checks are
in the [Rebuild V1 bench carrier build pack](https://app.notion.com/p/3d0f4e695ae3812eb996db04cc5e7621).

## Host tests

```sh
cmake -S . -B build/host -G Ninja
cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

## RP2350 firmware

Install the Raspberry Pi Pico SDK and Arm embedded toolchain, then run:

```sh
PICO_SDK_PATH=/path/to/pico-sdk cmake -S firmware -B build/firmware -G Ninja
cmake --build build/firmware
```

The first firmware milestone only implements safe PWM initialization, SBUS
manual passthrough, arming, receiver-loss failsafe, and a watchdog. Sensors,
logging, the companion, and stabilization are tracked as later milestones.

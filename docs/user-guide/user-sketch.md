<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Writing a user sketch

CrowPilot handles flight: stabilization, mixing, actuator output. Everything else on your aircraft (retractable gear, flaps, a cargo bay door, navigation lights, a smoke pump, a payload release, a custom sensor) belongs in the user sketch.

The user sketch is `user-sketch.ino` at the repository root. It is your file. CrowPilot calls into it through two functions and otherwise leaves it alone.

## Enabling the hook

The user hook is off by default. Turn it on in `src/Config.h`:

```cpp
#define ENABLE_USER_HOOK  1
```

With the hook off, `user_setup()` and `user_tick()` still compile but are never called. Your aircraft flies exactly the same.

## The two entry points

```cpp
void user_setup();
void user_tick();
```

- **`user_setup()`** runs once, after CrowPilot's flight stack has finished initializing. Configure your pins and initialize your state here.
- **`user_tick()`** runs repeatedly during flight, once every `USER_HOOK_RATE_DIV` main loop iterations (default every 5th tick, so 200 Hz at the 1 kHz loop). It runs after CrowPilot has committed the actuator output for the tick, so your code sees the freshest state.

## The time budget

`user_tick()` runs inside the flight loop. It shares the loop's time. CrowPilot enforces a budget:

- `USER_HOOK_BUDGET_US` (default 100 us) is the soft budget. Exceeding it logs an advisory warning.
- The hard limit (250 us) is the safety net. Three hard overruns and the hook is **disabled for the rest of the flight**. A reboot re-enables it.

Keep `user_tick()` short. No `delay()`. No dynamic allocation. No blocking I/O. Watch the budget with `DEBUG_PRINT_USER_HOOK = 1`, which prints a 1 Hz line:

```
user_hook enabled=1 calls=200 last_us=31 max_us=44 warns=0 strikes=0
```

Tune your code against `max_us` before you fly.

## Reading flight state

Your code reads sensors and flight state through the read-only `cp::user::` API. Include the header at the top of your sketch:

```cpp
#include "user_hook/SensorApi.h"
```

What it gives you:

| Call | Returns |
|---|---|
| `cp::user::channel(n)` | Receiver channel `n` (1 to 16) pulse width in microseconds |
| `cp::user::imu()` | Gyro (dps), accel (g), and temperature |
| `cp::user::baro()` | Pressure, temperature, relative altitude, and a `present` flag |
| `cp::user::fader()` | Transition fader, 0.0 forward to 1.0 hover |
| `cp::user::armed()` | True while the motors are armed |
| `cp::user::failsafeActive()` | True while the lost-link failsafe is active |
| `cp::user::attitudeForward()` | Roll, pitch, yaw in degrees |

Every accessor returns a copy. Your code cannot reach into or change flight state. That separation is deliberate: a bug in your sketch must not be able to destabilize the aircraft.

## Using pins

Use the standard Arduino calls:

```cpp
pinMode(22, OUTPUT);
digitalWrite(22, HIGH);
int reading = analogRead(26);
```

CrowPilot guards the pins it uses itself (motors, servos, I2C, SD card, SBUS, LEDs). A write to one of those is ignored, and the first time it happens CrowPilot prints a one-time warning over serial. Reads always pass through.

The free pins depend on the airframe, because each airframe claims a different set of servo pins. Check [pin-maps.md](../reference/pin-maps.md) and the warning output for your build. On the Waveshare RP2350-Tiny the free pins are generally `GP2, GP3, GP6, GP7, GP15, GP22, GP26, GP27, GP28`, with `GP26` to `GP28` also usable as analog inputs. A twin-engine plane build additionally claims `GP8` and `GP9`.

## What you cannot do

- No `delay()`. Use `delayMicroseconds()`, and keep the total under about 50 us.
- No dynamic allocation: no `new`, no `malloc`, no `String`, no `std::vector`.
- No blocking I/O. No direct access to CrowPilot's I2C bus or the SD card.
- No changing flight state. The `cp::user::` API is read-only by design.

## The Caribou example

The `user-sketch.ino` shipped in the repository is a worked example for the Caribou twin-engine cargo plane. It drives:

- **Sequenced retractable gear.** Three electric retracts deploy nose-first, then left, then right, with a half-second delay between each leg.
- **Three-position flaps.** A transmitter switch maps to flaps up, half, and full.
- **A cargo bay door.** A two-position switch opens and closes it.
- **Navigation lights.** A two-position switch toggles them.

Read it as a template. The patterns it shows (a state machine for sequencing, a switch read mapped to discrete servo positions, free-pin assignment) cover most scale-feature needs. Replace it with your own code for your own aircraft.

## A minimal sketch

The smallest useful sketch toggles one output from one channel:

```cpp
#include "user_hook/SensorApi.h"

constexpr uint8_t GP_LIGHT = 22;

void user_setup() {
  pinMode(GP_LIGHT, OUTPUT);
}

void user_tick() {
  digitalWrite(GP_LIGHT, cp::user::channel(9) > 1500 ? HIGH : LOW);
}
```

Start there, bench-test it, then build up. Verify every pin and every channel on the bench before flight, the same way you verified the flight controller itself in [first-bench-test.md](../getting-started/first-bench-test.md).

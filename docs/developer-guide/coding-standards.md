<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Coding standards

The conventions every CrowPilot source file follows. Flight-controller code is read once and maintained for years; the standards favor consistency over cleverness.

## Language and toolchain

- **C++17.** Required by parts of the Pi Pico Arduino core.
- **Compiler.** The Arduino-Pico GCC ARM toolchain at default settings.
- **No exceptions.** Do not write `throw`, `try`, or `catch`.
- **No RTTI.** Do not write `dynamic_cast` or `typeid`.
- **No STL containers in the hot path.** No `std::vector`, `std::map`, `std::string`, `std::unique_ptr` in any code that runs in `tick()` or its callees. Use fixed-size arrays and structs.
- **No dynamic allocation after `setup()` returns.** No `new`, no `malloc`, no `make_unique` in `loop()` or anything it calls.

## File layout

Every source file is structured top to bottom:

```cpp
// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#pragma once

#include <stdint.h>          // C standard library first
#include <Arduino.h>          // framework second

#include "Config.h"           // local headers last, quoted
#include "State.h"

namespace cp::module_name {

// declarations

}  // namespace cp::module_name
```

`.cpp` files mirror this without `#pragma once`, and include their own header first.

- **SPDX and copyright** on lines 1 and 2 of every file. No padding, no rationale.
- **Header guards** are always `#pragma once`, never `#ifndef`.
- **Include order:** C standard library, C++ standard library, Arduino framework, project headers. Alphabetical within each group.
- **Namespace.** Every module lives in `cp::module_name`. Anonymous namespaces in `.cpp` files are encouraged for file-local helpers.

## Naming

| Kind | Convention | Example |
|---|---|---|
| Local variable, parameter | `lower_snake_case` | `gyro_x_dps`, `dt_s` |
| Function | `lowerCamelCase` | `attitudeUpdate`, `readBurst` |
| Type (struct, class, enum) | `UpperCamelCase` | `Quaternion`, `FlightMode` |
| True constant | `kUpperCamelCase` | `kPi` |
| Config tunable | `UPPER_SNAKE_CASE` | `MAX_ROLL_DEG` |
| Macro | `UPPER_SNAKE_CASE` | `IMU_MPU6500` |
| Namespace | `lower_snake_case` | `cp::sensors` |
| File | `UpperCamelCase.h` / `.cpp` | `Attitude.h` |
| Private member | trailing underscore | `t_prev_us_` |

PID gain constants (`Kp_roll_hover` and friends) keep the `Kp`/`Kd`/`Ki` notation that the tuning literature and audience expect. That exception applies to gain constants only.

## Module structure

Each module is a header (public API) plus a `.cpp` (implementation). The header declares the namespace and the public free functions. Module state lives in an anonymous namespace at file scope in the `.cpp`, never as `static` inside a function and never as a global extern.

Values shared across modules go in `src/State.h` as `extern` declarations, defined in exactly one `.cpp`. The owning module's `update()` writes the state; consumers read it.

## Function design

- **Short.** A function that does not fit on one screen is too long.
- **One return in the hot path.** Early returns are fine for boring helpers; the main flight-loop functions stay single-return for predictable timing.
- **Pure where possible.** Reserve `void update()` for module entry points.
- **No virtual functions in the hot path.** Facades dispatch at compile time.
- **`constexpr`** only for values that genuinely ought to be compile-time.

## Variables and types

- **Fixed-width integers** (`<stdint.h>`) for anything that crosses an API boundary, lives in a struct, or touches hardware. Plain `int` is fine for loop counters.
- **`float`, never `double`.** The RP2350 FPU is single-precision; `double` math falls through to slow software emulation. Always suffix float literals with `f`.
- **`const`** for locals and parameters that should not be reassigned. **`auto`** sparingly: fine for range-for and iterators, write the type out for numerics.

## Error handling

There are no exceptions. Errors are handled three ways:

1. **Sensor unreachable at init.** Print a clear `ERROR:` line over serial, set the fast panic-blink on the onboard LED, and halt in `setup()` before any motor output is enabled. This is the only acceptable infinite loop.
2. **Sensor failure mid-flight.** Mark the reading stale, substitute the last good value or a safe default, count the fault. The flight does not stop; failsafe handles total sensor loss separately.
3. **Configuration errors.** Caught at compile time with `#error` directives. Never silently fall through to a runtime default.

## Comments

Default to no comments. The exceptions:

- **Comment what is non-obvious:** a hidden invariant, a clamp that prevents a NaN, a numerical edge case.
- **Cite the source** for non-trivial math (datasheet equation, paper reference).
- **Mark genuine workarounds** with the greppable keyword `WORKAROUND`.
- Do not comment the obvious, the author, the date, or the current task. The version-control history and commit message cover those. No decorative section banners.
- Public-API headers may carry a one- or two-sentence plain-comment description. No Doxygen blocks.

## Hot-loop discipline

The flight loop runs at 1 kHz: 1000 microseconds shared across every module.

- **No blocking.** No `delay()`. `delayMicroseconds()` is acceptable only for the short fixed waits inside the OneShot125 bit-bang. Gate every serial print on `if (Serial)`.
- **No dynamic allocation.** No `new`, `malloc`, `String`. Use `char[N]` buffers and fixed-size arrays.
- **Cache reciprocals.** Float division is several times slower than multiply on the M33 FPU. Compute `1.0f / x` once at init.
- **Avoid `sqrt()` in the inner loop.** Compare squared magnitudes where possible.
- **No `printf` family.** It pulls in significant code size and runtime cost.

## What we do not do

- No template metaprogramming beyond what compile-time config dispatch needs.
- No `std::function` or type-erased callables. Function pointers suffice.
- No `std::optional`, `std::variant`, or other C++17 vocabulary types in flight code.
- No `goto`.
- No new third-party libraries. Driver code is written from datasheets, which is what keeps CrowPilot a clean-build project.

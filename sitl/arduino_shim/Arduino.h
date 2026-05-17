// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// Minimal Arduino compatibility shim for the SITL host build. The
// firmware logic includes <Arduino.h> for timing, the Serial console,
// and a few pin calls. On the host this header stands in for the
// Arduino-Pico core: timing comes from the host clock, Serial prints to
// stdout, and the pin calls are no-ops. It is only ever on the include
// path for the CMake host build, never the real Arduino build.

#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <chrono>
#include <string>

// --- Pin and level constants -----------------------------------------------

#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LED_BUILTIN 25

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#ifndef HALF_PI
#define HALF_PI 1.5707963267948966192313216916398
#endif
#ifndef TWO_PI
#define TWO_PI 6.283185307179586476925286766559
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

// --- Timing ----------------------------------------------------------------

// Microseconds since the first call, wrapping at 2^32 like the real
// micros(). The host steady clock is the time source.
inline uint32_t micros() {
  static const auto t0 = std::chrono::steady_clock::now();
  const auto now = std::chrono::steady_clock::now();
  const auto us =
      std::chrono::duration_cast<std::chrono::microseconds>(now - t0).count();
  return static_cast<uint32_t>(us);
}

inline uint32_t millis() {
  return micros() / 1000u;
}

inline void delayMicroseconds(uint32_t us) {
  const uint32_t start = micros();
  while ((micros() - start) < us) {
  }
}

inline void delay(uint32_t ms) {
  delayMicroseconds(ms * 1000u);
}

inline void yield() {}

// --- Pin I/O (no-ops on the host) ------------------------------------------

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return 0; }
inline int analogRead(uint8_t) { return 0; }
inline void analogWrite(uint8_t, int) {}

// Arduino-Pico fast-GPIO variants, used by the OneShot125 bit-bang.
inline void pinModeFast(uint8_t, uint8_t) {}
inline void digitalWriteFast(uint8_t, uint8_t) {}
inline int digitalReadFast(uint8_t) { return 0; }

// --- String conversion -----------------------------------------------------

// Arduino's float-to-string. width is the minimum field width (negative
// left-justifies), prec the decimal places. The caller owns sout, as on
// the real platform.
inline char* dtostrf(double val, int width, int prec, char* sout) {
  // Arduino's dtostrf has no buffer-size parameter; the caller owns the
  // sizing, exactly as on the real platform. snprintf with a bound that
  // covers any realistically formatted value keeps the host build from
  // tripping the platform sprintf-deprecation warning.
  snprintf(sout, 64, "%*.*f", width, prec, val);
  return sout;
}

// --- Serial console --------------------------------------------------------

// Stands in for the USB CDC Serial. Output goes to stdout. Input is not
// wired in the host-build checkpoint, so available() reports nothing.
class SerialClass {
 public:
  void begin(long) {}
  void end() {}
  explicit operator bool() const { return true; }

  int available() { return 0; }
  int read() { return -1; }
  int peek() { return -1; }
  void flush() { fflush(stdout); }

  size_t write(uint8_t c) {
    putchar(c);
    return 1;
  }
  size_t write(const uint8_t* p, size_t n) {
    return fwrite(p, 1, n, stdout);
  }

  size_t print(const char* s) {
    if (s == nullptr) {
      return 0;
    }
    fputs(s, stdout);
    return strlen(s);
  }
  size_t print(char c) {
    putchar(c);
    return 1;
  }
  size_t print(double v, int digits = 2) {
    return static_cast<size_t>(printf("%.*f", digits, v));
  }
  template <class T>
  size_t print(T v) {
    const std::string s = std::to_string(v);
    fputs(s.c_str(), stdout);
    return s.size();
  }

  size_t println() {
    putchar('\n');
    return 1;
  }
  size_t println(const char* s) {
    const size_t n = print(s);
    putchar('\n');
    return n + 1;
  }
  size_t println(char c) {
    const size_t n = print(c);
    putchar('\n');
    return n + 1;
  }
  size_t println(double v, int digits = 2) {
    const size_t n = print(v, digits);
    putchar('\n');
    return n + 1;
  }
  template <class T>
  size_t println(T v) {
    const size_t n = print(v);
    putchar('\n');
    return n + 1;
  }
};

inline SerialClass Serial;

// --- RP2350 core object ----------------------------------------------------

// The Arduino-Pico core exposes a global `rp2040`. Only rebootToBootloader
// is reached by the firmware logic (the cp boot command). On the host it
// is a no-op.
struct Rp2040Shim {
  void rebootToBootloader() {}
};

inline Rp2040Shim rp2040;

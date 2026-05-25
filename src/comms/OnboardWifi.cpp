// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/comms/OnboardWifi.h"

#include "src/Config.h"

#if ENABLE_ONBOARD_WIFI

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <pico/unique_id.h>

// The phone UI is the same single-page app the ESP companion serves, shared
// verbatim so the two companions never drift. It defines const char kWebUi[]
// and connects its WebSocket to ws://<this host>:81, so it works unchanged
// whether the page comes from the ESP or from here.
#include "esp-companion/web_ui.h"

namespace cp::comms::onboard_wifi {

namespace {

// Access point. The SSID gets a per-board suffix from the unique board ID so
// two aircraft on the same field do not collide. Change the password before
// field use; WPA2 needs at least 8 characters.
constexpr char     kApSsidPrefix[] = "CrowPilot-";
constexpr char     kApPassword[]   = "crowpilot";
constexpr uint16_t kHttpPort       = 80;
constexpr uint16_t kWsPort         = 81;
constexpr uint32_t kLinkBlinkMs    = 500;  // 1 Hz radio-LED heartbeat.
constexpr size_t   kLineMax        = 256;  // longest cp line (a telemetry line).

// Lock-free single-producer / single-consumer byte ring. One core pushes, the
// other pops, so no lock is needed: a full barrier orders the data write
// against the index publish. Size must be a power of two.
template <uint32_t N>
class ByteRing {
 public:
  bool push(uint8_t b) {  // producer core only
    const uint32_t h    = head_;
    const uint32_t next = (h + 1) & (N - 1);
    if (next == tail_) {
      return false;  // full, drop
    }
    buf_[h] = b;
    __sync_synchronize();
    head_ = next;
    return true;
  }
  int pop() {  // consumer core only
    const uint32_t t = tail_;
    if (t == head_) {
      return -1;  // empty
    }
    const uint8_t b = buf_[t];
    __sync_synchronize();
    tail_ = (t + 1) & (N - 1);
    return b;
  }
  int peek() const {
    if (tail_ == head_) {
      return -1;
    }
    return buf_[tail_];
  }
  int available() const {
    return static_cast<int>((head_ - tail_) & (N - 1));
  }

 private:
  volatile uint32_t head_ = 0;
  volatile uint32_t tail_ = 0;
  uint8_t           buf_[N];
};

// rx: WebSocket (core1, producer) -> cp CLI (core0, consumer). Commands short.
// tx: cp CLI (core0, producer) -> WebSocket (core1, consumer). Telemetry lines.
ByteRing<256>  s_rx;
ByteRing<2048> s_tx;

// The Stream the core0 CLI talks to. read() drains the rx ring, write() fills
// the tx ring. The CLI never knows it is talking across cores.
class BridgeStream : public Stream {
 public:
  int available() override { return s_rx.available(); }
  int read() override { return s_rx.pop(); }
  int peek() override { return s_rx.peek(); }
  size_t write(uint8_t b) override { return s_tx.push(b) ? 1 : 0; }
  size_t write(const uint8_t* b, size_t n) override {
    size_t w = 0;
    while (w < n && s_tx.push(b[w])) {
      ++w;
    }
    return w;
  }
  void flush() override {}
};

BridgeStream     s_bridge;
WebServer        s_http(kHttpPort);
WebSocketsServer s_ws(kWsPort);

char     s_line[kLineMax];
size_t   s_line_len    = 0;
bool     s_up          = false;
uint32_t s_led_last_ms = 0;
bool     s_led_state   = false;

void handleRoot() {
  s_http.send_P(200, "text/html", kWebUi);
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  (void)num;
  if (type == WStype_TEXT) {
    // One cp command line from the browser. Hand it to core0 and terminate it
    // so the CLI's line parser dispatches it, exactly as the ESP bridge does.
    for (size_t i = 0; i < length; ++i) {
      s_rx.push(payload[i]);
    }
    s_rx.push('\n');
  }
}

// Drain cp output coming back from core0 and broadcast each complete line to
// every connected browser. Mirrors the ESP companion's UART pump.
void pumpTx() {
  int c;
  while ((c = s_tx.pop()) >= 0) {
    if (c == '\n' || c == '\r') {
      if (s_line_len > 0) {
        s_line[s_line_len] = '\0';
        s_ws.broadcastTXT(s_line);
        s_line_len = 0;
      }
    } else if (s_line_len < kLineMax - 1) {
      s_line[s_line_len++] = static_cast<char>(c);
    }
  }
}

}  // namespace

Stream* cli_stream() {
  return &s_bridge;
}

void init() {
  // The whole radio and web stack lives on core1, so the 1 kHz flight loop on
  // core0 is never interrupted by WiFi, HTTP, or WebSocket work.
  char ssid[32];
  pico_unique_board_id_t bid;
  pico_get_unique_board_id(&bid);
  snprintf(ssid, sizeof(ssid), "%s%02X%02X", kApSsidPrefix,
           bid.id[6], bid.id[7]);

  WiFi.mode(WIFI_AP);
  s_up = WiFi.softAP(ssid, kApPassword);

  s_http.on("/", handleRoot);
  s_http.onNotFound(handleRoot);  // captive-portal style: any path -> the UI.
  s_http.begin();

  s_ws.begin();
  s_ws.onEvent(onWsEvent);

  // The CYW43439 link LED is drivable now the radio is initialised. It blinks
  // a slow "radio alive" beat, separate from the flight heartbeat the LED
  // driver runs on the external status LED (GP14) from core0.
  pinMode(LED_BUILTIN, OUTPUT);
}

void tick() {
  if (!s_up) {
    return;
  }
  s_http.handleClient();
  s_ws.loop();
  pumpTx();

  const uint32_t now = millis();
  if (now - s_led_last_ms >= kLinkBlinkMs) {
    s_led_last_ms = now;
    s_led_state   = !s_led_state;
    digitalWrite(LED_BUILTIN, s_led_state ? HIGH : LOW);
  }
}

}  // namespace cp::comms::onboard_wifi

// Arduino core1 entry points. The arduino-pico core provides weak empty
// defaults; these strong definitions claim core1 for the onboard WiFi
// companion. They live here rather than in the .ino so the central Config.h
// include resolves on the normal source path (the sketch root is a build
// copy, where a direct Config.h include would double-define).
void setup1() {
  cp::comms::onboard_wifi::init();
}

void loop1() {
  cp::comms::onboard_wifi::tick();
}

#endif  // ENABLE_ONBOARD_WIFI

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

#include "src/comms/OnboardWifi.h"

#include "src/Config.h"

#if ENABLE_ONBOARD_WIFI

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <pico/unique_id.h>

namespace cp::comms::onboard_wifi {

namespace {

// Access point. The SSID gets a per-board suffix from the chip ID so two
// aircraft on the same field do not collide. Change the password before use.
constexpr char     kApSsidPrefix[] = "CrowPilot-";
constexpr char     kApPassword[]   = "crowpilot";
constexpr uint16_t kHttpPort       = 80;
constexpr uint32_t kLinkBlinkMs    = 500;  // 1 Hz radio-LED heartbeat.

WebServer s_server(kHttpPort);
bool      s_up           = false;
uint32_t  s_led_last_ms  = 0;
bool      s_led_state    = false;

// Minimal status page. The full tuning UI and the cp WebSocket bridge are the
// next increment. This page confirms the access point is up and lets the loop
// jitter be measured on the bench before the bridge is wired in.
void handleRoot() {
  s_server.send(200, "text/html",
    "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>CrowPilot</title>"
    "<body style='background:#10141A;color:#e9ecf1;font-family:Inter,system-ui,sans-serif;margin:0;padding:24px'>"
    "<h1 style='color:#15B8A6;margin:0 0 8px'>CrowPilot</h1>"
    "<p>Onboard WiFi companion is up. The tuning UI and live telemetry are "
    "served over the cp WebSocket bridge in the next firmware increment.</p>"
    "</body>");
}

}  // namespace

void init() {
  // The whole radio stack lives on core1, so the 1 kHz flight loop on core0
  // is never interrupted by WiFi or HTTP work.
  char ssid[32];
  pico_unique_board_id_t bid;
  pico_get_unique_board_id(&bid);
  snprintf(ssid, sizeof(ssid), "%s%02X%02X", kApSsidPrefix,
           bid.id[6], bid.id[7]);

  WiFi.mode(WIFI_AP);
  s_up = WiFi.softAP(ssid, kApPassword);

  s_server.on("/", handleRoot);
  s_server.begin();

  // The CYW43439 link LED is drivable now the radio is initialised. It blinks
  // a slow "radio alive" beat, separate from the flight heartbeat that the LED
  // driver runs on the external status LED (GP14) from core0.
  pinMode(LED_BUILTIN, OUTPUT);
}

void tick() {
  if (!s_up) {
    return;
  }
  s_server.handleClient();

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

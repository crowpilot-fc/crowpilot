// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

// CrowPilot ESP32 wireless companion.
//
// A small ESP32-C3 board wired to the flight controller's companion UART.
// It raises a WiFi hotspot, serves a single-page web UI to a phone, and
// bridges the browser to the FC over the existing line-oriented `cp`
// protocol. The phone tunes parameters and watches live telemetry; the
// FC never depends on the companion and stays in control either way.
//
// Architecture:
//   browser  <--WebSocket :81-->  ESP  <--UART 115200-->  FC companion port
//   browser  <--HTTP :80-------->  ESP serves the web UI
// Every WebSocket text message is a `cp` command line; every UART line
// from the FC is forwarded back to the browser verbatim.
//
// Build (Arduino IDE or arduino-cli):
//   Board:    ESP32C3 Dev Module  (fqbn esp32:esp32:esp32c3)
//   Library:  WebSockets by Markus Sattler  (arduino-cli lib install WebSockets)
//
// Wiring to the FC (verify the ESP pin numbers against your board's
// silkscreen; any free GPIO works through the C3 GPIO matrix):
//   ESP FC_UART_RX  <--  FC PIN_COMPANION_TX
//   ESP FC_UART_TX  -->  FC PIN_COMPANION_RX
//   ESP 3V3 / GND   <->  FC 3V3 / GND  (common ground is required)

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

#include "web_ui.h"

// --- Configuration ---------------------------------------------------------

// WiFi access point. The SSID gets a per-board suffix from the MAC so two
// aircraft on the same field do not collide. Change the password before
// field use; WPA2 requires at least 8 characters.
static const char* kApSsidPrefix = "CrowPilot-";
static const char* kApPassword   = "crowpilot";

// UART to the flight controller's companion port. 115200 8N1, matching
// the FC's ENABLE_COMPANION_CLI baud.
constexpr int      kFcUartRx   = 20;
constexpr int      kFcUartTx   = 21;
constexpr uint32_t kFcUartBaud = 115200;

// Longest `cp` line from the FC (a telemetry line is the worst case).
constexpr size_t kLineMax = 256;

// --- State -----------------------------------------------------------------

WebServer       g_http(80);
WebSocketsServer g_ws(81);

char   g_line[kLineMax];
size_t g_line_len = 0;

// --- HTTP ------------------------------------------------------------------

void handleRoot() {
  g_http.send_P(200, "text/html", kWebUi);
}

// --- WebSocket -------------------------------------------------------------

void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_TEXT:
      // The browser sends one `cp` command line. Forward it to the FC and
      // terminate it so the FC's line parser dispatches it.
      Serial1.write(payload, length);
      Serial1.write('\n');
      break;
    case WStype_CONNECTED:
      Serial.printf("ws client %u connected\n", num);
      break;
    case WStype_DISCONNECTED:
      Serial.printf("ws client %u disconnected\n", num);
      break;
    default:
      break;
  }
}

// Drain the FC UART. Each complete line is broadcast to every connected
// browser. Lines longer than the buffer are truncated rather than dropped.
void pumpUart() {
  while (Serial1.available() > 0) {
    const int c = Serial1.read();
    if (c < 0) {
      break;
    }
    if (c == '\n' || c == '\r') {
      if (g_line_len > 0) {
        g_line[g_line_len] = '\0';
        g_ws.broadcastTXT(g_line);
        g_line_len = 0;
      }
    } else if (g_line_len < kLineMax - 1) {
      g_line[g_line_len++] = static_cast<char>(c);
    }
  }
}

// --- Arduino entry points --------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial1.begin(kFcUartBaud, SERIAL_8N1, kFcUartRx, kFcUartTx);

  uint8_t mac[6] = {0};
  WiFi.softAPmacAddress(mac);
  char ssid[32];
  snprintf(ssid, sizeof(ssid), "%s%02X%02X", kApSsidPrefix, mac[4], mac[5]);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, kApPassword);
  Serial.printf("AP \"%s\" up at %s\n", ssid,
                WiFi.softAPIP().toString().c_str());

  g_http.on("/", handleRoot);
  g_http.onNotFound(handleRoot);  // captive-portal style: any path -> UI
  g_http.begin();

  g_ws.begin();
  g_ws.onEvent(onWsEvent);
}

void loop() {
  g_http.handleClient();
  g_ws.loop();
  pumpUart();
}

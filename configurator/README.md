<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# CrowPilot Configurator

A browser-based setup tool for CrowPilot. Connect the flight controller
over USB, edit the runtime parameters in a form, and write them back to
the board.

It has four tabs: Parameters (the PID gain editor), Telemetry (live
attitude, receiver channels, arm and failsafe state, loop health), Log
(decode a `.BIN` telemetry log pulled from the SD card), and Firmware
(flash a `.uf2` to the board over USB).

There is also a **Mock device** button. It connects to an in-browser
stand-in that answers the same protocol, so the configurator can be
tried out and developed without any hardware.

## Requirements

- A Chromium-based browser (Chrome or Edge). The configurator uses the
  Web Serial API, which Firefox and Safari do not support.
- A CrowPilot board built with `ENABLE_CONFIG_CLI = 1` in `src/Config.h`
  (the default).

## Running it

Open `index.html` in Chrome or Edge. Either double-click the file, or
serve the folder with any static file server, for example:

```
python3 -m http.server --directory configurator
```

then visit the printed URL.

## Using it

1. Connect the board over USB.
2. Click **Connect** and pick the serial port.
3. The current parameters load into the form, grouped by regime.
4. Edit any value. Changed fields are highlighted.
5. **Write changes** sends the edits to the board (RAM only).
6. **Save to flash** persists them so they survive a reboot. This is
   rejected while the aircraft is armed.
7. **Reload from flash** and **Reset to defaults** restore values.
8. The **Telemetry** tab shows live attitude, receiver channels, arm and
   failsafe state, and loop period. Streaming starts automatically on
   connect.

The protocol log at the bottom shows the raw exchange. Telemetry lines
are not logged, since they arrive continuously.

## The Log tab

The **Log** tab decodes a binary telemetry log without a device
connection. Pick a `.BIN` file pulled from the SD card, or click **Load
sample log** to decode a generated sample. It shows:

- A summary: duration, loop-period health, armed fraction, failsafe
  events, gyro RMS, the dominant oscillation frequency per axis, and the
  flight-mode breakdown.
- Charts of the gyro and PID-output traces across the flight.
- A sampled record table.

The decoder and the Goertzel oscillation scan follow the 109-byte record
schema and analysis in `tools/log_analyzer/decode_features.py`.

## The Firmware tab

The **Firmware** tab flashes a `.uf2` to the board over USB, using the
RP2350 bootrom PICOBOOT protocol through WebUSB.

1. Put the board in BOOTSEL mode. While connected, click **Reboot to
   bootloader** (this sends `cp boot`). Otherwise hold the BOOTSEL
   button while plugging the board in.
2. Pick the `.uf2` file. The tab shows its size, address, and family.
3. Click **Flash** and select the board in the WebUSB prompt.

A failed flash is recoverable: an RP2350 with no valid image stays in
BOOTSEL, so the firmware can always be re-flashed by dragging the
`.uf2` onto the board's USB drive, the standard fallback.

## The serial protocol

The configurator speaks a line-oriented text protocol over the USB serial
port at 115200 baud. Every firmware reply is prefixed with `cp `.

| Command | Purpose |
|---|---|
| `cp` | Handshake. Replies `cp fw <version> params <count>`. |
| `cp list` | Dump every parameter, terminated by `cp end`. |
| `cp set <key> <value>` | Set one parameter. The value is clamped. |
| `cp save` | Persist the registry to flash. |
| `cp load` | Reload the registry from flash. |
| `cp defaults` | Reset the registry to compile-time defaults. |
| `cp stream on` / `cp stream off` | Start or stop live telemetry. |
| `cp boot` | Reboot into the BOOTSEL bootloader. Rejected while armed. |

While streaming is on, the board pushes a telemetry line about ten times
a second:

```
cp tlm <roll> <pitch> <yaw> <armed> <failsafe> <mode> <loop_us> <ch1..ch6> \
       <alt> <yaw_rate> <ax> <ay> <az> <arm> <stab> <trans> <alt_hold> \
       <vbat> <cells> <low>
```

The five instrument fields (altitude, yaw rate, and the three body
accelerations) follow the six channels. The four high role channels (arm,
stabilizer, transition, altitude-hold) follow those, so the live view can show
the safety switches that sit above the first six channels. The three battery
fields (pack voltage, cell count, low-voltage flag) follow those, and are zero
when the battery monitor is off. A reader should take the first eight fields as
the stable core and treat the later groups as optional, so an older firmware
that sends only the channels still parses.

Because it is plain text, you can also drive the board from any serial
terminal for debugging.

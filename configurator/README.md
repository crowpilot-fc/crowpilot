<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# CrowPilot Configurator

A browser-based setup tool for CrowPilot. Connect the flight controller
over USB, edit the runtime parameters in a form, and write them back to
the board.

It has a Parameters tab (the PID gain editor) and a Telemetry tab (live
attitude, receiver channels, arm and failsafe state, loop health). It
does not flash firmware. Flashing stays the standard RP2350 workflow
(hold BOOTSEL, drag the `.uf2` onto the mass-storage drive).

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

While streaming is on, the board pushes a telemetry line about ten times
a second:

```
cp tlm <roll> <pitch> <yaw> <armed> <failsafe> <mode> <loop_us> <ch1..ch6>
```

Because it is plain text, you can also drive the board from any serial
terminal for debugging.

<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Log analyzer

An LLM-aided tuning helper. It reduces a CrowPilot telemetry log to a
compact statistical feature set, then either prints a ready-to-paste
diagnosis prompt or sends it to an LLM backend directly.

This is a host-side tool. It does not run on the flight controller and has
no third-party dependencies (Python 3.8+ standard library only).

## Files

- `decode_features.py` - reduces a `.BIN` log to a feature dictionary:
  loop-period health, per-axis gyro RMS, dominant oscillation frequency
  (via a Goertzel band scan), controller and actuator saturation counts,
  and a flight-mode breakdown.
- `analyze.py` - assembles the diagnosis prompt and optionally calls an
  LLM backend.
- `prompt_template.md` - the instruction text given to the model.

## Usage

Print the diagnosis prompt for manual use (no API key needed):

```bash
python tools/log_analyzer/analyze.py LOG0001.BIN
```

Include the PID gain set from `Config.h` as flight context:

```bash
python tools/log_analyzer/analyze.py LOG0001.BIN --config src/Config.h
```

Send the prompt to the Anthropic API and get the diagnosis directly:

```bash
export ANTHROPIC_API_KEY=sk-ant-...
python tools/log_analyzer/analyze.py LOG0001.BIN --config src/Config.h \
    --backend anthropic --model <model-id>
```

The model id can also come from the `ANTHROPIC_MODEL` environment variable.
Write the result to a file with `--out report.md`.

Inspect the raw feature set on its own:

```bash
python tools/log_analyzer/decode_features.py LOG0001.BIN
```

## How the oscillation scan works

A tuning oscillation shows up as a periodic signal on a gyro axis.
`decode_features.py` removes the mean from each axis and scans 1 to 40 Hz
with the Goertzel algorithm, reporting the strongest frequency and its
amplitude. This stays in the Python standard library (no FFT package) and
is fast enough for logs of any practical length.

A fast oscillation (above ~10 Hz) usually points at too much P or too
little D. A slow wobble (a few Hz) points at too little D. Slow drift with
no clear peak points at too little I. The analyzer's prompt explains this
to the model so the recommendation follows CrowPilot's D-then-P-then-I
tuning order.

## Scope

The analyzer reads the binary log format documented in
`internal-docs/TELEMETRY_FORMAT.md`. The companion firmware integration
(streaming params and logs over the ESP link) is a separate, later piece
of work and is not part of this tool.

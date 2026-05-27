#!/usr/bin/env bash
# Build the CrowPilot firmware for the WeAct RP2350A (generic RP2350).
#
# Two things the stock Arduino toolchain does not do on its own:
#   1. CrowPilot uses repo-root-relative includes (#include "src/...").
#      The sketch root is not on the compiler include path by default,
#      so it is added explicitly via build.extra_flags.
#   2. The SBUS receiver and the DShot motor output each need a PIO
#      program. sbus_rx.pio and dshot.pio are assembled to their .pio.h
#      headers here (the results are also committed, so an Arduino IDE
#      build without this script still works).
#
# Pass extra arguments straight through, e.g. ./build.sh --upload -p <port>.
# If your board has more than 2 MB flash, append a flash option to FQBN,
# e.g. rp2040:rp2040:generic_rp2350:flash=4194304_0.

set -euo pipefail

SKETCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FQBN="rp2040:rp2040:generic_rp2350"

# Assemble the PIO programs if pioasm from the rp2040 core is present.
PIOASM="$(find "$HOME/Library/Arduino15/packages/rp2040/tools/pqt-pioasm" \
  -name pioasm -type f 2>/dev/null | head -1 || true)"
if [ -n "${PIOASM}" ]; then
  "${PIOASM}" -o c-sdk "${SKETCH_DIR}/sbus_rx.pio" "${SKETCH_DIR}/sbus_rx.pio.h"
  "${PIOASM}" -o c-sdk "${SKETCH_DIR}/dshot.pio"   "${SKETCH_DIR}/dshot.pio.h"
else
  echo "note: pioasm not found, using the committed .pio.h headers" >&2
fi

arduino-cli compile \
  --fqbn "${FQBN}" \
  --build-property "build.extra_flags=-I{build.source.path}" \
  "${SKETCH_DIR}" "$@"

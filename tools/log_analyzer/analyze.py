#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Nitin Kumar
#
# CrowPilot log analyzer.
#
# Reduces a telemetry log to statistical features (via decode_features.py),
# assembles a tuning-diagnosis prompt, and optionally sends it to an LLM
# backend. With no backend it prints the assembled prompt so it can be
# pasted into any chat model; the tool is fully usable offline.
#
# Usage:
#   python tools/log_analyzer/analyze.py LOG0001.BIN
#   python tools/log_analyzer/analyze.py LOG0001.BIN --config src/Config.h
#   python tools/log_analyzer/analyze.py LOG0001.BIN --backend anthropic \
#       --model <model-id>
#
# The Anthropic backend reads the API key from the ANTHROPIC_API_KEY
# environment variable and the model id from --model or ANTHROPIC_MODEL.
#
# Python 3.8+. No third-party dependencies.

import argparse
import json
import os
import re
import sys
import urllib.error
import urllib.request

from decode_features import extract_features

TEMPLATE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "prompt_template.md")

# Config.h gain constants the analyzer reports as flight context.
GAIN_PREFIXES = ("Kp_", "Kd_", "Ki_")


def load_template():
    with open(TEMPLATE_PATH, "r", encoding="utf-8") as fh:
        return fh.read()


def parse_gains(config_path):
    """Pull the PID gain constants out of a Config.h file."""
    gains = {}
    pattern = re.compile(
        r"constexpr\s+float\s+(\w+)\s*=\s*([-+0-9.eEf]+)\s*;"
    )
    with open(config_path, "r", encoding="utf-8") as fh:
        for line in fh:
            m = pattern.search(line)
            if not m:
                continue
            name, value = m.group(1), m.group(2)
            if name.startswith(GAIN_PREFIXES):
                gains[name] = value.rstrip("f")
    return gains


def build_prompt(features, gains):
    """Assemble the full prompt: template instructions plus log data."""
    template = load_template()
    parts = [template, "\n\n## Log data\n"]

    parts.append("\n### Feature summary\n\n```json\n")
    parts.append(json.dumps(features, indent=2))
    parts.append("\n```\n")

    parts.append("\n### PID gain set\n\n")
    if gains:
        parts.append("```\n")
        for name in sorted(gains):
            parts.append(f"{name} = {gains[name]}\n")
        parts.append("```\n")
    else:
        parts.append("Gain set not provided. Base the diagnosis on the "
                      "feature summary alone and note where knowing the "
                      "gains would sharpen the recommendation.\n")
    return "".join(parts)


def call_anthropic(prompt, model):
    """Send the prompt to the Anthropic Messages API. Returns the reply text."""
    api_key = os.environ.get("ANTHROPIC_API_KEY")
    if not api_key:
        raise SystemExit(
            "error: --backend anthropic needs the ANTHROPIC_API_KEY "
            "environment variable."
        )
    if not model:
        raise SystemExit(
            "error: --backend anthropic needs a model id via --model or "
            "the ANTHROPIC_MODEL environment variable."
        )

    body = json.dumps({
        "model": model,
        "max_tokens": 2048,
        "messages": [{"role": "user", "content": prompt}],
    }).encode("utf-8")

    request = urllib.request.Request(
        "https://api.anthropic.com/v1/messages",
        data=body,
        method="POST",
        headers={
            "x-api-key": api_key,
            "anthropic-version": "2023-06-01",
            "content-type": "application/json",
        },
    )
    try:
        with urllib.request.urlopen(request, timeout=120) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise SystemExit(f"error: Anthropic API HTTP {exc.code}: {detail}")
    except urllib.error.URLError as exc:
        raise SystemExit(f"error: Anthropic API request failed: {exc.reason}")

    blocks = payload.get("content", [])
    text = "".join(b.get("text", "") for b in blocks if b.get("type") == "text")
    if not text:
        raise SystemExit("error: Anthropic API returned no text content.")
    return text


def main():
    parser = argparse.ArgumentParser(
        description="Analyze a CrowPilot telemetry log and produce a tuning "
                    "diagnosis."
    )
    parser.add_argument("log", help="path to a .BIN telemetry log")
    parser.add_argument("--config", help="path to Config.h, to include the "
                                         "PID gain set as context")
    parser.add_argument("--backend", choices=("none", "anthropic"),
                        default="none",
                        help="LLM backend. 'none' prints the prompt for "
                             "manual use (default).")
    parser.add_argument("--model", default=os.environ.get("ANTHROPIC_MODEL"),
                        help="model id for the chosen backend")
    parser.add_argument("--out", help="write the result to this file instead "
                                      "of stdout")
    args = parser.parse_args()

    features = extract_features(args.log)
    gains = parse_gains(args.config) if args.config else {}
    prompt = build_prompt(features, gains)

    if args.backend == "anthropic":
        result = call_anthropic(prompt, args.model)
    else:
        result = (
            "# CrowPilot log analysis prompt\n\n"
            "No LLM backend selected. Paste everything below into a chat "
            "model to get the tuning diagnosis, or rerun with "
            "`--backend anthropic`.\n\n"
            "---\n\n" + prompt
        )

    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            fh.write(result)
            if not result.endswith("\n"):
                fh.write("\n")
        print(f"wrote analysis to {args.out}", file=sys.stderr)
    else:
        sys.stdout.write(result)
        if not result.endswith("\n"):
            sys.stdout.write("\n")


if __name__ == "__main__":
    main()

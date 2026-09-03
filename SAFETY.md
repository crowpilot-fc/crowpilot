<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Safety Guide

Aircraft running CrowPilot can cause serious injury or death. This document covers the minimum operating practices for safe use of CrowPilot-powered aircraft. **Read it in full before powering any motor.**

This guide is not a substitute for hands-on training, manufacturer instructions, common sense, or applicable law. It is a minimum, not a maximum.

For the full legal disclaimer of warranty and liability, see [DISCLAIMER.md](DISCLAIMER.md). By using CrowPilot you accept all responsibility for safe operation.

## Before any power-on

Verify every item before connecting the battery:

- All propellers are removed.
- The aircraft is on a stable bench. No clutter within propeller-strike radius.
- Wiring is double-checked for shorts, reversed polarity, and loose connections.
- The transmitter is on, bound to the receiver, and at throttle minimum.
- No one is within propeller-strike radius if propellers are about to be installed.
- A working battery monitor or low-voltage alarm is in place.
- A fire extinguisher rated for electrical and lithium fires is within reach.

## Bench testing without propellers

The first hours of any new build are spent with propellers removed. Always.

- Confirm motor wiring by spinning each motor at low throttle and verifying direction by hand or with a piece of tape on the shaft.
- Verify arm and disarm logic. The aircraft must arm only when intended and must disarm immediately on a throttle-cut switch.
- Verify failsafe behavior by turning the transmitter off during a controlled bench run. The motors must stop or follow the documented failsafe response.
- Confirm that servo travel is correct on every channel and that no servo binds or stalls.
- Run the aircraft for several minutes on the bench and check ESC and motor temperatures.
- Confirm telemetry is recording on the SD card and that log files open correctly on the ground.

## First motor spin with propellers installed

Propellers spin at thousands of RPM. A failed prop or a finger in the wrong place ends in stitches or worse.

- Wear eye protection. Always.
- Tether the aircraft. Tie it down with cordage or zip-ties such that it cannot tip over, fly away, or strike anyone if it accelerates unexpectedly.
- Keep all bystanders behind a physical barrier or out of the room.
- Stand off-axis from the propellers. A blade failure projects fragments along the rotational axis at high speed.
- Hold the transmitter with throttle at minimum. Verify the throttle-cut switch is reachable without moving your hands.
- Confirm throttle cut works by triggering it once before any further test.
- Spool up motors gradually. No full-throttle test without the aircraft secured.

## First free flight

Free flight only happens after all bench testing has passed and the aircraft has been tethered for spin-up.

- Choose an empty area with no people, animals, vehicles, power lines, or sensitive property within at least 50 metres in any direction. More for larger aircraft.
- Verify the area complies with local drone regulations (line of sight, altitude limits, no-fly zones, airspace permissions, privacy law, time-of-day restrictions).
- Use a fresh, well-charged, balanced battery. Confirm cell voltage with a meter, not the aircraft.
- Confirm failsafe behavior is configured and tested.
- Do a final pre-flight check. Control surfaces move correctly, motors spin up and down on command, RC link is solid at distance.
- Keep everyone except the pilot and a trained launch assistant well clear of the aircraft during take-off. Do not attempt a free flight until restrained ground runs and control checks have passed.
- Keep a finger on the throttle-cut switch at all times during the entire flight.
- Keep visual line of sight at all times unless you hold appropriate certifications and permissions for beyond-line-of-sight flight.
- If anything feels wrong, cut throttle. Do not try to "recover" a suspicious flight. A controlled crash from low altitude is safer than a fly-away or an erratic recovery attempt.

## Battery handling

LiPo and Li-ion batteries are the most dangerous component of any small UAV.

- Never charge a damaged, puffed, hot, or visibly distressed battery.
- Charge inside a fireproof LiPo bag or a metal container. Charge in a location where a fire would not damage property or harm people.
- Use only chargers rated for the battery's chemistry and cell count. Set the charge rate per the manufacturer's specification, never higher.
- Store batteries at storage charge (typically 3.7 V to 3.85 V per cell) when not in use for more than a few days.
- Never leave charging batteries unattended.
- Inspect every battery before every use. Check for puff, dents, exposed wire, damaged balance leads.
- Dispose of damaged batteries at a proper recycling facility, not in household waste.

## Crash response

If the aircraft crashes:

- Cut power immediately. Disconnect the battery as soon as it is safe to approach.
- Wait at least 10 minutes before further handling. Crashed lithium batteries can ignite with a delay after impact.
- If the battery shows any sign of smoke, swelling, or heat, move to a fireproof area and let it cool. Do not attempt to recover the airframe until the battery is confirmed safe.
- Inspect the airframe, motors, ESCs, wiring, and FC board for damage before any further test.
- A crashed battery is a hazard even if it appears fine. When in doubt, dispose of it safely.
- Investigate the cause of the crash before re-flying. Pull telemetry logs, check for loose connections, check for obvious damage. A second crash from the same unfixed cause is your fault, not the aircraft's.

## Legal compliance

Drone regulations vary by country and change frequently. You are solely responsible for compliance with all applicable laws, including but not limited to:

- Registration and operator certification requirements.
- No-fly zones (airports, military areas, government buildings, sensitive infrastructure, national parks, populated areas).
- Maximum altitude (often 120 metres / 400 feet, varies by jurisdiction).
- Visual-line-of-sight requirements.
- Time-of-day restrictions.
- Privacy law (filming, recording, overflight of private property).
- Insurance requirements (often mandatory above a weight or use threshold).

Check your country's civil aviation authority. India: DGCA and the Digital Sky platform. United States: FAA. European Union: EASA. United Kingdom: CAA. Australia: CASA. Other jurisdictions have equivalent bodies. Do not assume any of these apply unchanged outside their stated region. Regulations change. Verify before every flight in unfamiliar territory.

## When this guide is not enough

This document covers minimum practices. It is not a substitute for:

- Local aero-club training and mentorship.
- Manufacturer instructions for your specific motors, ESCs, batteries, transmitter, and airframe.
- Pilot certification programs in jurisdictions that require them.
- Real-time judgment in conditions this guide cannot anticipate.
- Common sense.

If you are new to fixed-wing RC flight, train with an experienced pilot and spend time in a fixed-wing simulator before attempting CrowPilot on real hardware. Real-stick time with the same transmitter and control layout translates into safer first flights.

## Disclaimer

See [DISCLAIMER.md](DISCLAIMER.md) for the full legal disclaimer of warranty and limitation of liability. The disclaimer applies in full to all use of CrowPilot.

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// In-browser stand-in for a CrowPilot board. Mirrors the cp protocol in
// src/cli/Cli.cpp and the parameter registry in src/params/params.def so
// the configurator can be exercised end to end without hardware. It is
// not a flight simulator: it only answers protocol commands.

'use strict';

function createMockDevice() {
  const FW = '1.0.0-dev';

  // [key, default, min, max], in params.def order. Defaults and bounds
  // mirror src/Config.h and params.def.
  const TABLE = [
    ['kp_roll_hover',  0.030,  0, 4],
    ['kd_roll_hover',  0.004,  0, 1],
    ['kp_pitch_hover', 0.030,  0, 4],
    ['kd_pitch_hover', 0.004,  0, 1],
    ['kp_yaw_hover',   0.008,  0, 4],
    ['kd_yaw_hover',   0.002,  0, 1],
    ['kp_roll_ff',     0.025,  0, 4],
    ['kd_roll_ff',     0.003,  0, 1],
    ['kp_pitch_ff',    0.025,  0, 4],
    ['kd_pitch_ff',    0.003,  0, 1],
    ['kp_yaw_ff',      0.006,  0, 4],
    ['kd_yaw_ff',      0.0015, 0, 1],
    ['ki_roll',        0.0,    0, 1],
    ['ki_pitch',       0.0,    0, 1],
    ['ki_yaw',         0.0,    0, 1],
  ];

  const params = TABLE.map(([key, def, min, max]) => ({
    key, value: def, def, min, max, persist: true,
  }));

  // Simulated flash store: the values the board would reload on boot.
  let flash = params.map((p) => p.value);

  const fmt = (x) => x.toFixed(5);

  function paramLine(p, idx) {
    return ['cp param', idx, p.key, fmt(p.value), fmt(p.def),
            fmt(p.min), fmt(p.max), p.persist ? 1 : 0].join(' ');
  }

  // Take one command line, return the array of response lines, exactly
  // as the firmware cp handler would emit them.
  function handle(line) {
    const t = line.trim().split(/\s+/);
    if (t[0] !== 'cp') {
      return ['cp err badcmd'];
    }
    const verb = t[1];
    if (verb === undefined) {
      return ['cp fw ' + FW + ' params ' + params.length];
    }
    if (verb === 'list') {
      return params.map(paramLine).concat(['cp end']);
    }
    if (verb === 'set') {
      const key = t[2];
      const valstr = t[3];
      if (key === undefined || valstr === undefined) {
        return ['cp err noargs'];
      }
      const p = params.find((q) => q.key === key);
      if (!p) {
        return ['cp err nokey'];
      }
      const v = parseFloat(valstr);
      if (Number.isNaN(v)) {
        return ['cp err badval'];
      }
      p.value = Math.min(p.max, Math.max(p.min, v));
      return ['cp ok set ' + key + ' ' + fmt(p.value)];
    }
    if (verb === 'save') {
      flash = params.map((p) => p.value);
      return ['cp ok saved'];
    }
    if (verb === 'load') {
      params.forEach((p, i) => { p.value = flash[i]; });
      return ['cp ok loaded'];
    }
    if (verb === 'defaults') {
      params.forEach((p) => { p.value = p.def; });
      return ['cp ok defaults'];
    }
    if (verb === 'stream') {
      const arg = t[2];
      if (arg === undefined) {
        return ['cp err noargs'];
      }
      if (arg === 'on') {
        return ['cp ok stream on'];
      }
      if (arg === 'off') {
        return ['cp ok stream off'];
      }
      return ['cp err badval'];
    }
    return ['cp err badcmd'];
  }

  // One synthetic telemetry line in the firmware cp tlm format:
  //   roll pitch yaw armed failsafe mode loop_us ch1..ch6
  //   alt yaw_rate ax ay az arm stab trans alt_hold
  // The values gently animate so the Telemetry tab visibly updates.
  let tlmPhase = 0;
  function telemetry() {
    tlmPhase += 0.12;
    const roll = (Math.sin(tlmPhase) * 8).toFixed(1);
    const pitch = (Math.sin(tlmPhase * 0.7) * 5).toFixed(1);
    const yaw = ((tlmPhase * 9) % 360 - 180).toFixed(1);
    const loop = 1000 + Math.floor(Math.random() * 7);
    const ch = [
      1500 + Math.round(Math.sin(tlmPhase) * 60),
      1500 + Math.round(Math.sin(tlmPhase * 1.3) * 220),
      1500 + Math.round(Math.cos(tlmPhase * 0.9) * 220),
      1500 + Math.round(Math.sin(tlmPhase * 0.5) * 120),
      1000,
      1900,
    ];
    // Instrument fields: altitude, yaw rate, and the three body accelerations.
    const alt = (Math.sin(tlmPhase * 0.3) * 2 + 2).toFixed(1);
    const gz = (Math.sin(tlmPhase * 1.1) * 20).toFixed(1);
    const ax = (Math.sin(tlmPhase) * 0.05).toFixed(2);
    const ay = (Math.cos(tlmPhase) * 0.05).toFixed(2);
    const az = (1 + Math.sin(tlmPhase * 0.4) * 0.05).toFixed(2);
    // High role channels: arm held armed, stabilizer toggling, transition at
    // the hover end, altitude-hold off.
    const arm = 1000;
    const stab = Math.sin(tlmPhase * 0.2) > 0 ? 1000 : 2000;
    const trans = 2000;
    const altHold = 1000;
    // Battery: a slowly sagging 4S pack, voltage, cell count, low flag.
    const vbat = (15.6 + Math.sin(tlmPhase * 0.15) * 0.3).toFixed(2);
    const cells = 4;
    const low = parseFloat(vbat) / cells < 3.5 ? 1 : 0;
    return 'cp tlm ' + roll + ' ' + pitch + ' ' + yaw +
           ' 0 0 hover ' + loop + ' ' + ch.join(' ') +
           ' ' + alt + ' ' + gz + ' ' + ax + ' ' + ay + ' ' + az +
           ' ' + arm + ' ' + stab + ' ' + trans + ' ' + altHold +
           ' ' + vbat + ' ' + cells + ' ' + low;
  }

  return { handle, telemetry };
}

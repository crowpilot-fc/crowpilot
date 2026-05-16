// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// Telemetry log decoder for the configurator. Decodes the 109-byte
// binary record the firmware writes to the SD card, the same schema
// tools/log_analyzer/decode_features.py decodes. Used by the Log tab to
// load and summarise a .BIN log in the browser.

'use strict';

const LOG_RECORD_SIZE = 109;
const LOG_SCHEMA_V1 = 0x01;
const LOG_GYRO_LSB_PER_DPS = 16.4;
const LOG_FLAG_ARMED = 0x01;
const LOG_FLAG_FAILSAFE = 0x02;
const LOG_LOOP_OVERRUN_US = 1100;
const LOG_MODE_NAMES = ['hover', 'forward', 'transitioning'];

function logModeName(m) {
  return LOG_MODE_NAMES[m] !== undefined ? LOG_MODE_NAMES[m] : '?';
}

// Decode a .BIN buffer into an array of record objects. Throws on an
// empty buffer or an unsupported schema version. Field byte offsets
// follow the record layout in decode_features.py.
function decodeLog(buffer) {
  if (buffer.byteLength < LOG_RECORD_SIZE) {
    throw new Error('file is smaller than one 109-byte record');
  }
  const view = new DataView(buffer);
  const schema = view.getUint8(0);
  if (schema !== LOG_SCHEMA_V1) {
    throw new Error('unsupported schema version 0x' +
                    schema.toString(16).padStart(2, '0'));
  }
  const count = Math.floor(buffer.byteLength / LOG_RECORD_SIZE);
  const records = [];
  for (let r = 0; r < count; r++) {
    const o = r * LOG_RECORD_SIZE;
    records.push({
      t_us: Number(view.getBigUint64(o + 1, true)),
      loop_us: view.getUint32(o + 9, true),
      gyro: [
        view.getInt16(o + 13, true) / LOG_GYRO_LSB_PER_DPS,
        view.getInt16(o + 15, true) / LOG_GYRO_LSB_PER_DPS,
        view.getInt16(o + 17, true) / LOG_GYRO_LSB_PER_DPS,
      ],
      fader: view.getFloat32(o + 91, true),
      motor_us: [
        view.getUint16(o + 95, true),
        view.getUint16(o + 97, true),
      ],
      pid: [
        view.getInt8(o + 103) / 127,
        view.getInt8(o + 104) / 127,
        view.getInt8(o + 105) / 127,
      ],
      mode: view.getUint8(o + 107),
      flags: view.getUint8(o + 108),
    });
  }
  return records;
}

// Reduce decoded records to the summary the Log tab displays.
function summarizeLog(records) {
  const n = records.length;
  const durationS =
      n > 1 ? (records[n - 1].t_us - records[0].t_us) / 1e6 : 0;

  let loopSum = 0;
  let loopMax = 0;
  let overruns = 0;
  let armed = 0;
  let failsafeEvents = 0;
  let prevFs = false;
  const modeCounts = [0, 0, 0];
  const sq = [0, 0, 0];

  for (const rec of records) {
    loopSum += rec.loop_us;
    if (rec.loop_us > loopMax) {
      loopMax = rec.loop_us;
    }
    if (rec.loop_us > LOG_LOOP_OVERRUN_US) {
      overruns++;
    }
    if (rec.flags & LOG_FLAG_ARMED) {
      armed++;
    }
    const fs = (rec.flags & LOG_FLAG_FAILSAFE) !== 0;
    if (fs && !prevFs) {
      failsafeEvents++;
    }
    prevFs = fs;
    if (modeCounts[rec.mode] !== undefined) {
      modeCounts[rec.mode]++;
    }
    for (let a = 0; a < 3; a++) {
      sq[a] += rec.gyro[a] * rec.gyro[a];
    }
  }

  return {
    records: n,
    durationS: durationS,
    loopMeanUs: n ? loopSum / n : 0,
    loopMaxUs: loopMax,
    overruns: overruns,
    armedFraction: n ? armed / n : 0,
    failsafeEvents: failsafeEvents,
    gyroRms: sq.map((s) => (n ? Math.sqrt(s / n) : 0)),
    modeCounts: modeCounts,
  };
}

// Build a synthetic .BIN buffer so the Log tab can be tried without a
// real flight log. Not flight data: just enough structure to exercise
// the decoder and the summary. Six seconds at 100 Hz, armed through the
// middle, with a brief failsafe blip and the occasional loop overrun.
function makeSampleLog() {
  const count = 600;
  const buffer = new ArrayBuffer(count * LOG_RECORD_SIZE);
  const view = new DataView(buffer);
  for (let r = 0; r < count; r++) {
    const o = r * LOG_RECORD_SIZE;
    const ph = r * 0.05;
    view.setUint8(o, LOG_SCHEMA_V1);
    view.setBigUint64(o + 1, BigInt(r * 10000), true);
    const loop =
        1000 + (r % 137 === 0 ? 250 : Math.floor(Math.random() * 20));
    view.setUint32(o + 9, loop, true);
    view.setInt16(o + 13,
        Math.round(Math.sin(ph) * 12 * LOG_GYRO_LSB_PER_DPS), true);
    view.setInt16(o + 15,
        Math.round(Math.sin(ph * 1.3) * 8 * LOG_GYRO_LSB_PER_DPS), true);
    view.setInt16(o + 17,
        Math.round(Math.sin(ph * 0.7) * 4 * LOG_GYRO_LSB_PER_DPS), true);
    view.setFloat32(o + 91, 1.0, true);
    view.setUint16(o + 95, 1500, true);
    view.setUint16(o + 97, 1500, true);
    view.setInt8(o + 103, Math.round(Math.sin(ph) * 30));
    view.setInt8(o + 104, Math.round(Math.sin(ph * 1.3) * 20));
    view.setInt8(o + 105, Math.round(Math.sin(ph * 0.7) * 10));
    view.setUint8(o + 107, 0);
    let flags = 0;
    if (r > 100 && r < 540) {
      flags |= LOG_FLAG_ARMED;
    }
    if (r > 500 && r < 515) {
      flags |= LOG_FLAG_FAILSAFE;
    }
    view.setUint8(o + 108, flags);
  }
  return buffer;
}

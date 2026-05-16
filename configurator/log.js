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

// Oscillation scan band, in Hz. Matches decode_features.py: tuning
// oscillation on a small airframe sits between a slow wobble and a fast
// buzz, so 1 to 40 Hz at a fine step covers it without a full FFT.
const OSC_SCAN_LO_HZ = 1.0;
const OSC_SCAN_HI_HZ = 40.0;
const OSC_SCAN_STEP_HZ = 0.5;

// Single-frequency power via the Goertzel algorithm.
function goertzelPower(samples, freqHz, sampleRateHz) {
  const omega = (2 * Math.PI * freqHz) / sampleRateHz;
  const coeff = 2 * Math.cos(omega);
  let sPrev = 0;
  let sPrev2 = 0;
  for (let i = 0; i < samples.length; i++) {
    const s = samples[i] + coeff * sPrev - sPrev2;
    sPrev2 = sPrev;
    sPrev = s;
  }
  return sPrev2 * sPrev2 + sPrev * sPrev - coeff * sPrev * sPrev2;
}

// Scan the oscillation band, return {freqHz, amplitudeDps} for the
// strongest tone. A detrend removes the DC term first.
function dominantOscillation(samples, sampleRateHz) {
  if (samples.length < 32) {
    return { freqHz: 0, amplitudeDps: 0 };
  }
  let mean = 0;
  for (const x of samples) {
    mean += x;
  }
  mean /= samples.length;
  const detrended = samples.map((x) => x - mean);
  let bestFreq = 0;
  let bestPower = 0;
  for (let f = OSC_SCAN_LO_HZ; f <= OSC_SCAN_HI_HZ; f += OSC_SCAN_STEP_HZ) {
    if (f < sampleRateHz / 2) {
      const p = goertzelPower(detrended, f, sampleRateHz);
      if (p > bestPower) {
        bestPower = p;
        bestFreq = f;
      }
    }
  }
  const amplitudeDps = (2 * Math.sqrt(bestPower)) / samples.length;
  return { freqHz: bestFreq, amplitudeDps: amplitudeDps };
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
  const armedGyro = [[], [], []];
  let armedFirstT = 0;
  let armedLastT = 0;

  for (const rec of records) {
    loopSum += rec.loop_us;
    if (rec.loop_us > loopMax) {
      loopMax = rec.loop_us;
    }
    if (rec.loop_us > LOG_LOOP_OVERRUN_US) {
      overruns++;
    }
    const fs = (rec.flags & LOG_FLAG_FAILSAFE) !== 0;
    if (fs && !prevFs) {
      failsafeEvents++;
    }
    prevFs = fs;
    if (modeCounts[rec.mode] !== undefined) {
      modeCounts[rec.mode]++;
    }
    if (rec.flags & LOG_FLAG_ARMED) {
      if (armed === 0) {
        armedFirstT = rec.t_us;
      }
      armedLastT = rec.t_us;
      armed++;
      for (let a = 0; a < 3; a++) {
        armedGyro[a].push(rec.gyro[a]);
      }
    }
  }

  // Gyro RMS and the oscillation scan are taken over the armed segment:
  // idle motors-off records would only dilute them.
  let sampleRateHz = 100;
  if (armed > 1 && armedLastT > armedFirstT) {
    sampleRateHz = (armed - 1) / ((armedLastT - armedFirstT) / 1e6);
  }
  const rms = (xs) => {
    if (xs.length === 0) {
      return 0;
    }
    let s = 0;
    for (const x of xs) {
      s += x * x;
    }
    return Math.sqrt(s / xs.length);
  };

  return {
    records: n,
    durationS: durationS,
    loopMeanUs: n ? loopSum / n : 0,
    loopMaxUs: loopMax,
    overruns: overruns,
    armedFraction: n ? armed / n : 0,
    failsafeEvents: failsafeEvents,
    gyroRms: armedGyro.map(rms),
    oscillation: armedGyro.map((axis) =>
        dominantOscillation(axis, sampleRateHz)),
    sampleRateHz: sampleRateHz,
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
    // Roll carries a 17 Hz tone on top of the slow sweep, so the
    // oscillation scan has something in-band to find.
    view.setInt16(o + 13, Math.round(
        (Math.sin(ph) * 12 + Math.sin(r * 2 * Math.PI * 17 / 100) * 5) *
        LOG_GYRO_LSB_PER_DPS), true);
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

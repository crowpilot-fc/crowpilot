// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// CrowPilot Configurator. Talks the line-oriented "cp" text protocol to
// the flight controller over the Web Serial API: edits the runtime
// parameter registry, shows live telemetry, and decodes binary logs. A
// mock device stands in for hardware.

'use strict';

const BAUD = 115200;
const CMD_TIMEOUT_MS = 2500;

const state = {
  transport: null,  // {send(text), close()} -- serial or mock
  mock: false,
  connected: false,
  params: [],       // [{idx, key, value, def, min, max, persist}]
  pending: null,    // {isList, lines, resolve, reject, timer}
  firmwareUf2: null,  // parsed .uf2 staged for flashing
};

const els = {
  connect: document.getElementById('connect'),
  mock: document.getElementById('mock'),
  status: document.getElementById('status'),
  unsupported: document.getElementById('unsupported'),
  toolbar: document.getElementById('toolbar'),
  form: document.getElementById('form'),
  log: document.getElementById('log'),
  write: document.getElementById('write'),
  save: document.getElementById('save'),
  reload: document.getElementById('reload'),
  defaults: document.getElementById('defaults'),
  tabParams: document.getElementById('tab-params'),
  tabTelemetry: document.getElementById('tab-telemetry'),
  tabLog: document.getElementById('tab-log'),
  panelParams: document.getElementById('panel-params'),
  panelTelemetry: document.getElementById('panel-telemetry'),
  panelLog: document.getElementById('panel-log'),
  tRoll: document.getElementById('t-roll'),
  tPitch: document.getElementById('t-pitch'),
  tYaw: document.getElementById('t-yaw'),
  tArmed: document.getElementById('t-armed'),
  tFailsafe: document.getElementById('t-failsafe'),
  tMode: document.getElementById('t-mode'),
  tLoop: document.getElementById('t-loop'),
  tChannels: document.getElementById('t-channels'),
  logFile: document.getElementById('log-file'),
  logSample: document.getElementById('log-sample'),
  logError: document.getElementById('log-error'),
  logSummarySection: document.getElementById('log-summary-section'),
  logSummary: document.getElementById('log-summary'),
  logTableSection: document.getElementById('log-table-section'),
  logTable: document.getElementById('log-table'),
  logChartsSection: document.getElementById('log-charts-section'),
  logChartGyro: document.getElementById('log-chart-gyro'),
  logChartPid: document.getElementById('log-chart-pid'),
  logChartLegend: document.getElementById('log-chart-legend'),
  tabFirmware: document.getElementById('tab-firmware'),
  panelFirmware: document.getElementById('panel-firmware'),
  fwBoot: document.getElementById('fw-boot'),
  fwFile: document.getElementById('fw-file'),
  fwFlash: document.getElementById('fw-flash'),
  fwInfo: document.getElementById('fw-info'),
  fwError: document.getElementById('fw-error'),
  fwProgressSection: document.getElementById('fw-progress-section'),
  fwStatus: document.getElementById('fw-status'),
  fwBar: document.getElementById('fw-bar'),
};

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------

function log(msg, cls) {
  const line = document.createElement('span');
  line.textContent = msg + '\n';
  if (cls) {
    line.className = cls;
  }
  els.log.appendChild(line);
  els.log.scrollTop = els.log.scrollHeight;
}

// ---------------------------------------------------------------------------
// Serial connection
// ---------------------------------------------------------------------------

async function connectSerial() {
  try {
    state.transport = await makeSerialTransport(() => setTimeout(disconnect, 0));
    state.mock = false;
    await onConnected();
  } catch (err) {
    log('connect failed: ' + err.message, 'err');
    await disconnect();
  }
}

async function connectMock() {
  try {
    state.transport = makeMockTransport();
    state.mock = true;
    await onConnected();
  } catch (err) {
    log('mock connect failed: ' + err.message, 'err');
    await disconnect();
  }
}

async function onConnected() {
  state.connected = true;
  setStatus('Connecting...', 'on');
  syncConnButtons();
  await handshake();
  await refreshParams();
  setToolbarEnabled(true);
  buildChannelRows();
  els.fwBoot.disabled = false;
  showTab('params');
  // Start the live telemetry stream so the Telemetry tab has data.
  await sendCommand('cp stream on', false);
}

async function disconnect() {
  state.connected = false;
  setToolbarEnabled(false);
  els.fwBoot.disabled = true;
  els.toolbar.classList.add('hidden');
  els.form.innerHTML = '';
  if (state.transport) {
    try { await state.transport.close(); } catch (e) { /* ignore */ }
    state.transport = null;
  }
  state.mock = false;
  setStatus('Disconnected', 'off');
  syncConnButtons();
}

// Web Serial transport. Wraps a real port: writes go out the writer, and
// a read loop feeds received lines to dispatchLine. close() cancels the
// reader, waits for the loop to release its stream lock, then closes the
// port -- closing while a lock is held rejects.
async function makeSerialTransport(onUnexpectedClose) {
  const port = await navigator.serial.requestPort();
  await port.open({ baudRate: BAUD });
  const writer = port.writable.getWriter();
  let closing = false;
  let reader = null;

  const readDone = (async () => {
    const decoder = new TextDecoder();
    let buffer = '';
    reader = port.readable.getReader();
    let unexpected = false;
    try {
      while (true) {
        const { value, done } = await reader.read();
        if (done) {
          break;
        }
        buffer += decoder.decode(value, { stream: true });
        let nl;
        while ((nl = buffer.indexOf('\n')) >= 0) {
          const line = buffer.slice(0, nl).replace(/\r$/, '');
          buffer = buffer.slice(nl + 1);
          if (line.length > 0) {
            dispatchLine(line);
          }
        }
      }
    } catch (err) {
      if (!closing) {
        log('read error: ' + err.message, 'err');
        unexpected = true;
      }
    } finally {
      try { reader.releaseLock(); } catch (e) { /* ignore */ }
    }
    if (unexpected) {
      onUnexpectedClose();
    }
  })();

  return {
    send(text) {
      return writer.write(new TextEncoder().encode(text + '\n'));
    },
    async close() {
      closing = true;
      if (reader) {
        try { await reader.cancel(); } catch (e) { /* ignore */ }
      }
      try { await readDone; } catch (e) { /* ignore */ }
      try { writer.releaseLock(); } catch (e) { /* ignore */ }
      try { await port.close(); } catch (e) { /* ignore */ }
    },
  };
}

// Mock transport. No hardware: a createMockDevice() instance answers the
// cp protocol, and its replies are delivered on a later tick to mimic the
// asynchronous arrival of serial data.
function makeMockTransport() {
  const device = createMockDevice();
  let open = true;
  let streamTimer = null;

  function stopStream() {
    if (streamTimer !== null) {
      clearInterval(streamTimer);
      streamTimer = null;
    }
  }

  return {
    send(text) {
      if (!open) {
        return;
      }
      const responses = device.handle(text);
      // Deliver on microtasks: asynchronous, like real serial arrival,
      // but without the setTimeout throttling a background tab imposes.
      for (const line of responses) {
        queueMicrotask(() => {
          if (open) {
            dispatchLine(line);
          }
        });
      }
      // The stream commands also start or stop the unsolicited telemetry
      // the firmware would push once streaming is on.
      const trimmed = text.trim();
      if (trimmed === 'cp stream on' && streamTimer === null) {
        streamTimer = setInterval(() => {
          if (open) {
            dispatchLine(device.telemetry());
          }
        }, 200);
      } else if (trimmed === 'cp stream off') {
        stopStream();
      }
    },
    close() {
      open = false;
      stopStream();
    },
  };
}

// A received line. Protocol lines start with "cp "; everything else is
// firmware debug output and is shown dimmed but otherwise ignored.
function dispatchLine(line) {
  if (!line.startsWith('cp ')) {
    log(line, 'debug');
    return;
  }
  // Telemetry is unsolicited and arrives continuously. It is never part
  // of a command response, so it is routed out before the pending-command
  // logic and is not written to the protocol log.
  if (line.startsWith('cp tlm ')) {
    handleTelemetry(line);
    return;
  }
  log(line, 'rx');
  const p = state.pending;
  if (!p) {
    return;
  }
  if (p.isList) {
    if (line === 'cp end') {
      finishPending(p.lines);
    } else {
      p.lines.push(line);
    }
  } else {
    finishPending(line);
  }
}

function finishPending(result) {
  const p = state.pending;
  if (!p) {
    return;
  }
  clearTimeout(p.timer);
  state.pending = null;
  p.resolve(result);
}

// Send one command and resolve when its response arrives. For a list
// command the response is the array of "cp param" lines; for the rest it
// is the single "cp ok" / "cp err" / "cp fw" line.
function sendCommand(text, isList) {
  if (state.pending) {
    return Promise.reject(new Error('a command is already in flight'));
  }
  if (!state.transport) {
    return Promise.reject(new Error('not connected'));
  }
  log('> ' + text, 'tx');
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      if (state.pending && state.pending.timer === timer) {
        state.pending = null;
      }
      reject(new Error('timeout waiting for response to "' + text + '"'));
    }, CMD_TIMEOUT_MS);
    state.pending = { isList: !!isList, lines: [], resolve, reject, timer };
    // Send only after the pending slot is registered, so a transport that
    // delivers its reply synchronously or on a microtask still finds it.
    Promise.resolve(state.transport.send(text)).catch((err) => {
      if (state.pending && state.pending.timer === timer) {
        clearTimeout(timer);
        state.pending = null;
        reject(err);
      }
    });
  });
}

// ---------------------------------------------------------------------------
// Protocol operations
// ---------------------------------------------------------------------------

async function handshake() {
  const reply = await sendCommand('cp', false);
  const tok = reply.split(/\s+/);
  if (tok[1] === 'fw') {
    const label = state.mock ? 'Mock device' : 'Connected';
    setStatus(label + '  fw ' + tok[2], 'on');
  }
}

async function refreshParams() {
  const lines = await sendCommand('cp list', true);
  state.params = lines.map(parseParamLine).filter((p) => p !== null);
  renderForm();
}

// "cp param <idx> <key> <value> <def> <min> <max> <persist>"
function parseParamLine(line) {
  const t = line.split(/\s+/);
  if (t.length !== 9 || t[1] !== 'param') {
    return null;
  }
  return {
    idx: parseInt(t[2], 10),
    key: t[3],
    value: parseFloat(t[4]),
    def: parseFloat(t[5]),
    min: parseFloat(t[6]),
    max: parseFloat(t[7]),
    persist: t[8] === '1',
  };
}

async function writeChanged() {
  const changed = [];
  for (const p of state.params) {
    const input = document.getElementById('p_' + p.key);
    if (!input) {
      continue;
    }
    const v = parseFloat(input.value);
    if (!Number.isNaN(v) && v !== p.value) {
      changed.push({ key: p.key, value: v });
    }
  }
  if (changed.length === 0) {
    log('nothing changed', 'info');
    return;
  }
  for (const c of changed) {
    await sendCommand('cp set ' + c.key + ' ' + c.value, false);
  }
  await refreshParams();
  log('wrote ' + changed.length + ' parameter(s)', 'info');
}

async function saveFlash() {
  const reply = await sendCommand('cp save', false);
  log(reply.startsWith('cp ok') ? 'saved to flash' : 'save failed: ' + reply,
      reply.startsWith('cp ok') ? 'info' : 'err');
}

async function reloadFlash() {
  await sendCommand('cp load', false);
  await refreshParams();
  log('reloaded from flash', 'info');
}

async function resetDefaults() {
  await sendCommand('cp defaults', false);
  await refreshParams();
  log('registry reset to defaults (not yet saved to flash)', 'info');
}

// ---------------------------------------------------------------------------
// Form rendering
// ---------------------------------------------------------------------------

const GROUPS = [
  { title: 'Hover gains', match: (k) => k.includes('_hover') },
  { title: 'Forward-flight gains', match: (k) => k.includes('_ff') },
  { title: 'Integral gains', match: (k) => k.startsWith('ki_') },
  { title: 'Other', match: () => true },
];

function renderForm() {
  els.form.innerHTML = '';
  els.toolbar.classList.remove('hidden');
  const assigned = new Set();
  for (const group of GROUPS) {
    const members = state.params.filter(
        (p) => !assigned.has(p.key) && group.match(p.key));
    if (members.length === 0) {
      continue;
    }
    members.forEach((p) => assigned.add(p.key));
    els.form.appendChild(renderGroup(group.title, members));
  }
}

function renderGroup(title, members) {
  const section = document.createElement('section');
  section.className = 'group';
  const h = document.createElement('h2');
  h.textContent = title;
  section.appendChild(h);
  for (const p of members) {
    section.appendChild(renderParam(p));
  }
  return section;
}

function renderParam(p) {
  const row = document.createElement('div');
  row.className = 'param';

  const label = document.createElement('label');
  label.textContent = p.key;
  label.htmlFor = 'p_' + p.key;

  const input = document.createElement('input');
  input.type = 'number';
  input.id = 'p_' + p.key;
  input.step = 'any';
  input.min = p.min;
  input.max = p.max;
  input.value = p.value;

  const meta = document.createElement('span');
  meta.className = 'meta';
  meta.textContent = 'default ' + p.def + '  range ' + p.min + ' to ' + p.max;

  input.addEventListener('input', () => {
    const v = parseFloat(input.value);
    row.classList.toggle('dirty', !Number.isNaN(v) && v !== p.value);
  });

  row.appendChild(label);
  row.appendChild(input);
  row.appendChild(meta);
  return row;
}

// ---------------------------------------------------------------------------
// Telemetry tab
// ---------------------------------------------------------------------------

// "cp tlm <roll> <pitch> <yaw> <armed> <fs> <mode> <loop_us> <ch1..ch6>"
// Newer firmware appends instrument fields (alt, yaw rate, accel) after
// the channels; accept any line with at least the original 15 tokens and
// ignore the extras.
function handleTelemetry(line) {
  const t = line.split(/\s+/);
  if (t.length < 15) {
    return;
  }
  els.tRoll.textContent = t[2];
  els.tPitch.textContent = t[3];
  els.tYaw.textContent = t[4];
  const armed = t[5] === '1';
  const failsafe = t[6] === '1';
  setBadge(els.tArmed, armed ? 'ARMED' : 'disarmed',
           armed ? 'badge-warn' : 'badge-idle');
  setBadge(els.tFailsafe, failsafe ? 'FAILSAFE' : 'clear',
           failsafe ? 'badge-alert' : 'badge-ok');
  els.tMode.textContent = t[7];
  els.tLoop.textContent = t[8];
  for (let i = 0; i < 6; i++) {
    updateChannel(i, parseInt(t[9 + i], 10));
  }
}

function setBadge(el, text, cls) {
  el.textContent = text;
  el.className = 'badge ' + cls;
}

function buildChannelRows() {
  els.tChannels.innerHTML = '';
  for (let i = 0; i < 6; i++) {
    const row = document.createElement('div');
    row.className = 'chan';

    const label = document.createElement('span');
    label.className = 'chan-label';
    label.textContent = 'Ch' + (i + 1);

    const track = document.createElement('div');
    track.className = 'chan-track';
    const fill = document.createElement('div');
    fill.className = 'chan-fill';
    fill.id = 'chan-fill-' + i;
    track.appendChild(fill);

    const val = document.createElement('span');
    val.className = 'chan-val';
    val.id = 'chan-val-' + i;
    val.textContent = '----';

    row.appendChild(label);
    row.appendChild(track);
    row.appendChild(val);
    els.tChannels.appendChild(row);
  }
}

function updateChannel(i, us) {
  const fill = document.getElementById('chan-fill-' + i);
  const val = document.getElementById('chan-val-' + i);
  if (!fill || !val || Number.isNaN(us)) {
    return;
  }
  const frac = Math.min(1, Math.max(0, (us - 1000) / 1000));
  fill.style.width = (frac * 100).toFixed(1) + '%';
  val.textContent = us;
}

function showTab(name) {
  const panels = {
    params: els.panelParams,
    telemetry: els.panelTelemetry,
    log: els.panelLog,
    firmware: els.panelFirmware,
  };
  const tabs = {
    params: els.tabParams,
    telemetry: els.tabTelemetry,
    log: els.tabLog,
    firmware: els.tabFirmware,
  };
  for (const key of Object.keys(panels)) {
    panels[key].classList.toggle('hidden', key !== name);
    tabs[key].classList.toggle('active', key === name);
  }
}

// ---------------------------------------------------------------------------
// Log tab
// ---------------------------------------------------------------------------

// Decode a .BIN buffer, render its summary and a sampled record table,
// or show an error if the buffer is not a valid log.
function loadLogBuffer(buffer, name) {
  try {
    const records = decodeLog(buffer);
    hideLogError();
    renderLogSummary(summarizeLog(records), name);
    renderLogCharts(records);
    renderLogTable(records);
    els.logSummarySection.classList.remove('hidden');
    els.logChartsSection.classList.remove('hidden');
    els.logTableSection.classList.remove('hidden');
  } catch (err) {
    showLogError(name + ': ' + err.message);
    els.logSummarySection.classList.add('hidden');
    els.logChartsSection.classList.add('hidden');
    els.logTableSection.classList.add('hidden');
  }
}

function renderLogSummary(s, name) {
  const modeBits = [];
  for (let m = 0; m < 3; m++) {
    if (s.modeCounts[m] > 0) {
      const pct = ((s.modeCounts[m] / s.records) * 100).toFixed(0);
      modeBits.push(logModeName(m) + ' ' + pct + '%');
    }
  }
  const rows = [
    ['File', name],
    ['Records', String(s.records)],
    ['Duration', s.durationS.toFixed(1) + ' s'],
    ['Loop period', 'mean ' + s.loopMeanUs.toFixed(0) +
        ' us, max ' + s.loopMaxUs + ' us'],
    ['Loop overruns',
     s.overruns + ' (over ' + LOG_LOOP_OVERRUN_US + ' us)'],
    ['Armed', (s.armedFraction * 100).toFixed(0) + '% of records'],
    ['Failsafe events', String(s.failsafeEvents)],
    ['Gyro RMS', s.gyroRms.map((g) => g.toFixed(1)).join(' / ') +
        ' dps (roll/pitch/yaw)'],
    ['Sample rate', s.sampleRateHz.toFixed(0) + ' Hz (armed segment)'],
    ['Dominant oscillation', s.oscillation.map((o, i) =>
        ['roll', 'pitch', 'yaw'][i] + ' ' + o.freqHz.toFixed(1) +
        ' Hz / ' + o.amplitudeDps.toFixed(1) + ' dps').join(', ')],
    ['Flight mode', modeBits.join(', ') || '--'],
  ];
  els.logSummary.innerHTML = '';
  for (const [label, value] of rows) {
    const row = document.createElement('div');
    row.className = 'lrow';
    const l = document.createElement('span');
    l.className = 'lrow-label';
    l.textContent = label;
    const v = document.createElement('span');
    v.className = 'lrow-value';
    v.textContent = value;
    row.appendChild(l);
    row.appendChild(v);
    els.logSummary.appendChild(row);
  }
}

const LOG_TABLE_MAX_ROWS = 300;

function renderLogTable(records) {
  const step = Math.max(1, Math.ceil(records.length / LOG_TABLE_MAX_ROWS));
  const t0 = records[0].t_us;
  const head = ['t (s)', 'loop us', 'gyro x', 'gyro y', 'gyro z',
                'pid r', 'pid p', 'pid y', 'm1 us', 'm2 us',
                'mode', 'armed', 'fs'];
  let html = '<table class="ltable"><thead><tr>';
  for (const h of head) {
    html += '<th>' + h + '</th>';
  }
  html += '</tr></thead><tbody>';
  for (let i = 0; i < records.length; i += step) {
    const r = records[i];
    const cells = [
      ((r.t_us - t0) / 1e6).toFixed(2),
      r.loop_us,
      r.gyro[0].toFixed(1), r.gyro[1].toFixed(1), r.gyro[2].toFixed(1),
      r.pid[0].toFixed(2), r.pid[1].toFixed(2), r.pid[2].toFixed(2),
      r.motor_us[0], r.motor_us[1],
      logModeName(r.mode),
      (r.flags & LOG_FLAG_ARMED) ? 1 : 0,
      (r.flags & LOG_FLAG_FAILSAFE) ? 1 : 0,
    ];
    html += '<tr>';
    for (const c of cells) {
      html += '<td>' + c + '</td>';
    }
    html += '</tr>';
  }
  html += '</tbody></table>';
  els.logTable.innerHTML = html;
  if (step > 1) {
    const shown = Math.ceil(records.length / step);
    const note = document.createElement('p');
    note.className = 'hint';
    note.textContent = 'Showing every ' + step + 'th record (' + shown +
        ' of ' + records.length + ').';
    els.logTable.appendChild(note);
  }
}

// Series colours, shared by the charts and their legend.
const CHART_COLORS = ['#15B8A6', '#2FA36B', '#E0A100'];

function renderLogCharts(records) {
  const MAX_POINTS = 640;
  const step = Math.max(1, Math.ceil(records.length / MAX_POINTS));
  const gyro = [[], [], []];
  const pid = [[], [], []];
  for (let i = 0; i < records.length; i += step) {
    const r = records[i];
    for (let a = 0; a < 3; a++) {
      gyro[a].push(r.gyro[a]);
      pid[a].push(r.pid[a]);
    }
  }
  drawChart(els.logChartGyro, gyro.map((v, a) =>
      ({ color: CHART_COLORS[a], values: v })));
  drawChart(els.logChartPid, pid.map((v, a) =>
      ({ color: CHART_COLORS[a], values: v })));
  els.logChartLegend.textContent =
      'Blue: roll / x.   Green: pitch / y.   Orange: yaw / z.';
}

// Draw overlaid line series on a canvas. Every series shares one x-axis
// (sample index) and one auto-scaled y-axis.
function drawChart(canvas, series) {
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;
  const pad = { l: 46, r: 8, t: 8, b: 14 };
  ctx.fillStyle = '#0c0f14';
  ctx.fillRect(0, 0, w, h);
  if (series.length === 0 || series[0].values.length === 0) {
    return;
  }

  const n = series[0].values.length;
  let min = Infinity;
  let max = -Infinity;
  for (const s of series) {
    for (const v of s.values) {
      if (v < min) {
        min = v;
      }
      if (v > max) {
        max = v;
      }
    }
  }
  if (min === max) {
    min -= 1;
    max += 1;
  }
  const plotW = w - pad.l - pad.r;
  const plotH = h - pad.t - pad.b;
  const xAt = (i) => pad.l + (n > 1 ? (i / (n - 1)) * plotW : 0);
  const yAt = (v) => pad.t + (1 - (v - min) / (max - min)) * plotH;

  ctx.strokeStyle = '#383c43';
  ctx.lineWidth = 1;
  ctx.strokeRect(pad.l, pad.t, plotW, plotH);
  if (min < 0 && max > 0) {
    const y0 = yAt(0);
    ctx.beginPath();
    ctx.moveTo(pad.l, y0);
    ctx.lineTo(w - pad.r, y0);
    ctx.stroke();
  }
  ctx.fillStyle = '#9CA3AF';
  ctx.font = "10px 'JetBrains Mono', ui-monospace, Menlo, monospace";
  ctx.fillText(max.toFixed(1), 4, pad.t + 9);
  ctx.fillText(min.toFixed(1), 4, h - pad.b);

  for (const s of series) {
    ctx.strokeStyle = s.color;
    ctx.beginPath();
    for (let i = 0; i < n; i++) {
      const x = xAt(i);
      const y = yAt(s.values[i]);
      if (i === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.stroke();
  }
}

function showLogError(msg) {
  els.logError.textContent = msg;
  els.logError.classList.remove('hidden');
}

function hideLogError() {
  els.logError.classList.add('hidden');
}

// ---------------------------------------------------------------------------
// Firmware tab
// ---------------------------------------------------------------------------

// Parse a picked .uf2, show what it contains, and enable Flash. A bad
// file leaves Flash disabled and shows the error.
function loadFirmwareFile(buffer, name) {
  state.firmwareUf2 = null;
  els.fwFlash.disabled = true;
  els.fwInfo.classList.add('hidden');
  hideFwError();
  try {
    const uf2 = parseUf2(buffer);
    state.firmwareUf2 = uf2;
    showFirmwareInfo(uf2, name);
    els.fwFlash.disabled = false;
  } catch (err) {
    showFwError(name + ': ' + err.message);
  }
}

function showFirmwareInfo(uf2, name) {
  const image = buildFlashImage(uf2);
  const fams = [];
  let rp2350 = false;
  for (const f of uf2.families) {
    fams.push(UF2_FAMILIES[f] || ('0x' + f.toString(16)));
    if (UF2_RP2350_FAMILIES.includes(f)) {
      rp2350 = true;
    }
  }
  let text = name + ': ' + uf2.blocks.length + ' blocks, ' +
      image.bytes.length + ' bytes from 0x' +
      image.base.toString(16) + '. Family: ' +
      (fams.join(', ') || 'unknown') + '.';
  if (!rp2350) {
    text += ' Warning: this is not an RP2350 image.';
  }
  els.fwInfo.textContent = text;
  els.fwInfo.classList.remove('hidden');
}

async function flashFirmware() {
  if (!state.firmwareUf2) {
    return;
  }
  if (!('usb' in navigator)) {
    showFwError('this browser does not support WebUSB. Use Chrome or Edge.');
    return;
  }
  els.fwFlash.disabled = true;
  els.fwBoot.disabled = true;
  hideFwError();
  els.fwProgressSection.classList.remove('hidden');
  els.fwBar.style.width = '0%';
  try {
    await flashUf2(state.firmwareUf2, (msg, frac) => {
      els.fwStatus.textContent = msg;
      if (typeof frac === 'number') {
        els.fwBar.style.width = (frac * 100).toFixed(0) + '%';
      }
    });
    els.fwStatus.textContent = 'Flash complete. The board is rebooting.';
    els.fwBar.style.width = '100%';
  } catch (err) {
    els.fwStatus.textContent = 'Flash failed.';
    showFwError(err.message);
  } finally {
    els.fwFlash.disabled = state.firmwareUf2 === null;
    els.fwBoot.disabled = !state.connected;
  }
}

async function rebootToBootloader() {
  try {
    await sendCommand('cp boot', false);
  } catch (err) {
    // The board resets and the serial link drops. That is the expected
    // outcome; the read loop handles the disconnect.
  }
  log('reboot to bootloader requested', 'info');
}

function showFwError(msg) {
  els.fwError.textContent = msg;
  els.fwError.classList.remove('hidden');
}

function hideFwError() {
  els.fwError.classList.add('hidden');
}

// ---------------------------------------------------------------------------
// UI helpers
// ---------------------------------------------------------------------------

function setStatus(text, cls) {
  els.status.textContent = text;
  els.status.className = 'status ' + cls;
}

function setToolbarEnabled(on) {
  for (const b of [els.write, els.save, els.reload, els.defaults]) {
    b.disabled = !on;
  }
}

// Connect and Mock buttons reflect connection state. Disconnect always
// goes through the Connect button, whichever transport is in use.
function syncConnButtons() {
  els.connect.disabled = !state.connected && !('serial' in navigator);
  els.connect.textContent = state.connected ? 'Disconnect' : 'Connect';
  els.mock.disabled = state.connected;
}

// Run an async handler with the toolbar locked, so two protocol commands
// never overlap on the single shared port.
function guarded(fn) {
  return async () => {
    setToolbarEnabled(false);
    try {
      await fn();
    } catch (err) {
      log('error: ' + err.message, 'err');
    } finally {
      setToolbarEnabled(state.connected);
    }
  };
}

// ---------------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------------

function main() {
  if (!('serial' in navigator)) {
    // Web Serial is missing, but the mock device still works, so the
    // page stays usable. The banner explains the real-hardware limit.
    els.unsupported.classList.remove('hidden');
  }
  syncConnButtons();

  els.connect.addEventListener('click', async () => {
    els.connect.disabled = true;
    els.mock.disabled = true;
    if (state.connected) {
      await disconnect();
    } else {
      await connectSerial();
    }
  });

  els.mock.addEventListener('click', async () => {
    els.connect.disabled = true;
    els.mock.disabled = true;
    await connectMock();
  });

  els.tabParams.addEventListener('click', () => showTab('params'));
  els.tabTelemetry.addEventListener('click', () => showTab('telemetry'));
  els.tabLog.addEventListener('click', () => showTab('log'));

  els.logFile.addEventListener('change', async () => {
    const file = els.logFile.files[0];
    if (!file) {
      return;
    }
    try {
      const buffer = await file.arrayBuffer();
      loadLogBuffer(buffer, file.name);
    } catch (err) {
      showLogError('could not read file: ' + err.message);
    }
  });

  els.logSample.addEventListener('click', () => {
    loadLogBuffer(makeSampleLog(), 'sample.BIN');
  });

  els.tabFirmware.addEventListener('click', () => showTab('firmware'));

  els.fwFile.addEventListener('change', async () => {
    const file = els.fwFile.files[0];
    if (!file) {
      return;
    }
    try {
      const buffer = await file.arrayBuffer();
      loadFirmwareFile(buffer, file.name);
    } catch (err) {
      showFwError('could not read file: ' + err.message);
    }
  });

  els.fwFlash.addEventListener('click', flashFirmware);
  els.fwBoot.addEventListener('click', rebootToBootloader);

  els.write.addEventListener('click', guarded(writeChanged));
  els.save.addEventListener('click', guarded(saveFlash));
  els.reload.addEventListener('click', guarded(reloadFlash));
  els.defaults.addEventListener('click', guarded(resetDefaults));
}

main();

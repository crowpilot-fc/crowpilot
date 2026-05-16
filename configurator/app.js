// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
//
// CrowPilot Configurator v0. Talks the line-oriented "cp" text protocol
// to the flight controller over the Web Serial API, renders the runtime
// parameter registry as a form, and writes edited values back.

'use strict';

const BAUD = 115200;
const CMD_TIMEOUT_MS = 2500;

const state = {
  transport: null,  // {send(text), close()} -- serial or mock
  mock: false,
  connected: false,
  params: [],       // [{idx, key, value, def, min, max, persist}]
  pending: null,    // {isList, lines, resolve, reject, timer}
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
}

async function disconnect() {
  state.connected = false;
  setToolbarEnabled(false);
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
    },
    close() {
      open = false;
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

  els.write.addEventListener('click', guarded(writeChanged));
  els.save.addEventListener('click', guarded(saveFlash));
  els.reload.addEventListener('click', guarded(reloadFlash));
  els.defaults.addEventListener('click', guarded(resetDefaults));
}

main();

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

// The CrowPilot companion web UI: one self-contained page the ESP serves
// to a phone. It speaks the `cp` protocol over a WebSocket and presents
// live telemetry plus a slider per tunable parameter. Edit this file
// directly; it is the source of truth (no build step).

#pragma once

const char kWebUi[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>CrowPilot</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin:0; background:#0f1115; color:#e9ecf1;
         font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif; }
  header { display:flex; align-items:center; justify-content:space-between;
           padding:14px 16px; background:#171a21; position:sticky; top:0;
           border-bottom:1px solid #262b36; }
  header h1 { font-size:18px; margin:0; letter-spacing:.5px; }
  #status { font-size:12px; padding:4px 10px; border-radius:12px;
            background:#3a2530; color:#ff9aa8; }
  #status.on { background:#1f3a2a; color:#7ee2a8; }
  main { padding:14px; max-width:680px; margin:0 auto; }
  .card { background:#171a21; border:1px solid #262b36; border-radius:12px;
          padding:14px; margin-bottom:14px; }
  .card h2 { font-size:13px; text-transform:uppercase; letter-spacing:1px;
             color:#8b93a4; margin:0 0 12px; }
  .card h2 small { text-transform:none; letter-spacing:0; color:#5b6373;
                   font-weight:normal; }
  .stats { display:grid; grid-template-columns:repeat(4,1fr); gap:10px; }
  .stat { background:#0f1115; border-radius:8px; padding:10px 6px;
          text-align:center; }
  .stat .v { font-size:20px; font-weight:600; }
  .stat .k { font-size:10px; color:#8b93a4; text-transform:uppercase;
             margin-top:3px; letter-spacing:.5px; }
  .v.bad { color:#ff7a8a; } .v.good { color:#7ee2a8; }
  .param { margin-bottom:14px; }
  .param .row { display:flex; justify-content:space-between; font-size:13px;
                margin-bottom:4px; }
  .param .key { color:#c7cdda; } .param .pv { color:#7ee2a8;
                font-variant-numeric:tabular-nums; }
  input[type=range] { width:100%; height:30px; accent-color:#5b8cff; }
  .actions { display:flex; gap:8px; margin-top:6px; }
  button { flex:1; padding:11px; font-size:14px; border:0; border-radius:8px;
           background:#2a3140; color:#e9ecf1; }
  button:active { background:#5b8cff; }
  #log { font:11px/1.5 ui-monospace,Menlo,Consolas,monospace; color:#8b93a4;
         max-height:120px; overflow:auto; white-space:pre-wrap; }
</style>
</head>
<body>
<header>
  <h1>CrowPilot</h1>
  <span id="status">connecting</span>
</header>
<main>
  <section class="card">
    <h2>Telemetry <small id="fw"></small></h2>
    <div class="stats">
      <div class="stat"><div class="v" id="t_roll">-</div><div class="k">roll deg</div></div>
      <div class="stat"><div class="v" id="t_pitch">-</div><div class="k">pitch deg</div></div>
      <div class="stat"><div class="v" id="t_yaw">-</div><div class="k">yaw deg</div></div>
      <div class="stat"><div class="v" id="t_loop">-</div><div class="k">loop us</div></div>
      <div class="stat"><div class="v" id="t_arm">-</div><div class="k">arm</div></div>
      <div class="stat"><div class="v" id="t_fs">-</div><div class="k">failsafe</div></div>
      <div class="stat" style="grid-column:span 2"><div class="v" id="t_mode">-</div><div class="k">mode</div></div>
    </div>
  </section>
  <section class="card">
    <h2>Parameters</h2>
    <div id="params"></div>
    <div class="actions">
      <button id="save">Save to flash</button>
      <button id="reload">Reload</button>
      <button id="defaults">Defaults</button>
    </div>
  </section>
  <section class="card">
    <h2>Log</h2>
    <div id="log"></div>
  </section>
</main>
<script>
const $ = id => document.getElementById(id);
let ws, params = {};

function setStatus(text, on) {
  const s = $("status"); s.textContent = text;
  s.className = on ? "on" : "";
}
function log(msg) {
  const l = $("log");
  l.textContent = (msg + "\n" + l.textContent).slice(0, 2000);
}
function fmt(x) { return parseFloat(Number(x).toFixed(5)); }

function send(cmd) {
  if (ws && ws.readyState === 1) ws.send(cmd);
}

function addParam(p) {
  let e = params[p.key];
  if (!e) {
    const wrap = document.createElement("div");
    wrap.className = "param";
    wrap.innerHTML =
      '<div class="row"><span class="key"></span><span class="pv"></span></div>' +
      '<input type="range">';
    wrap.querySelector(".key").textContent = p.key;
    const range = wrap.querySelector("input");
    range.min = p.min; range.max = p.max;
    range.step = (p.max - p.min) / 500 || 0.001;
    const pv = wrap.querySelector(".pv");
    e = params[p.key] = { wrap, range, pv, lastSent: 0 };
    range.addEventListener("input", () => {
      pv.textContent = fmt(range.value);
      const now = Date.now();
      if (now - e.lastSent >= 100) {
        e.lastSent = now;
        send("cp set " + p.key + " " + range.value);
      }
    });
    range.addEventListener("change", () => {
      send("cp set " + p.key + " " + range.value);
    });
    $("params").appendChild(wrap);
  }
  e.range.value = p.val;
  e.pv.textContent = fmt(p.val);
}

function updateTlm(t) {
  const set = (id, v, cls) => {
    const el = $(id); el.textContent = v;
    el.className = "v" + (cls ? " " + cls : "");
  };
  set("t_roll", t[2]); set("t_pitch", t[3]); set("t_yaw", t[4]);
  set("t_arm", t[5] === "1" ? "ARMED" : "safe", t[5] === "1" ? "bad" : "good");
  set("t_fs", t[6] === "1" ? "LOST" : "ok", t[6] === "1" ? "bad" : "good");
  set("t_mode", t[7] || "-");
  set("t_loop", t[8] || "-");
}

function handle(line) {
  const t = line.trim().split(/\s+/);
  if (t[0] !== "cp") return;
  if (t[1] === "param") {
    addParam({ idx:+t[2], key:t[3], val:+t[4], def:+t[5],
               min:+t[6], max:+t[7], persist: t[8] === "1" });
  } else if (t[1] === "tlm") {
    updateTlm(t);
  } else if (t[1] === "fw") {
    $("fw").textContent = "fw " + t[2];
  } else if (t[1] === "end") {
    log("parameters loaded");
  } else {
    log(line.trim());
  }
}

function connect() {
  setStatus("connecting", false);
  ws = new WebSocket("ws://" + location.hostname + ":81/");
  ws.onopen = () => {
    setStatus("connected", true);
    send("cp");
    send("cp list");
    send("cp stream on");
  };
  ws.onmessage = e => handle(e.data);
  ws.onerror = () => ws.close();
  ws.onclose = () => {
    setStatus("disconnected", false);
    setTimeout(connect, 2000);
  };
}

$("save").onclick = () => send("cp save");
$("reload").onclick = () => send("cp list");
$("defaults").onclick = () => { send("cp defaults"); send("cp list"); };

connect();
</script>
</body>
</html>
)HTML";

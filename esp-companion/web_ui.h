// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar

// The CrowPilot companion web UI: one self-contained page the ESP serves
// to a phone. It speaks the `cp` protocol over a WebSocket. Two tabs: a
// Fly instrument panel (artificial horizon, altimeter, vertical speed,
// turn and slip, G-load) and a Tune panel (a slider per parameter plus a
// log). Edit this file directly; it is the source of truth (no build step).

#pragma once

const char kWebUi[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>CrowPilot</title>
<style>
  :root { color-scheme: dark; --teal:#15b8a6; --bad:#e5484d; --ok:#2fa36b; --warn:#e0a100; }
  * { box-sizing: border-box; }
  body { margin:0; background:#10141A; color:#e9ecf1;
         font-family:Inter,system-ui,-apple-system,Segoe UI,Roboto,sans-serif; }
  header { display:flex; align-items:center; justify-content:space-between;
           padding:12px 16px; background:#1B2127; border-bottom:1px solid #262b36; }
  header h1 { font-size:17px; margin:0; letter-spacing:.5px; }
  #status { font-size:12px; padding:4px 10px; border-radius:12px;
            background:#3a2530; color:#ff9aa8; }
  #status.on { background:#1f3a2a; color:#7ee2a8; }
  .tabs { display:flex; background:#1B2127; border-bottom:1px solid #262b36; }
  .tabs button { flex:1; padding:12px; background:none; border:0; color:#8b93a4;
                 font-size:14px; font-weight:600; border-bottom:2px solid transparent; }
  .tabs button.active { color:#fff; border-bottom-color:var(--teal); }
  main { padding:14px; max-width:560px; margin:0 auto; }
  .tab { display:none; } .tab.active { display:block; }
  .badges { display:flex; gap:8px; margin-bottom:12px; }
  .badge { flex:1; text-align:center; padding:8px; border-radius:8px; font-size:12px;
           font-weight:700; letter-spacing:.5px; background:#1B2127; color:#8b93a4; }
  .badge.bad { background:#3a2226; color:#ff8a96; } .badge.ok { background:#1d3329; color:#7ee2a8; }
  .ah-wrap { display:flex; justify-content:center; margin-bottom:14px; }
  .ah { width:min(78vw,300px); height:min(78vw,300px); }
  .grid { display:grid; grid-template-columns:repeat(2,1fr); gap:10px; }
  .gauge { background:#1B2127; border:1px solid #262b36; border-radius:12px;
           padding:12px; text-align:center; }
  .gauge .lbl { font-size:10px; color:#8b93a4; text-transform:uppercase;
                letter-spacing:1px; margin-bottom:4px; }
  .gauge .val { font:600 26px/1 "JetBrains Mono",ui-monospace,Menlo,monospace; }
  .gauge .unit { font-size:11px; color:#8b93a4; }
  .vsi-up { color:var(--ok); } .vsi-dn { color:var(--warn); }
  .note { font-size:11px; color:#6b7280; margin-top:12px; text-align:center; }
  .card { background:#1B2127; border:1px solid #262b36; border-radius:12px;
          padding:14px; margin-bottom:14px; }
  .card h2 { font-size:13px; text-transform:uppercase; letter-spacing:1px;
             color:#8b93a4; margin:0 0 12px; }
  .param { margin-bottom:14px; }
  .param .row { display:flex; justify-content:space-between; font-size:13px; margin-bottom:4px; }
  .param .pv { color:var(--teal); font:13px "JetBrains Mono",ui-monospace,monospace; }
  input[type=range] { width:100%; height:30px; accent-color:var(--teal); }
  .actions { display:flex; gap:8px; }
  button.act { flex:1; padding:11px; font-size:14px; border:0; border-radius:8px;
               background:#2a3140; color:#e9ecf1; }
  button.act:active { background:var(--teal); }
  #log { font:11px/1.5 "JetBrains Mono",ui-monospace,monospace; color:#8b93a4;
         max-height:120px; overflow:auto; white-space:pre-wrap; }
</style>
</head>
<body>
<header>
  <h1>CrowPilot</h1>
  <span id="status">connecting</span>
</header>
<div class="tabs">
  <button id="tab-fly-btn" class="active">Fly</button>
  <button id="tab-tune-btn">Tune</button>
</div>
<main>
  <section id="tab-fly" class="tab active">
    <div class="badges">
      <div class="badge" id="b_arm">ARM</div>
      <div class="badge" id="b_fs">LINK</div>
      <div class="badge" id="b_mode">MODE</div>
    </div>
    <div class="ah-wrap">
      <svg class="ah" viewBox="0 0 240 240">
        <defs><clipPath id="ahc"><circle cx="120" cy="120" r="110"/></clipPath></defs>
        <circle cx="120" cy="120" r="113" fill="#0c1014" stroke="#2a313a" stroke-width="3"/>
        <g clip-path="url(#ahc)">
          <g id="ahmove">
            <rect x="-180" y="-280" width="600" height="400" fill="#3d7ea6"/>
            <rect x="-180" y="120" width="600" height="400" fill="#7a5c3a"/>
            <line x1="-180" y1="120" x2="420" y2="120" stroke="#fff" stroke-width="2"/>
            <g stroke="#fff" stroke-width="1.4" fill="#fff" font-size="9"
               font-family="ui-monospace,monospace" text-anchor="middle">
              <line x1="98" y1="72"  x2="142" y2="72"/><text x="150" y="75">20</text>
              <line x1="104" y1="96" x2="136" y2="96"/><text x="150" y="99">10</text>
              <line x1="104" y1="144" x2="136" y2="144"/><text x="150" y="147">-10</text>
              <line x1="98" y1="168" x2="142" y2="168"/><text x="150" y="171">-20</text>
            </g>
            <polygon points="120,18 113,31 127,31" fill="#15b8a6"/>
          </g>
        </g>
        <g stroke="#cdd4de" stroke-width="2">
          <line x1="120" y1="9" x2="120" y2="20"/>
          <line x1="68" y1="22" x2="74" y2="32"/>
          <line x1="172" y1="22" x2="166" y2="32"/>
          <line x1="30" y1="58" x2="40" y2="64"/>
          <line x1="210" y1="58" x2="200" y2="64"/>
        </g>
        <g stroke="#ffd23f" stroke-width="3" fill="none">
          <line x1="84" y1="120" x2="108" y2="120"/>
          <line x1="132" y1="120" x2="156" y2="120"/>
        </g>
        <circle cx="120" cy="120" r="3" fill="#ffd23f"/>
      </svg>
    </div>
    <div class="grid">
      <div class="gauge"><div class="lbl">Altitude (rel)</div>
        <div class="val" id="g_alt">--</div><div class="unit">m</div></div>
      <div class="gauge"><div class="lbl">Vertical speed</div>
        <div class="val" id="g_vsi">--</div><div class="unit">m/s</div></div>
      <div class="gauge"><div class="lbl">Turn &amp; slip</div>
        <svg viewBox="0 0 120 84" style="width:100%;max-height:64px">
          <g id="turnac"><line x1="28" y1="34" x2="92" y2="34" stroke="#cdd4de" stroke-width="3"/>
            <line x1="60" y1="27" x2="60" y2="34" stroke="#cdd4de" stroke-width="3"/></g>
          <rect x="40" y="62" width="40" height="15" rx="7.5" fill="#0c1014" stroke="#2a313a"/>
          <line x1="52" y1="60" x2="52" y2="79" stroke="#2a313a"/>
          <line x1="68" y1="60" x2="68" y2="79" stroke="#2a313a"/>
          <circle id="ball" cx="60" cy="69.5" r="5" fill="#cdd4de"/>
        </svg></div>
      <div class="gauge"><div class="lbl">G-load</div>
        <div class="val" id="g_g">--</div><div class="unit">g</div></div>
    </div>
    <div class="note">No airspeed or compass heading. Those need a pitot or GPS
      and a magnetometer (v2). Altitude and vertical speed are barometric and
      relative to power-on.</div>
  </section>

  <section id="tab-tune" class="tab">
    <div class="card">
      <h2>Parameters <small id="fw" style="color:#5b6373"></small></h2>
      <div id="params"></div>
      <div class="actions">
        <button class="act" id="save">Save</button>
        <button class="act" id="reload">Reload</button>
        <button class="act" id="defaults">Defaults</button>
      </div>
    </div>
    <div class="card"><h2>Log</h2><div id="log"></div></div>
  </section>
</main>
<script>
const $ = id => document.getElementById(id);
let ws, params = {};

function setStatus(t, on){ const s=$("status"); s.textContent=t; s.className=on?"on":""; }
function log(m){ const l=$("log"); l.textContent=(m+"\n"+l.textContent).slice(0,2000); }
function fmt(x){ return parseFloat(Number(x).toFixed(5)); }
function clamp(v,a,b){ return v<a?a:(v>b?b:v); }
function send(c){ if(ws&&ws.readyState===1) ws.send(c); }

// --- tabs ---
function showTab(name){
  for(const n of ["fly","tune"]){
    $("tab-"+n).classList.toggle("active", n===name);
    $("tab-"+n+"-btn").classList.toggle("active", n===name);
  }
}
$("tab-fly-btn").onclick = () => showTab("fly");
$("tab-tune-btn").onclick = () => showTab("tune");

// --- instruments ---
function updateAH(roll, pitch){
  if(isNaN(roll)||isNaN(pitch)) return;
  const PXDEG = 2.4;
  $("ahmove").setAttribute("transform",
    "rotate("+(-roll)+" 120 120) translate(0 "+(pitch*PXDEG)+")");
}
function setBadge(el, txt, cls){ el.textContent=txt; el.className="badge"+(cls?" "+cls:""); }

let vsiPrevAlt=null, vsiPrevT=0, vsiVal=0;
function updateAltVsi(alt){
  if(isNaN(alt)) return;
  $("g_alt").textContent = alt.toFixed(1);
  const now = performance.now();
  if(vsiPrevAlt!==null){
    const dt=(now-vsiPrevT)/1000;
    if(dt>0.03){
      vsiVal += 0.3*((alt-vsiPrevAlt)/dt - vsiVal);  // low-pass the derivative
      const e=$("g_vsi");
      e.textContent=(vsiVal>=0?"+":"")+vsiVal.toFixed(1);
      e.className="val "+(vsiVal>0.2?"vsi-up":(vsiVal<-0.2?"vsi-dn":""));
    }
  }
  vsiPrevAlt=alt; vsiPrevT=now;
}
function updateTurn(gz, ay){
  if(!isNaN(gz)) $("turnac").setAttribute("transform",
    "rotate("+clamp(gz*1.2,-35,35)+" 60 34)");   // ~standard-rate turn near a tick
  if(!isNaN(ay)) $("ball").setAttribute("cx", (60 - clamp(ay/0.35,-1,1)*9).toFixed(1));
}
function updateG(ax,ay,az){
  if(isNaN(ax)||isNaN(ay)||isNaN(az)) return;
  $("g_g").textContent = Math.sqrt(ax*ax+ay*ay+az*az).toFixed(2);
}
function updateTlm(t){
  updateAH(+t[2], +t[3]);
  setBadge($("b_arm"), t[5]==="1"?"ARMED":"SAFE", t[5]==="1"?"bad":"ok");
  setBadge($("b_fs"),  t[6]==="1"?"FAILSAFE":"LINK OK", t[6]==="1"?"bad":"ok");
  setBadge($("b_mode"), (t[7]||"-").toUpperCase(), "");
  updateAltVsi(+t[15]);
  updateTurn(+t[16], +t[18]);
  updateG(+t[17], +t[18], +t[19]);
}

// --- parameters (Tune tab) ---
function addParam(p){
  let e=params[p.key];
  if(!e){
    const w=document.createElement("div"); w.className="param";
    w.innerHTML='<div class="row"><span></span><span class="pv"></span></div><input type="range">';
    w.querySelector(".row span").textContent=p.key;
    const r=w.querySelector("input"); r.min=p.min; r.max=p.max;
    r.step=(p.max-p.min)/500||0.001;
    const pv=w.querySelector(".pv");
    e=params[p.key]={range:r,pv:pv,lastSent:0};
    r.addEventListener("input",()=>{ pv.textContent=fmt(r.value);
      const n=Date.now(); if(n-e.lastSent>=100){ e.lastSent=n; send("cp set "+p.key+" "+r.value); }});
    r.addEventListener("change",()=>send("cp set "+p.key+" "+r.value));
    $("params").appendChild(w);
  }
  e.range.value=p.val; e.pv.textContent=fmt(p.val);
}
function handle(line){
  const t=line.trim().split(/\s+/);
  if(t[0]!=="cp") return;
  if(t[1]==="param") addParam({key:t[3],val:+t[4],min:+t[6],max:+t[7]});
  else if(t[1]==="tlm") updateTlm(t);
  else if(t[1]==="fw") $("fw").textContent="fw "+t[2];
  else if(t[1]==="end") log("parameters loaded");
  else log(line.trim());
}
function connect(){
  setStatus("connecting", false);
  ws=new WebSocket("ws://"+location.hostname+":81/");
  ws.onopen=()=>{ setStatus("connected",true); send("cp"); send("cp list"); send("cp stream on"); };
  ws.onmessage=e=>handle(e.data);
  ws.onerror=()=>ws.close();
  ws.onclose=()=>{ setStatus("disconnected",false); setTimeout(connect,2000); };
}
$("save").onclick=()=>send("cp save");
$("reload").onclick=()=>send("cp list");
$("defaults").onclick=()=>{ send("cp defaults"); send("cp list"); };
connect();
</script>
</body>
</html>
)HTML";

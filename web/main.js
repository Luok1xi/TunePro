// SPDX-License-Identifier: GPL-3.0-or-later
import { esc } from './lib/esc.js';
import { createLiveParser, commands, frameStats } from './lib/robot-live-model.js';
import { createStore, createIngest, tracking, saturation, crossings, toCSV, parseCSV, PWM_LIMIT, VBAT_COMP_MIN, VBAT_USB } from './lib/robot-live-data.js';
import { createChart } from './lib/robot-live-charts.js';
import { mountSpeedScope } from './lib/robot-speed-scope.js';
import { mountTunePro } from './lib/robot-tune-pro.js';
import { mountAiTuner } from './lib/robot-ai-tuner.js';
import { mountOptics, mountFluidTabs } from './lib/robot-optics.js';

const $ = q => document.querySelector(q), $$ = q => document.querySelectorAll(q);
const W = 188, H = 120, BAND = [50, 69];
const hasWebSerial = 'serial' in navigator;

// ---------- 主题 / 动效 / 控制拓扑偏好持久化 ----------
const PREF_THEME = 'tp-studio-theme';
const PREF_MOTION = 'tp-studio-motion';
const PREF_TOPO = 'tp-studio-topo';

let currentTheme = localStorage.getItem(PREF_THEME) === 'light' ? 'light' : 'dark';
let currentMotion = ['standard', 'subtle', 'off'].includes(localStorage.getItem(PREF_MOTION)) ? localStorage.getItem(PREF_MOTION) : 'standard';
let currentTopo = localStorage.getItem(PREF_TOPO) === '3cascade' ? '3cascade' : '2s1p';

function applyTheme(theme) {
  currentTheme = theme === 'light' ? 'light' : 'dark';
  document.documentElement.dataset.theme = currentTheme;
  localStorage.setItem(PREF_THEME, currentTheme);
  $('[data-theme-toggle]').textContent = currentTheme === 'light' ? '暗房深色' : '纸面浅色';
  dirty = true;
  window.dispatchEvent(new CustomEvent('tunepro-theme', { detail: { theme: currentTheme } }));
}
function applyMotion(tier) {
  currentMotion = ['standard', 'subtle', 'off'].includes(tier) ? tier : 'standard';
  document.documentElement.dataset.motion = currentMotion;
  localStorage.setItem(PREF_MOTION, currentMotion);
  $$('[data-motion-btn]').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.motionBtn === currentMotion)));
}
function applyTopologyChrome(topo) {
  currentTopo = topo === '3cascade' ? '3cascade' : '2s1p';
  document.documentElement.dataset.topology = currentTopo;
  localStorage.setItem(PREF_TOPO, currentTopo);
  $$('[data-topology-btn]').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.topologyBtn === currentTopo)));
  $('[data-topo-pill]').textContent = currentTopo === '2s1p' ? '两串一并 · 2S1P' : '传统三串 · 3-Cascade';
  $('[data-kpi-speed-lbl]').textContent = currentTopo === '2s1p' ? '均速反馈 · V_AVG' : '左右轮速 · ENC L/R';
}

// 5 种主示波器预设视图（含两串一并专属「共模/差模 PWM」）
const VIEWS = {
  speed: {
    series: [
      { key: 'spd_set', label: '均速目标', color: '--rl-c3', dash: [4, 3] },
      { key: 'spd', label: '均速实测', color: '--rl-c1' },
      { key: 'encL', label: '左实测', color: '--rl-c4', dash: [2, 2] },
      { key: 'encR', label: '右实测', color: '--rl-c6', dash: [2, 2] },
    ],
    unit: { l: '计数/2ms' },
    diag(s, t0, t1) {
      const trAvg = tracking(s, 'spd_set', 'spd', t0, t1);
      const trL = tracking(s, 'setL', 'encL', t0, t1);
      const cr = crossings(s, 'spd_set', 'spd', t0, t1, 2);
      return [
        trAvg && ['均速误差', trAvg.rmsErr.toFixed(1)],
        trL && ['左误差', trL.rmsErr.toFixed(1)],
        cr && ['波动', `${cr.count} 次`, cr.count > 4 ? 'warn' : 'good'],
        trAvg && Number.isFinite(trAvg.lagMs) && ['滞后', `${Math.round(trAvg.lagMs)} ms`, trAvg.lagMs > 30 ? 'warn' : 'good'],
      ].filter(Boolean);
    },
  },
  yaw: {
    series: [
      { key: 'yaw_target_dps', label: '角速度目标 °/s', color: '--rl-c3', dash: [4, 3] },
      { key: 'gyro_dps', label: '陀螺实测 °/s', color: '--rl-c2' },
    ],
    unit: { l: '°/s' },
    diag(s, t0, t1) {
      const tr = tracking(s, 'yaw_target_dps', 'gyro_dps', t0, t1);
      const cr = crossings(s, 'yaw_target_dps', 'gyro_dps', t0, t1, 8);
      return [
        tr && ['跟踪误差', `${tr.rmsErr.toFixed(1)} °/s`],
        tr && Number.isFinite(tr.lagMs) && ['滞后', `${Math.round(tr.lagMs)} ms`, tr.lagMs > 25 ? 'warn' : 'good'],
        cr && ['波动', `${cr.count} 次`, cr.count > 4 ? 'warn' : 'good'],
      ].filter(Boolean);
    },
  },
  pwm_split: {
    series: [
      { key: 'pwm_cm', label: '纵向并联 PWM_speed', color: '--rl-c1' },
      { key: 'pwm_dm', label: '转向串级 PWM_turn', color: '--rl-c2' },
    ],
    fixed: { l: [-PWM_LIMIT * 1.05, PWM_LIMIT * 1.05] },
    unit: { l: 'PWM' },
    diag(s, t0, t1) {
      const cm = s.last('pwm_cm'), dm = s.last('pwm_dm');
      const sL = saturation(s, 'pwmL', PWM_LIMIT, t0, t1), sR = saturation(s, 'pwmR', PWM_LIMIT, t0, t1);
      const satMax = Math.max(Number.isFinite(sL) ? sL : 0, Number.isFinite(sR) ? sR : 0);
      return [
        cm && ['纵向共模', `${Math.round(cm.v)}`],
        dm && ['转向差模', `${Math.round(dm.v)}`],
        Number.isFinite(satMax) && ['单轮饱和', `${Math.round(satMax * 100)}%`, satMax > 0.15 ? 'warn' : 'good'],
      ].filter(Boolean);
    },
  },
  outer: {
    series: [
      { key: 'image_error', label: '视觉偏差 px', color: '--rl-c4' },
      { key: 'yaw_target_dps', label: '外环目标 °/s', color: '--rl-c3', axis: 'r' },
      { key: 'gyro_dps', label: '陀螺 °/s', color: '--rl-c2', axis: 'r' },
    ],
    unit: { l: 'px' },
    diag(s, t0, t1) {
      const tr = tracking(s, 'yaw_target_dps', 'gyro_dps', t0, t1);
      const e = s.last('image_error'), k = s.last('route_k');
      return [
        e && ['当前偏差', `${e.v.toFixed(0)} px`],
        k && ['路线步', `k=${k.v}`],
        tr && ['角速误差', `${tr.rmsErr.toFixed(1)} °/s`],
      ].filter(Boolean);
    },
  },
  pwm: {
    series: [
      { key: 'pwmL', label: '左 PWM', color: '--rl-c1' },
      { key: 'pwmR', label: '右 PWM', color: '--rl-c5' },
    ],
    fixed: { l: [-PWM_LIMIT * 1.05, PWM_LIMIT * 1.05] },
    unit: { l: 'PWM' },
    diag(s, t0, t1) {
      const sL = saturation(s, 'pwmL', PWM_LIMIT, t0, t1), sR = saturation(s, 'pwmR', PWM_LIMIT, t0, t1);
      return [
        Number.isFinite(sL) && ['左饱和', `${Math.round(sL * 100)}%`, sL > 0.15 ? 'warn' : 'good'],
        Number.isFinite(sR) && ['右饱和', `${Math.round(sR * 100)}%`, sR > 0.15 ? 'warn' : 'good'],
      ].filter(Boolean);
    },
  },
};

let transport = 'bridge';
let bridgeConn = false, bridgeES = null;
let wsPort = null, wsReader = null, wsWriter = null;
const isConnected = () => (transport === 'bridge' ? bridgeConn : Boolean(wsPort && wsWriter));

const store = createStore(120000);
const ingest = createIngest(store);
let currentView = 'speed', spanSec = 10, following = true, viewEnd = 0, hoverT = null, dirty = true;
let mainChart = null;
let lastFrame = null, badCnt = 0, byteLog = [], frameTimes = [];
let camImgData = null, camImg32 = null, camImgW = 0, camImgH = 0, lastGuideKey = '';
let logsDirty = false;
const logs = [];
const MAX_LOGS = 800;

const camCanvas = $('[data-cam-canvas]'), camOverlay = $('[data-cam-overlay]');
const camCtx = camCanvas.getContext('2d', { alpha: false }), ovCtx = camOverlay.getContext('2d');
const logCard = $('[data-log-card]');
const logCountEl = $('[data-log-count]');
const logPreEl = $('[data-log-pre]');
const logStatusChk = $('[data-log-status]');
const guidesChk = $('[data-guides]');
const readoutEl = $('[data-readout]');
const diagEl = $('[data-diag]');
const rateEl = $('[data-rate]');
const linkEl = $('[data-link]');
const speedScope = mountSpeedScope($('[data-speed-scope]'));

// Cached KPI DOM references + diff map (zero redundant DOM writes at 5 Hz)
const kpiRefs = {
  speed: $('[data-kpi-speed]'),
  speedSub: $('[data-kpi-speed-sub]'),
  gyro: $('[data-kpi-gyro]'),
  gyroSub: $('[data-kpi-gyro-sub]'),
  pwm: $('[data-kpi-pwm]'),
  pwmSub: $('[data-kpi-pwm-sub]'),
  vis: $('[data-kpi-vis]'),
  visSub: $('[data-kpi-vis-sub]'),
  bat: $('[data-kpi-bat]'),
  meters: {
    speed: $('[data-kpi-meter="speed"]'),
    gyro: $('[data-kpi-meter="gyro"]'),
    pwm: $('[data-kpi-meter="pwm"]'),
    vis: $('[data-kpi-meter="vis"]'),
    bat: $('[data-kpi-meter="bat"]'),
  },
};
const kpiCache = Object.create(null);
function setTextCached(key, el, text) {
  if (!el || kpiCache[key] === text) return;
  kpiCache[key] = text;
  el.textContent = text;
}
function setMeterCached(name, ratio, hot = false) {
  const el = kpiRefs.meters[name];
  if (!el) return;
  const clamped = Number.isFinite(ratio) ? Math.max(-1, Math.min(1, ratio)) : 0;
  const mStr = clamped.toFixed(3);
  const hStr = String(Boolean(hot));
  if (kpiCache[`m_${name}`] !== mStr) {
    kpiCache[`m_${name}`] = mStr;
    el.style.setProperty('--m', mStr);
  }
  if (kpiCache[`h_${name}`] !== hStr) {
    kpiCache[`h_${name}`] = hStr;
    el.dataset.hot = hStr;
  }
}

let inspectorMode = 'manual';
function setInspectorMode(mode) {
  inspectorMode = mode === 'ai' ? 'ai' : 'manual';
  const showAi = inspectorMode === 'ai';
  $('[data-tune-pro]').hidden = showAi;
  aiTuner.setOpen(showAi);
  $$('[data-inspector-tab]').forEach(b => b.setAttribute('aria-selected', String(b.dataset.inspectorTab === inspectorMode)));
  $('[data-ai-toggle]').setAttribute('aria-expanded', String(showAi));
}

function getLiveTelemetrySnapshot() {
  return {
    spd: store.last('spd')?.v,
    encL: store.last('encL')?.v,
    encR: store.last('encR')?.v,
    gyro: store.last('gyro_dps')?.v,
    yawTgt: store.last('yaw_target_dps')?.v,
    errPx: store.last('image_error')?.v,
    pwmCm: store.last('pwm_cm')?.v,
    pwmDm: store.last('pwm_dm')?.v,
    pwmL: store.last('pwmL')?.v,
    pwmR: store.last('pwmR')?.v,
  };
}

const tunePro = mountTunePro($('[data-tune-pro]'), {
  stageHost: $('[data-stage-host]'),
  camStreamHost: $('[data-cam-stream-host]'),
  isConnected,
  sendRaw,
  initialTopology: currentTopo,
  onLog: addLog,
  onTopologyChange: topo => { applyTopologyChrome(topo); aiTuner.setTopology(topo); },
  onFocusLoop: tab => {
    setPlotView(tab === 'outer' ? 'outer' : tab === 'yaw' ? 'yaw' : 'speed');
    if (inspectorMode !== 'manual') setInspectorMode('manual');
  },
});

const aiTuner = mountAiTuner($('[data-ai-tuner]'), {
  getTransport: () => transport,
  isConnected,
  getLiveTelemetry: getLiveTelemetrySnapshot,
  onLog: addLog,
  onFocusView: view => setPlotView(view),
  onStateBadge: ({ active, open }) => {
    const btn = $('[data-ai-toggle]');
    if (btn) {
      btn.dataset.running = String(Boolean(active));
      if (typeof open === 'boolean') btn.setAttribute('aria-expanded', String(open));
    }
    const dot = $('[data-inspector-ai-dot]');
    if (dot) dot.hidden = !active;
    if (open === false && inspectorMode === 'ai') {
      inspectorMode = 'manual';
      $('[data-tune-pro]').hidden = false;
      $$('[data-inspector-tab]').forEach(b => b.setAttribute('aria-selected', String(b.dataset.inspectorTab === 'manual')));
    }
  },
});

if (!hasWebSerial) $('[data-transport] option[value="webserial"]').disabled = true;

function addLog(kind, text) {
  const now = new Date();
  const ts = `${String(now.getHours()).padStart(2, '0')}:${String(now.getMinutes()).padStart(2, '0')}:${String(now.getSeconds()).padStart(2, '0')}.${String(now.getMilliseconds()).padStart(3, '0')}`;
  logs.push({ kind, line: `${ts}  ${text}` });
  if (logs.length > MAX_LOGS) logs.splice(0, logs.length - MAX_LOGS);
  logsDirty = true;
  if (logCard?.open) renderLogs();
}
function renderLogs() {
  logsDirty = false;
  setTextCached('logCount', logCountEl, `${logs.length} 条`);
  if (!logCard?.open) return;
  const showStatus = logStatusChk.checked;
  const list = logs.filter(l => showStatus || l.kind !== 'status').slice(-200);
  logPreEl.innerHTML = list.map(l => `<span data-k="${l.kind}">${esc(l.line)}</span>`).join('\n');
  logPreEl.scrollTop = logPreEl.scrollHeight;
}

// ---------- 串口解析 ----------
const parse = createLiveParser(ev => {
  const now = performance.now();
  tunePro.onEvent(ev);
  if (ev.type === 'm2' || ev.type === 'tlm' || ev.type === 'status' || ev.type === 'packet') {
    ingest.push(ev, now);
    if (ev.type === 'status') addLog('status', ev.line);
    dirty = true;
  } else if (ev.type === 'frame') {
    lastFrame = ev;
    frameTimes.push(now);
    while (frameTimes.length && now - frameTimes[0] > 3000) frameTimes.shift();
    drawFrame(ev);
  } else if (ev.type === 'frame-lost') {
    badCnt++;
    addLog('error', `丢图 #${ev.seq}：${ev.reason}`);
  } else if (ev.type === 'tlm-bad') {
    badCnt++;
  } else if (ev.type === 'tune-capture') {
    addLog(ev.complete ? 'event' : 'error', `收到板载波形：${ev.rows.length} 点 (${ev.header.dt_ms}ms) ${ev.complete ? '完整' : ev.issues.join('；')}`);
    if (ev.rows.length) speedScope.addCapture(ev);
  } else if (ev.type === 'line') {
    addLog('reply', ev.text);
  }
});

function feedBytes(bytes) {
  const now = performance.now();
  byteLog.push([now, bytes.length]);
  while (byteLog.length && now - byteLog[0][0] > 1000) byteLog.shift();
  parse(bytes);
}

// ---------- 本机串口桥 + WebSerial 双通道 ----------
function setConnUI(yes, label) {
  $('[data-conn-text]').textContent = label;
  $('[data-conn-text]').dataset.on = String(yes);
  $('[data-connect]').disabled = yes;
  $('[data-disconnect]').disabled = !yes;
  $('[data-transport]').disabled = yes;
  $('[data-port]').disabled = yes;
  $('[data-baud]').disabled = yes;
  $$('[data-grab]').forEach(b => { b.disabled = !yes; });
}

async function refreshPorts() {
  const sel = $('[data-port]');
  try {
    const r = await fetch('/api/serial/ports', { cache: 'no-store' });
    const data = await r.json();
    const list = data.ports || [];
    sel.innerHTML = list.length
      ? list.map(p => {
          const shortHint = p.port === 'MOCK0' ? '虚拟小车' : String(p.hint || '').replace(/（.*?）|\(.*?\)/g, '').slice(0, 14);
          return `<option value="${esc(p.port)}" title="${esc(p.port)} · ${esc(p.hint)}">${esc(p.port)}${shortHint ? ` · ${esc(shortHint)}` : ''}</option>`;
        }).join('')
      : '<option value="">未检测到串口</option>';
    if (data.status?.port && list.some(p => p.port === data.status.port)) sel.value = data.status.port;
    else if (data.demo && list.some(p => p.port === 'MOCK0')) sel.value = 'MOCK0';
    if (data.status?.connected && !bridgeConn && transport === 'bridge') attachSSE(data.status);
  } catch (e) {
    sel.innerHTML = '<option value="">扫描失败</option>';
    addLog('error', `扫描串口失败：${e.message}`);
  }
}

function attachSSE(st) {
  if (bridgeES) { bridgeES.close(); bridgeES = null; }
  bridgeConn = Boolean(st.connected);
  bridgeES = new EventSource('/api/serial/stream');
  bridgeES.addEventListener('data', ev => {
    try {
      const { b64 } = JSON.parse(ev.data);
      const bin = atob(b64), bytes = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
      feedBytes(bytes);
    } catch {}
  });
  bridgeES.addEventListener('status', ev => {
    try {
      const s = JSON.parse(ev.data);
      bridgeConn = Boolean(s.connected);
      setConnUI(bridgeConn, bridgeConn ? `已连 ${s.port} (${s.baud})` : (s.last_error || '重连中…'));
    } catch {}
  });
  bridgeES.addEventListener('saved', ev => {
    try { const { path } = JSON.parse(ev.data); addLog('event', `已自动落盘保存：${path}`); } catch {}
  });
  setConnUI(true, `已连 ${st.port} (${st.baud})`);
  addLog('event', `本机串口桥已连接 ${st.port} · ${st.baud}`);
  tunePro.onConnected();
  aiTuner.onConnected();
}

async function connect() {
  const baud = Number($('[data-baud]').value);
  if (transport === 'bridge') {
    const port = $('[data-port]').value;
    if (!port) { addLog('error', '请先插上串口并点「刷新」选择 COM 口'); return; }
    setConnUI(false, `正在连接 ${port}…`);
    try {
      const r = await fetch('/api/serial/connect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ port, baud, auto_reconnect: true, auto_log: $('[data-autolog]').checked }),
      });
      const data = await r.json();
      if (!r.ok) throw new Error(data.error || `HTTP ${r.status}`);
      attachSSE(data);
    } catch (e) {
      addLog('error', `连接失败：${e.message}`);
      setConnUI(false, '未连接');
    }
    return;
  }
  try {
    wsPort = await navigator.serial.requestPort();
    await wsPort.open({ baudRate: baud, bufferSize: 65536 });
    wsWriter = wsPort.writable.getWriter();
    setConnUI(true, `WebSerial (${baud})`);
    addLog('event', `WebSerial 已打开 · ${baud}`);
    tunePro.onConnected();
    (async () => {
      while (wsPort?.readable) {
        wsReader = wsPort.readable.getReader();
        try {
          for (;;) { const { value, done } = await wsReader.read(); if (done) break; if (value) feedBytes(value); }
        } catch (e) { addLog('error', `串口读取中断：${e.message}`); }
        finally { wsReader?.releaseLock(); wsReader = null; }
        break;
      }
      if (wsPort) disconnect();
    })();
  } catch (e) {
    if (e?.name !== 'NotFoundError') addLog('error', `WebSerial 连接失败：${e.message}`);
    await disconnect();
  }
}

async function disconnect() {
  if (bridgeES) { bridgeES.close(); bridgeES = null; }
  if (bridgeConn) {
    bridgeConn = false;
    await fetch('/api/serial/disconnect', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' }).catch(() => {});
  }
  try { await wsReader?.cancel(); } catch {}
  try { wsWriter?.releaseLock(); } catch {}
  try { await wsPort?.close(); } catch {}
  wsPort = wsReader = wsWriter = null;
  parse.end();
  tunePro.onDisconnected();
  aiTuner.onDisconnected();
  setConnUI(false, '未连接');
}

async function sendRaw(text, { urgent = false, paced = false } = {}) {
  if (!isConnected()) return false;
  if (transport === 'bridge') {
    try {
      const r = await fetch('/api/serial/send', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text, paced, urgent }),
      });
      if (!r.ok) { const d = await r.json().catch(() => ({})); throw new Error(d.error || `HTTP ${r.status}`); }
      return true;
    } catch (e) { addLog('error', `发送失败：${e.message}`); return false; }
  }
  try {
    const bytes = new TextEncoder().encode(text);
    if (urgent || !paced || Number($('[data-baud]').value) >= 921600) await wsWriter.write(bytes);
    else {
      for (let i = 0; i < bytes.length; i += 4) {
        await wsWriter.write(bytes.subarray(i, i + 4));
        if (i + 4 < bytes.length) await new Promise(r => setTimeout(r, 4));
      }
    }
    return true;
  } catch (e) { addLog('error', `发送失败：${e.message}`); return false; }
}

// ---------- 实时主示波器 ----------
function setPlotView(name) {
  currentView = name;
  $$('[data-view]').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.view === name)));
  const cfg = VIEWS[name];
  mainChart?.destroy?.();
  mainChart = createChart($('[data-main-chart]'), store, { series: cfg.series, fixed: cfg.fixed, unit: cfg.unit });
  $('[data-legend]').innerHTML = cfg.series.map(s => `<span style="--c:var(${s.color})">${s.label}${s.axis === 'r' ? '（右轴）' : ''}</span>`).join('');
  dirty = true;
}

function getWindow() {
  const [a, b] = store.range();
  const end = following ? b : Math.min(Math.max(viewEnd, a), b);
  return spanSec ? [end - spanSec * 1000, end] : [a, Math.max(end, a + 1000)];
}

// ---------- 实时摄像头绘制（复用 Uint32Array 像素缓冲，零逐帧分配） ----------
function drawFrame(f) {
  if (!camImgData || camImgW !== f.width || camImgH !== f.height) {
    camImgW = f.width;
    camImgH = f.height;
    camImgData = camCtx.createImageData(f.width, f.height);
    camImg32 = new Uint32Array(camImgData.data.buffer);
    if (camCanvas.width !== f.width || camCanvas.height !== f.height) {
      camCanvas.width = camOverlay.width = f.width;
      camCanvas.height = camOverlay.height = f.height;
      lastGuideKey = '';
    }
  }
  const px = f.pixels, len = Math.min(px.length, camImg32.length);
  if (f.kind === 'bin') {
    for (let i = 0; i < len; i++) camImg32[i] = px[i] ? 0xFFFFFFFF : 0xFF000000;
  } else {
    for (let i = 0; i < len; i++) {
      const v = px[i];
      camImg32[i] = 0xFF000000 | (v << 16) | (v << 8) | v;
    }
  }
  camCtx.putImageData(camImgData, 0, 0);
  drawGuides(true);
  const fps = frameTimes.length >= 2 ? ((frameTimes.length - 1) * 1000 / Math.max(1, frameTimes.at(-1) - frameTimes[0])).toFixed(1) : '—';
  const tag = f.compressed ? ({ 1: '二值 RLE', 2: '二值 1bit', 3: '灰度 94×60', 4: '灰度 47×30' }[f.fmt] || '压缩') : (f.kind === 'bin' ? '二值' : '灰度');
  $('[data-cam-title]').textContent = `${tag} · #${f.seq} · ${fps} fps${f.compBytes ? ` · ${f.compBytes} B` : ''}`;
  const m = f.meta;
  if (m) {
    $('[data-cam-stats]').innerHTML =
      `偏差 <b>${m.image_error} px</b> · 均灰 <b>${m.avg_gray}</b>${m.avg_gray > 175 ? ' <span data-level="warn">⚠ 反光偏亮</span>' : ''} · L/R <b>${m.lp}/${m.rp}</b> · 步 <b>k=${m.route_k}</b>`;
  } else {
    const s = frameStats(f.pixels);
    $('[data-cam-stats]').innerHTML = `最小 <b>${s.min}</b> · 中位 <b>${s.median}</b> · 最大 <b>${s.max}</b> · 过曝 <b>${s.saturated}</b>`;
  }
  $('[data-save-png]').disabled = false;
}

function drawGuides(force = false) {
  const show = guidesChk.checked;
  const e = store.last('image_error');
  const px = e ? e.v : lastFrame?.meta?.image_error;
  const pxRound = Number.isFinite(px) ? Math.round(px) : 'n';
  const key = `${show}|${camOverlay.width}|${camOverlay.height}|${pxRound}`;
  if (!force && key === lastGuideKey) return;
  lastGuideKey = key;

  ovCtx.clearRect(0, 0, camOverlay.width, camOverlay.height);
  if (!show) return;
  const ky = camOverlay.height / H, kx = camOverlay.width / W;
  ovCtx.fillStyle = 'rgba(110,161,255,0.18)';
  ovCtx.fillRect(0, BAND[0] * ky, camOverlay.width, (BAND[1] - BAND[0] + 1) * ky);
  ovCtx.fillStyle = 'rgba(226,178,89,0.78)';
  ovCtx.fillRect(camOverlay.width / 2, 0, 1, camOverlay.height);
  if (Number.isFinite(px) && Math.abs(px) < W) {
    ovCtx.fillStyle = 'rgba(217,119,87,0.95)';
    ovCtx.fillRect(camOverlay.width / 2 + px * kx - 1, BAND[0] * ky, 2, (BAND[1] - BAND[0] + 1) * ky);
  }
}

function updateKPIStrip() {
  const spd = store.last('spd')?.v, encL = store.last('encL')?.v, encR = store.last('encR')?.v;
  const gyro = store.last('gyro_dps')?.v, yawTgt = store.last('yaw_target_dps')?.v;
  const pwmCm = store.last('pwm_cm')?.v, pwmDm = store.last('pwm_dm')?.v, pwmL = store.last('pwmL')?.v, pwmR = store.last('pwmR')?.v;
  const errPx = store.last('image_error')?.v, routeK = store.last('route_k')?.v;
  const vbat = tunePro.getVbatMv();

  setTextCached('spd', kpiRefs.speed, Number.isFinite(spd) ? `${spd.toFixed(1)}` : '—');
  setTextCached('spdSub', kpiRefs.speedSub, Number.isFinite(encL) && Number.isFinite(encR)
    ? `左 ${Math.round(encL)} · 右 ${Math.round(encR)} cnt/2ms` : '左 — · 右 — cnt/2ms');
  setMeterCached('speed', Number.isFinite(spd) ? Math.abs(spd) / 260 : 0, Math.abs(spd || 0) > 220);

  setTextCached('gyro', kpiRefs.gyro, Number.isFinite(gyro) ? `${gyro >= 0 ? '+' : ''}${gyro.toFixed(1)} °/s` : '—');
  setTextCached('gyroSub', kpiRefs.gyroSub, Number.isFinite(yawTgt) ? `目标 ${yawTgt >= 0 ? '+' : ''}${yawTgt.toFixed(0)} °/s` : '目标 — °/s');
  setMeterCached('gyro', Number.isFinite(gyro) ? gyro / 600 : 0, Math.abs(gyro || 0) > 380);

  setTextCached('pwm', kpiRefs.pwm, Number.isFinite(pwmCm) && Number.isFinite(pwmDm)
    ? `${Math.round(pwmCm)} / ${Math.round(pwmDm)}` : '—');
  setTextCached('pwmSub', kpiRefs.pwmSub, Number.isFinite(pwmL) && Number.isFinite(pwmR)
    ? `左 ${Math.round(pwmL)} · 右 ${Math.round(pwmR)}` : '纵向 — · 转向 —');
  const maxPwmRatio = Math.max(Math.abs(pwmL || 0), Math.abs(pwmR || 0)) / PWM_LIMIT;
  setMeterCached('pwm', Number.isFinite(maxPwmRatio) ? maxPwmRatio : 0, maxPwmRatio > 0.85);

  setTextCached('vis', kpiRefs.vis, Number.isFinite(errPx) ? `${errPx >= 0 ? '+' : ''}${errPx.toFixed(0)} px` : '—');
  const fps = frameTimes.length >= 2 ? ((frameTimes.length - 1) * 1000 / Math.max(1, frameTimes.at(-1) - frameTimes[0])).toFixed(1) : null;
  setTextCached('visSub', kpiRefs.visSub, `k = ${Number.isFinite(routeK) ? Math.round(routeK) : '—'} · ${fps ? `${fps} fps` : '等待图传'}`);
  setMeterCached('vis', Number.isFinite(errPx) ? errPx / 70 : 0, Math.abs(errPx || 0) > 35);

  const batEl = kpiRefs.bat;
  let batTxt = '—', batLvl = 'idle', batM = 0, batHot = false;
  if (Number.isFinite(vbat) && vbat >= VBAT_USB) {
    batTxt = `${(vbat / 1000).toFixed(2)} V`;
    batLvl = vbat < 10500 ? 'bad' : vbat < VBAT_COMP_MIN ? 'warn' : 'good';
    batM = (vbat - 10500) / 2100;
    batHot = vbat < VBAT_COMP_MIN;
  } else if (Number.isFinite(vbat) && vbat > 0) {
    batTxt = 'USB';
    batLvl = 'warn';
    batM = 0.25;
    batHot = true;
  }
  setTextCached('bat', batEl, batTxt);
  if (kpiCache.batLvl !== batLvl) { kpiCache.batLvl = batLvl; batEl.dataset.level = batLvl; }
  setMeterCached('bat', batM, batHot);

  tunePro.updateTelemetry({ spd, encL, encR, gyro, yawTgt, errPx, pwmCm, pwmDm, pwmL, pwmR });
}

// ---------- UI 绑定 ----------
$('[data-theme-toggle]').addEventListener('click', () => applyTheme(currentTheme === 'light' ? 'dark' : 'light'));
$$('[data-motion-btn]').forEach(b => b.addEventListener('click', () => applyMotion(b.dataset.motionBtn)));
$$('[data-topology-btn]').forEach(b => b.addEventListener('click', () => {
  const next = b.dataset.topologyBtn;
  applyTopologyChrome(next);
  tunePro.setTopology(next, { sendToCar: true });
  aiTuner.setTopology(next);
}));
$('[data-ai-toggle]').addEventListener('click', () => {
  setInspectorMode(inspectorMode === 'ai' ? 'manual' : 'ai');
});
$$('[data-inspector-tab]').forEach(b => b.addEventListener('click', () => setInspectorMode(b.dataset.inspectorTab)));
$('[data-inspector-expand]').addEventListener('click', e => {
  const ws = $('[data-workspace]');
  const next = ws.dataset.wideInspector !== 'true';
  ws.dataset.wideInspector = String(next);
  e.currentTarget.setAttribute('aria-pressed', String(next));
  e.currentTarget.textContent = next ? '收窄' : '展宽';
  setTimeout(() => { dirty = true; }, 320);
});
window.addEventListener('keydown', e => {
  if (e.key === 'Escape') aiTuner.onEmergencyStop();
});

$('[data-transport]').addEventListener('change', e => {
  transport = e.target.value;
  $('[data-bridge-box]').hidden = transport !== 'bridge';
  $('[data-autolog-wrap]').hidden = transport !== 'bridge';
});
$('[data-refresh]').addEventListener('click', refreshPorts);
$('[data-connect]').addEventListener('click', connect);
$('[data-disconnect]').addEventListener('click', disconnect);
$$('[data-view]').forEach(b => b.addEventListener('click', () => setPlotView(b.dataset.view)));
$$('[data-span]').forEach(b => b.addEventListener('click', () => {
  spanSec = Number(b.dataset.span);
  $$('[data-span]').forEach(x => x.setAttribute('aria-pressed', String(x === b)));
  dirty = true;
}));
$('[data-follow]').addEventListener('click', () => {
  following = !following;
  if (!following) viewEnd = store.range()[1];
  $('[data-follow]').setAttribute('aria-pressed', String(following));
  $('[data-follow]').textContent = following ? '暂停' : '跟随最新';
  dirty = true;
});
$('[data-clear-plot]').addEventListener('click', () => { store.clear(); dirty = true; });
$('[data-guides]').addEventListener('change', drawGuides);
$$('[data-grab]').forEach(b => b.addEventListener('click', () => sendRaw(commands.grab(b.dataset.grab))));
$('[data-save-png]').addEventListener('click', () => camCanvas.toBlob(blob => blob && download(blob, `car-cam-${Date.now()}.png`)));
$('[data-save-csv]').addEventListener('click', () => { if (store.size) download(new Blob([toCSV(store)], { type: 'text/csv;charset=utf-8' }), `tunepro-${Date.now()}.csv`); });
$('[data-open]').addEventListener('change', async e => {
  const f = e.target.files[0]; if (!f) return; e.target.value = '';
  store.clear();
  if (/\.csv$/i.test(f.name)) {
    for (const r of parseCSV(await f.text())) { const { t, ...rest } = r; store.push(t, rest); }
  } else {
    const bytes = new Uint8Array(await f.arrayBuffer());
    for (let i = 0; i < bytes.length; i += 65536) parse(bytes.subarray(i, i + 65536));
    parse.end();
  }
  following = false; spanSec = 0; viewEnd = store.range()[1];
  $$('[data-span]').forEach(x => x.setAttribute('aria-pressed', String(x.dataset.span === '0')));
  $('[data-follow]').setAttribute('aria-pressed', 'false'); $('[data-follow]').textContent = '跟随最新';
  addLog('event', `已打开离线文件 ${f.name}（${store.size} 点）`);
  dirty = true;
});
$('[data-log-clear]').addEventListener('click', () => { logs.length = 0; logsDirty = true; renderLogs(); });
$('[data-log-copy]').addEventListener('click', () => navigator.clipboard?.writeText(logs.map(l => l.line).join('\n')));
$('[data-log-status]').addEventListener('change', renderLogs);
logCard?.addEventListener('toggle', () => { if (logCard.open) renderLogs(); });

const chartCanvas = $('[data-main-chart]');
let chartRect = null;
chartCanvas.addEventListener('pointerenter', () => { chartRect = chartCanvas.getBoundingClientRect(); }, { passive: true });
chartCanvas.addEventListener('pointermove', e => {
  const r = chartRect || chartCanvas.getBoundingClientRect(), [t0, t1] = getWindow();
  hoverT = t0 + (e.clientX - r.left - 58) / Math.max(1, r.width - 106) * (t1 - t0);
  dirty = true;
}, { passive: true });
chartCanvas.addEventListener('pointerleave', () => { chartRect = null; hoverT = null; dirty = true; });

function download(blob, name) {
  const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = name; a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

// ---------- 30 fps 刷新循环（后台隐藏零绘制 + 差分更新零抖动） ----------
let lastVer = -1, lastDraw = 0, lastDiag = 0, lastDiagHTML = '';
function tick(now) {
  requestAnimationFrame(tick);
  if (document.hidden) return;
  if ((dirty || (following && store.version !== lastVer)) && now - lastDraw > 32) {
    lastDraw = now; lastVer = store.version; dirty = false;
    const [t0, t1] = getWindow();
    const { readout } = mainChart.draw({ t0, t1, hoverT });
    const hideBox = !readout.length || hoverT === null;
    if (readoutEl.hidden !== hideBox) readoutEl.hidden = hideBox;
    if (!hideBox) readoutEl.innerHTML = readout.map(r => `<span style="--c:var(${r.color})">${esc(r.label)} <b>${Math.abs(r.v) >= 100 ? r.v.toFixed(0) : r.v.toFixed(2)}</b></span>`).join('');
  }
  if (now - lastDiag > 220) {
    lastDiag = now;
    const [t0, t1] = getWindow();
    const chips = VIEWS[currentView].diag(store, t0, t1);
    const diagHTML = chips.map(([k, v, lv]) => `<span${lv ? ` data-level="${lv}"` : ''}><small>${esc(k)}</small>${esc(v)}</span>`).join('');
    if (diagHTML !== lastDiagHTML) { lastDiagHTML = diagHTML; diagEl.innerHTML = diagHTML; }
    while (byteLog.length && now - byteLog[0][0] > 1000) byteLog.shift();
    const kbs = byteLog.reduce((a, [, n]) => a + n, 0) / 1024;
    setTextCached('rate', rateEl, ingest.ctx.source ? `${ingest.ctx.source} · ${ingest.ctx.rate.toFixed(ingest.ctx.rate < 10 ? 1 : 0)} Hz` : '无数据');
    setTextCached('link', linkEl, `${kbs.toFixed(1)} KB/s${badCnt ? ` · 坏包 ${badCnt}` : ''}`);
    if (logsDirty) setTextCached('logCount', logCountEl, `${logs.length} 条`);
    updateKPIStrip();
    drawGuides(false);
  }
}

applyTheme(currentTheme);
applyMotion(currentMotion);
applyTopologyChrome(currentTopo);
setPlotView('speed');
refreshPorts();
mountOptics(document);
mountFluidTabs($('.tp-topo-tabs'), { kind: 'pill' });
mountFluidTabs($('.tp-line-tabs'), { kind: 'line' });
mountFluidTabs($('.tp-inspector-tabs'), { selector: 'button[role="tab"]', kind: 'line' });
try { new EventSource('/api/app/lifecycle'); } catch {}
requestAnimationFrame(tick);

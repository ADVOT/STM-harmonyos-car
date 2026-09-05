'use strict';

/* ============================================================
   先锋号 · 气象观测站 —— 前端逻辑
   WebSocket 实时遥测 + 双轴曲线 + 迷你遥控
   ============================================================ */

const $ = (id) => document.getElementById(id);

const el = {
  clock: $('clock'), connDot: $('conn-dot'), connLabel: $('conn-label'), footConn: $('foot-conn'),
  deviceDot: $('device-dot'), deviceStatus: $('device-status'),
  tempHero: $('temp-hero'), tempDesc: $('temp-desc'), tempDelta: $('temp-delta'),
  humiVal: $('humi-val'), lightVal: $('light-val'),
  humiMin: $('humi-min'), humiMax: $('humi-max'),
  lightMin: $('light-min'), lightMax: $('light-max'),
  freshVal: $('fresh-val'), freshNote: $('fresh-note'), freshTime: $('fresh-time'),
  sparkHumi: $('spark-humi'), sparkLight: $('spark-light'),
  chart: $('chart'), chartEmpty: $('chart-empty'), chartTooltip: $('chart-tooltip'), chartRange: $('chart-range'),
};

const state = {
  current: null,
  history: [],
  device: { status: null },
  lastEventTime: null,
};

const STALE_MS = 30000;               // 固件 5s 上报，超 30s 未见新数据视为陈旧
const CHART_WINDOW_MS = 10 * 60 * 1000;

/* ---------- 工具 ---------- */
function isNum(v) { return typeof v === 'number' && Number.isFinite(v); }
function pad2(n) { return String(n).padStart(2, '0'); }
function fmtClock(d) { return `${pad2(d.getHours())}:${pad2(d.getMinutes())}:${pad2(d.getSeconds())}`; }
function fmtHM(ms) { const d = new Date(ms); return `${pad2(d.getHours())}:${pad2(d.getMinutes())}`; }
function fmtHMS(ms) { const d = new Date(ms); return `${pad2(d.getHours())}:${pad2(d.getMinutes())}:${pad2(d.getSeconds())}`; }

function ageText(ms) {
  if (ms == null) return '—';
  if (ms < 1500) return '刚刚';
  if (ms < 60000) return `${Math.round(ms / 1000)} 秒前`;
  if (ms < 3600000) return `${Math.round(ms / 60000)} 分钟前`;
  if (ms < 86400000) return `${Math.round(ms / 3600000)} 小时前`;
  return `${Math.round(ms / 86400000)} 天前`;
}

// 数值平滑滚动
const prevVals = new WeakMap();
function animateValue(node, to, decimals) {
  if (!isNum(to)) { node.textContent = '—'; return; }
  const from = prevVals.has(node) ? prevVals.get(node) : to;
  prevVals.set(node, to);
  const start = performance.now(), dur = 650;
  (function frame(t) {
    const p = Math.min(1, (t - start) / dur);
    const e = 1 - Math.pow(1 - p, 3);
    node.textContent = (from + (to - from) * e).toFixed(decimals);
    if (p < 1) requestAnimationFrame(frame);
  })(start);
}

/* ---------- 天气文案 ---------- */
function tempWord(t) {
  if (!isNum(t)) return '等待数据…';
  if (t < 10) return '清冷，注意保暖';
  if (t < 18) return '微凉，清爽宜人';
  if (t < 26) return '温和，体感舒适';
  if (t < 32) return '温暖，略有暖意';
  return '炎热，注意降温';
}

/* ============================================================
   曲线绘制（Canvas，双轴：左温度 / 右湿度）
   ============================================================ */
const chart = { ctx: el.chart.getContext('2d'), hover: -1, geom: null };

function setupCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}

function domain(vals, padRatio = 0.15, clamp01) {
  const nums = vals.filter(isNum);
  if (!nums.length) return [0, 1];
  let lo = Math.min(...nums), hi = Math.max(...nums);
  if (hi - lo < 1e-6) { lo -= 1; hi += 1; }
  const pad = (hi - lo) * padRatio;
  lo -= pad; hi += pad;
  if (clamp01) { lo = Math.max(0, lo); hi = Math.min(100, hi); }
  return [lo, hi];
}

// Catmull-Rom → 贝塞尔平滑折线
function traceSmooth(ctx, pts) {
  if (pts.length < 2) return;
  ctx.moveTo(pts[0].x, pts[0].y);
  for (let i = 0; i < pts.length - 1; i++) {
    const p0 = pts[Math.max(0, i - 1)], p1 = pts[i], p2 = pts[i + 1], p3 = pts[Math.min(pts.length - 1, i + 2)];
    const c1x = p1.x + (p2.x - p0.x) / 6, c1y = p1.y + (p2.y - p0.y) / 6;
    const c2x = p2.x - (p3.x - p1.x) / 6, c2y = p2.y - (p3.y - p1.y) / 6;
    ctx.bezierCurveTo(c1x, c1y, c2x, c2y, p2.x, p2.y);
  }
}

function windowPoints() {
  const h = state.history;
  if (!h.length) return [];
  const last = h[h.length - 1].t, first = h[0].t;
  const t0 = Math.max(first, last - CHART_WINDOW_MS);
  return h.filter((p) => p.t >= t0 && p.t <= last);
}

function drawChart() {
  const { ctx, w, h } = setupCanvas(el.chart);
  ctx.clearRect(0, 0, w, h);
  const pts = windowPoints();
  el.chartEmpty.style.display = pts.length ? 'none' : 'grid';
  if (!pts.length) { chart.geom = null; return; }

  const padL = 46, padR = 46, padT = 18, padB = 30;
  const iw = w - padL - padR, ih = h - padT - padB;
  const t0 = pts[0].t, t1 = pts[pts.length - 1].t;
  const span = Math.max(1, t1 - t0);

  const [tLo, tHi] = domain(pts.map((p) => p.Temperature), 0.2);
  const [hLo, hHi] = domain(pts.map((p) => p.Humidity), 0.2, true);

  const xOf = (t) => padL + (pts.length === 1 ? iw / 2 : ((t - t0) / span) * iw);
  const yOfT = (v) => padT + (1 - (v - tLo) / (tHi - tLo)) * ih;
  const yOfH = (v) => padT + (1 - (v - hLo) / (hHi - hLo)) * ih;

  ctx.font = '10px "Spline Sans Mono", monospace';
  ctx.lineWidth = 1;

  const rows = 4;
  ctx.textBaseline = 'middle';
  for (let i = 0; i <= rows; i++) {
    const f = i / rows;
    const y = padT + f * ih;
    ctx.strokeStyle = 'rgba(148,178,224,0.09)';
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
    const tv = tHi - f * (tHi - tLo), hv = hHi - f * (hHi - hLo);
    ctx.fillStyle = 'rgba(255,180,84,0.65)'; ctx.textAlign = 'right';
    ctx.fillText(tv.toFixed(1), padL - 8, y);
    ctx.fillStyle = 'rgba(78,205,196,0.65)'; ctx.textAlign = 'left';
    ctx.fillText(Math.round(hv), w - padR + 8, y);
  }

  ctx.fillStyle = 'rgba(159,176,205,0.55)'; ctx.textAlign = 'center';
  const ticks = Math.min(5, Math.max(2, pts.length));
  const useSec = span < 120000;
  for (let i = 0; i < ticks; i++) {
    const t = t0 + (span * i) / (ticks - 1 || 1);
    ctx.fillText(useSec ? fmtHMS(t) : fmtHM(t), xOf(t), h - 12);
  }

  const tempPts = pts.filter((p) => isNum(p.Temperature)).map((p) => ({ x: xOf(p.t), y: yOfT(p.Temperature), v: p.Temperature, t: p.t }));
  const humiPts = pts.filter((p) => isNum(p.Humidity)).map((p) => ({ x: xOf(p.t), y: yOfH(p.Humidity), v: p.Humidity, t: p.t }));

  if (tempPts.length) {
    const grad = ctx.createLinearGradient(0, padT, 0, padT + ih);
    grad.addColorStop(0, 'rgba(255,180,84,0.28)'); grad.addColorStop(1, 'rgba(255,180,84,0)');
    ctx.beginPath(); traceSmooth(ctx, tempPts);
    ctx.lineTo(tempPts[tempPts.length - 1].x, padT + ih); ctx.lineTo(tempPts[0].x, padT + ih); ctx.closePath();
    ctx.fillStyle = grad; ctx.fill();
    ctx.beginPath(); traceSmooth(ctx, tempPts);
    ctx.strokeStyle = '#ffb454'; ctx.lineWidth = 2.2; ctx.shadowColor = 'rgba(255,180,84,0.5)'; ctx.shadowBlur = 8; ctx.stroke();
    ctx.shadowBlur = 0;
  }
  if (humiPts.length) {
    ctx.beginPath(); traceSmooth(ctx, humiPts);
    ctx.strokeStyle = '#4ecdc4'; ctx.lineWidth = 2; ctx.shadowColor = 'rgba(78,205,196,0.45)'; ctx.shadowBlur = 7; ctx.stroke();
    ctx.shadowBlur = 0;
  }

  const tail = (arr, color) => {
    if (!arr.length) return;
    const p = arr[arr.length - 1];
    ctx.beginPath(); ctx.arc(p.x, p.y, 3.4, 0, 7); ctx.fillStyle = color; ctx.fill();
    ctx.beginPath(); ctx.arc(p.x, p.y, 7, 0, 7); ctx.strokeStyle = color; ctx.globalAlpha = 0.35; ctx.stroke(); ctx.globalAlpha = 1;
  };
  tail(tempPts, '#ffb454'); tail(humiPts, '#4ecdc4');

  chart.geom = { padL, padR, padT, padB, iw, ih, t0, span, tempPts, humiPts, w, h };

  if (chart.hover >= 0 && chart.hover < pts.length) drawHover(pts);
}

function drawHover(pts) {
  const g = chart.geom, ctx = chart.ctx;
  if (!g) return;
  const p = pts[chart.hover];
  const x = g.padL + (pts.length === 1 ? g.iw / 2 : ((p.t - g.t0) / g.span) * g.iw);
  ctx.strokeStyle = 'rgba(238,244,255,0.22)'; ctx.lineWidth = 1; ctx.setLineDash([4, 4]);
  ctx.beginPath(); ctx.moveTo(x, g.padT); ctx.lineTo(x, g.padT + g.ih); ctx.stroke(); ctx.setLineDash([]);

  const parts = [fmtHMS(p.t)];
  if (isNum(p.Temperature)) parts.push(`<b style="color:#ffb454">${p.Temperature.toFixed(1)}°C</b>`);
  if (isNum(p.Humidity)) parts.push(`<b style="color:#4ecdc4">${p.Humidity.toFixed(0)}%</b>`);
  if (isNum(p.Light)) parts.push(`<span style="color:#ffe08a">${p.Light} lux</span>`);

  let anchorY = g.padT + 12;
  const tp = g.tempPts.find((q) => q.t === p.t);
  if (tp) anchorY = Math.max(g.padT + 8, tp.y);
  const tt = el.chartTooltip;
  tt.innerHTML = parts.join('　');
  tt.style.display = 'block';
  tt.style.left = `${Math.min(g.w - g.padR, Math.max(g.padL, x))}px`;
  tt.style.top = `${anchorY}px`;
}

el.chart.addEventListener('mousemove', (e) => {
  const pts = windowPoints();
  if (!pts.length || !chart.geom) return;
  const rect = el.chart.getBoundingClientRect();
  const g = chart.geom;
  const frac = Math.min(1, Math.max(0, (e.clientX - rect.left - g.padL) / g.iw));
  chart.hover = Math.round(frac * (pts.length - 1));
  drawChart();
});
el.chart.addEventListener('mouseleave', () => { chart.hover = -1; el.chartTooltip.style.display = 'none'; drawChart(); });

/* ---------- 迷你曲线 ---------- */
function drawSpark(canvas, key, color) {
  const { ctx, w, h } = setupCanvas(canvas);
  ctx.clearRect(0, 0, w, h);
  const pts = state.history.slice(-40).filter((p) => isNum(p[key]));
  if (pts.length < 1) return;
  const vals = pts.map((p) => p[key]);
  const [lo, hi] = domain(vals, 0.2);
  const yOf = (v) => 4 + (1 - (v - lo) / (hi - lo)) * (h - 8);
  const xOf = (i) => pts.length === 1 ? w / 2 : (i / (pts.length - 1)) * (w - 4) + 2;
  const xy = pts.map((p, i) => ({ x: xOf(i), y: yOf(p[key]) }));
  const grad = ctx.createLinearGradient(0, 0, 0, h);
  grad.addColorStop(0, color + '44'); grad.addColorStop(1, color + '00');
  ctx.beginPath(); traceSmooth(ctx, xy);
  ctx.lineTo(xy[xy.length - 1].x, h); ctx.lineTo(xy[0].x, h); ctx.closePath();
  ctx.fillStyle = grad; ctx.fill();
  ctx.beginPath(); traceSmooth(ctx, xy);
  ctx.strokeStyle = color; ctx.lineWidth = 1.8; ctx.stroke();
  const last = xy[xy.length - 1];
  ctx.beginPath(); ctx.arc(last.x, last.y, 2.4, 0, 7); ctx.fillStyle = color; ctx.fill();
}

/* ============================================================
   状态渲染
   ============================================================ */
function renderDevice() {
  const on = state.device && state.device.status === 'ONLINE';
  el.deviceDot.className = 'device-dot ' + (on ? 'on' : 'off');
  el.deviceStatus.textContent = on ? '设备在线' : (state.device && state.device.status ? '设备离线' : '设备状态未知');
}

function renderFreshness() {
  const age = state.lastEventTime ? (Date.now() - Date.parse(state.lastEventTime)) : null;
  const stale = age != null && age > STALE_MS;
  document.body.classList.toggle('stale', stale || state.current == null);
  el.freshVal.textContent = ageText(age);
  el.freshNote.textContent = age == null ? '等待设备首次上报…'
    : stale ? '已有一段时间未更新，设备可能离线' : '实时刷新中';
  el.freshTime.textContent = state.lastEventTime ? fmtHMS(Date.parse(state.lastEventTime)) : '—';
}

function renderDelta() {
  const h = state.history;
  if (h.length < 2 || !isNum(h[h.length - 1].Temperature) || !isNum(h[h.length - 2].Temperature)) {
    el.tempDelta.textContent = '—'; return;
  }
  const d = h[h.length - 1].Temperature - h[h.length - 2].Temperature;
  const arrow = d > 0.05 ? '▲' : d < -0.05 ? '▼' : '—';
  el.tempDelta.textContent = `${arrow} ${Math.abs(d).toFixed(1)}°C`;
}

function renderMinMax() {
  const hs = state.history.filter((p) => isNum(p.Humidity)).map((p) => p.Humidity);
  const ls = state.history.filter((p) => isNum(p.Light)).map((p) => p.Light);
  el.humiMin.textContent = `最低 ${hs.length ? Math.min(...hs).toFixed(0) + '%' : '—'}`;
  el.humiMax.textContent = `最高 ${hs.length ? Math.max(...hs).toFixed(0) + '%' : '—'}`;
  el.lightMin.textContent = `最低 ${ls.length ? Math.min(...ls) : '—'}`;
  el.lightMax.textContent = `最高 ${ls.length ? Math.max(...ls) : '—'}`;
}

function renderChartRange() {
  const pts = windowPoints();
  if (pts.length < 2) { el.chartRange.textContent = pts.length ? '仅 1 个数据点' : '—'; return; }
  el.chartRange.textContent = `${fmtHMS(pts[0].t)} → ${fmtHMS(pts[pts.length - 1].t)} · ${pts.length} 点`;
}

function render() {
  const c = state.current;
  if (c) {
    animateValue(el.tempHero, c.Temperature, 1);
    animateValue(el.humiVal, c.Humidity, 1);
    animateValue(el.lightVal, c.Light, 0);
    el.tempDesc.textContent = tempWord(c.Temperature);
  }
  renderDevice(); renderFreshness(); renderDelta(); renderMinMax(); renderChartRange();
  drawChart();
  drawSpark(el.sparkHumi, 'Humidity', '#4ecdc4');
  drawSpark(el.sparkLight, 'Light', '#ffe08a');
}

/* ============================================================
   WebSocket
   ============================================================ */
let ws = null, retry = 1500;
function connect() {
  const url = (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws';
  ws = new WebSocket(url);
  ws.onopen = () => {
    retry = 1500;
    el.connDot.className = 'conn-dot live'; el.connLabel.textContent = '实时连接';
    el.footConn.textContent = 'WebSocket 已连接';
  };
  ws.onmessage = (ev) => {
    try {
      const d = JSON.parse(ev.data);
      state.current = d.current || null;
      state.history = Array.isArray(d.history) ? d.history : [];
      state.device = d.device || {};
      state.lastEventTime = d.lastEventTime || null;
      render();
    } catch (_) {}
  };
  ws.onclose = () => {
    el.connDot.className = 'conn-dot dead'; el.connLabel.textContent = '连接断开，重试中';
    el.footConn.textContent = 'WebSocket 断开';
    setTimeout(connect, retry); retry = Math.min(retry * 1.6, 10000);
  };
  ws.onerror = () => { try { ws.close(); } catch (_) {} };
}
connect();

/* ============================================================
   时钟 + 时效心跳
   ============================================================ */
setInterval(() => {
  el.clock.textContent = fmtClock(new Date());
  renderFreshness();
}, 1000);

window.addEventListener('resize', () => drawChart());
if (window.ResizeObserver) new ResizeObserver(() => drawChart()).observe(el.chart.parentElement);

/* ============================================================
   迷你遥控（按住行驶 / 松开即停）
   ============================================================ */
async function sendCommand(dir, duration = 0) {
  try {
    await fetch('/api/command', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ dir, duration }),
    });
  } catch (_) {}
}

const dock = $('dock'), dockToggle = $('dock-toggle'), dockPad = $('dock-pad');
dockToggle.addEventListener('click', () => {
  const open = dock.classList.toggle('open');
  dockPad.hidden = !open;
});

let heldDir = null;
function press(btn) {
  const dir = btn.dataset.dir;
  if (dir === 'STOP') { heldDir = null; sendCommand('STOP', 0); return; }
  heldDir = dir; btn.classList.add('held'); sendCommand(dir, 0);
}
function release(btn) {
  btn.classList.remove('held');
  if (heldDir) { heldDir = null; sendCommand('STOP', 0); }
}
document.querySelectorAll('.dp').forEach((btn) => {
  btn.addEventListener('pointerdown', (e) => { e.preventDefault(); btn.setPointerCapture(e.pointerId); press(btn); });
  ['pointerup', 'pointercancel'].forEach((t) => btn.addEventListener(t, () => release(btn)));
});
window.addEventListener('blur', () => { if (heldDir) { heldDir = null; sendCommand('STOP', 0); } });

const KEYMAP = { ArrowUp: 'FWD', ArrowDown: 'BACK', ArrowLeft: 'LEFT', ArrowRight: 'RIGHT' };
let keyHeld = [];
const dirBtn = (d) => document.querySelector(`.dp[data-dir="${d}"]`);
document.addEventListener('keydown', (e) => {
  if (e.repeat) return;
  if (KEYMAP[e.key]) {
    e.preventDefault();
    const d = KEYMAP[e.key];
    if (!keyHeld.includes(d)) keyHeld.push(d);
    const b = dirBtn(d); if (b) b.classList.add('held');
    sendCommand(d, 0);
  } else if (e.code === 'Space') {
    e.preventDefault(); keyHeld = [];
    document.querySelectorAll('.dp.held').forEach((b) => b.classList.remove('held'));
    sendCommand('STOP', 0);
  }
});
document.addEventListener('keyup', (e) => {
  if (!KEYMAP[e.key]) return;
  e.preventDefault();
  const d = KEYMAP[e.key];
  keyHeld = keyHeld.filter((x) => x !== d);
  const b = dirBtn(d); if (b) b.classList.remove('held');
  sendCommand(keyHeld[keyHeld.length - 1] || 'STOP', 0);
});

render();

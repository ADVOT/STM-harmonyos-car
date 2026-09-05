'use strict';

require('dotenv').config();
const express = require('express');
const path = require('path');
const http = require('http');
const { WebSocketServer, WebSocket } = require('ws');
const hw = require('./huawei');

const app = express();
app.use(express.json());
app.use(express.static(path.join(__dirname, 'public')));

const VALID_DIR = new Set(['FWD', 'BACK', 'LEFT', 'RIGHT', 'STOP']);
const MAX_DURATION = 60000;

function httpStatus(r) {
  if (r.ok) return 200;
  return r.status && r.status >= 400 && r.status < 600 ? r.status : 502;
}

/* ==================== 影子轮询 + 实时遥测状态 ====================
   固件每 5s 上报一次 Environment，这里以 SHADOW_POLL_MS 高频拉影子，
   一旦 reported.event_time 变化即视为新一帧数据，追加进环形历史并广播。
   设备在线状态另以较低频率拉 showDevice（省 API 调用）。 */
const SHADOW_POLL_MS = 3000;
const STATUS_POLL_MS = 10000;
const MAX_HISTORY = 1440;   // 5s/帧时约 2 小时

const state = {
  current: null,          // { Temperature, Humidity, Light }
  lastEventTime: null,    // ISO 字符串（标准化后）
  lastEventMs: null,      // epoch ms
  device: { status: null, device_name: null, connection_update_time: null },
  history: [],            // [{ t, Temperature, Humidity, Light }]
  cloudError: null,       // 最近一次拉取错误（调试用）
};

// 华为云影子 event_time 形如 "20260903T074155Z"（紧凑 ISO），Date.parse 不认，先标准化
function parseEventTime(s) {
  if (!s || typeof s !== 'string') return null;
  const m = s.match(/^(\d{4})(\d{2})(\d{2})T(\d{2})(\d{2})(\d{2})/);
  if (m) return `${m[1]}-${m[2]}-${m[3]}T${m[4]}:${m[5]}:${m[6]}Z`;
  return s;
}

// 在影子对象里递归找 service_id 的 reported.properties
function findEnv(shadow) {
  if (!shadow || typeof shadow !== 'object') return null;
  const list = Array.isArray(shadow) ? shadow : (shadow.shadow || []);
  for (const svc of list) {
    if (svc && svc.service_id === 'Environment') {
      const rep = (svc.reported && svc.reported.properties) || svc.properties || null;
      return { props: rep, eventTime: (svc.reported && svc.reported.event_time) || svc.event_time || null };
    }
  }
  return null;
}

function num(v) {
  if (v === undefined || v === null) return null;
  const n = Number(v);
  return Number.isFinite(n) ? n : null;
}

function snapshot() {
  const now = Date.now();
  return {
    type: 'snapshot',
    current: state.current,
    lastEventTime: state.lastEventTime,
    dataAgeMs: state.lastEventMs ? now - state.lastEventMs : null,
    device: state.device,
    history: state.history,
    pollIntervalMs: SHADOW_POLL_MS,
    cloudError: state.cloudError,
  };
}

function broadcast(payload) {
  const msg = JSON.stringify(payload);
  for (const client of wss.clients) {
    if (client.readyState === WebSocket.OPEN) client.send(msg);
  }
}

async function pollShadow() {
  try {
    const r = await hw.getShadow();
    if (!r.ok) { state.cloudError = r.error_code || r.error_msg || 'shadow fail'; return; }
    state.cloudError = null;
    const env = findEnv(r.data);
    if (!env || !env.props) return;
    const iso = parseEventTime(env.eventTime);
    const ms = iso ? Date.parse(iso) : null;
    // 只有 event_time 变化才算新数据，避免重复帧
    if (iso && iso === state.lastEventTime) return;
    const point = {
      t: ms || Date.now(),
      Temperature: num(env.props.Temperature),
      Humidity: num(env.props.Humidity),
      Light: num(env.props.Light),
    };
    state.current = { Temperature: point.Temperature, Humidity: point.Humidity, Light: point.Light };
    state.lastEventTime = iso;
    state.lastEventMs = ms;
    state.history.push(point);
    if (state.history.length > MAX_HISTORY) state.history.splice(0, state.history.length - MAX_HISTORY);
    broadcast(Object.assign({}, snapshot(), { type: 'update' }));
    console.log(`[telemetry] T=${point.Temperature} H=${point.Humidity} L=${point.Light} @ ${iso}`);
  } catch (e) {
    state.cloudError = e.message || String(e);
  }
}

async function pollStatus() {
  try {
    const r = await hw.getDeviceStatus();
    if (r.ok && r.data) state.device = r.data;
    else state.device = Object.assign({}, state.device, { status: null });
  } catch (_) { /* 忽略，下轮再试 */ }
}

/* ==================== HTTP 路由 ==================== */

// 网页 -> 后端 -> 华为云 IoTDA -> 小车
app.post('/api/command', async (req, res) => {
  const raw = (req.body && req.body.dir) || '';
  const dir = String(raw).toUpperCase();
  if (!VALID_DIR.has(dir)) {
    return res.status(400).json({ ok: false, error_msg: `无效方向: ${raw}` });
  }
  let ms = parseInt(req.body && req.body.duration, 10);
  if (Number.isNaN(ms) || ms < 0) ms = 1000;
  if (ms > MAX_DURATION) ms = MAX_DURATION;
  if (dir === 'STOP') ms = 0;

  const r = await hw.sendCommand(dir, ms);
  console.log(`[cmd] ${dir} ${ms}ms -> ok=${r.ok} status=${r.status}${r.error_code ? ' ' + r.error_code : ''}`);
  res.status(httpStatus(r)).json(Object.assign({}, r, { dir, duration: ms }));
});

// 阶段二：查设备影子回显遥测（原始影子，兼容旧前端）
app.get('/api/status', async (req, res) => {
  const r = await hw.getShadow();
  res.status(httpStatus(r)).json(r);
});

// 归一化遥测快照（HTTP 兜底 / 调试）
app.get('/api/telemetry', (req, res) => {
  res.json(Object.assign({}, snapshot(), { ok: true }));
});

/* ==================== 启动 ==================== */
const PORT = process.env.PORT || 3000;
const server = http.createServer(app);
const wss = new WebSocketServer({ server, path: '/ws' });

wss.on('connection', (ws) => {
  console.log(`[ws] client connected (total ${wss.clients.size})`);
  ws.send(JSON.stringify(snapshot()));
  ws.on('close', () => console.log(`[ws] client left (total ${wss.clients.size})`));
});

server.listen(PORT, () => {
  console.log(`cloud-car-server 已启动: http://localhost:${PORT}  (WS: ws://localhost:${PORT}/ws)`);
  pollShadow();
  pollStatus();
  setInterval(pollShadow, SHADOW_POLL_MS);
  setInterval(pollStatus, STATUS_POLL_MS);
});

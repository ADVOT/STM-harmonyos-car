'use strict';

require('dotenv').config();
const express = require('express');
const path = require('path');
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

// 阶段二：查设备影子回显遥测
app.get('/api/status', async (req, res) => {
  const r = await hw.getShadow();
  res.status(httpStatus(r)).json(r);
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`cloud-car-server 已启动: http://localhost:${PORT}`);
});

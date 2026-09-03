'use strict';

const logEl = document.getElementById('log');
const shadowEl = document.getElementById('shadow');
const refreshBtn = document.getElementById('refresh');
const autoBox = document.getElementById('auto');

function ts() {
  return new Date().toLocaleTimeString('zh-CN', { hour12: false });
}

function log(msg) {
  logEl.textContent = `[${ts()}] ${msg}\n` + logEl.textContent;
}

// duration=0 → 持续模式（按住一直走，固件每 10ms 续租）；>0 → 定时
async function sendCommand(dir, duration = 0) {
  try {
    const res = await fetch('/api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ dir, duration }),
    });
    const data = await res.json();
    const tag = duration > 0 ? `${duration}ms` : (dir === 'STOP' ? '停止' : '持续');
    if (data.ok) {
      log(`✔ ${dir} ${tag} 已下发（HTTP ${data.status}）`);
    } else {
      log(`✘ ${dir} 下发失败 HTTP ${data.status || '?'} ${data.error_code || ''} ${data.error_msg || ''}`.trim());
    }
  } catch (e) {
    log(`✘ ${dir} 请求异常：${e.message}`);
  }
}

/* ===== 按住持续 / 松开停（赛车手感）=====
   键盘与鼠标各自记录按住的方向；currentTop 取当前应生效的方向，
   松开时回落到另一个仍按住的方向，全松则 STOP。 */
let heldKeys = [];      // 键盘按住的方向栈（最新在末尾）
let mouseDir = null;    // 鼠标/触摸按住的方向

function currentTop() {
  return mouseDir || heldKeys[heldKeys.length - 1] || null;
}
function applyHeld() {
  sendCommand(currentTop() || 'STOP', 0);
}
function allStop() {
  heldKeys = [];
  mouseDir = null;
  sendCommand('STOP', 0);
}

function keyDown(dir) {
  if (!heldKeys.includes(dir)) heldKeys.push(dir);
  applyHeld();
}
function keyUp(dir) {
  heldKeys = heldKeys.filter((d) => d !== dir);
  applyHeld();
}
function mouseDown(dir) {
  mouseDir = dir;
  applyHeld();
}
function mouseUp() {
  if (mouseDir !== null) { mouseDir = null; applyHeld(); }
}

// 在设备影子里递归找指定 service_id 的上报属性（兼容 reported/desired/直接 properties）
function findServiceProps(obj, serviceId) {
  if (!obj || typeof obj !== 'object') return null;
  if (obj.service_id === serviceId) {
    const rep = obj.reported || obj.desired || {};
    if (rep && typeof rep === 'object' && rep.properties) return rep.properties;
    if (obj.properties) return obj.properties;
  }
  for (const k of Object.keys(obj)) {
    const r = findServiceProps(obj[k], serviceId);
    if (r) return r;
  }
  return null;
}

function fmt(v) {
  if (v === undefined || v === null) return '—';
  return (typeof v === 'number') ? (Number.isInteger(v) ? String(v) : v.toFixed(1)) : String(v);
}

async function refreshStatus() {
  try {
    const res = await fetch('/api/status');
    const data = await res.json();
    if (data.ok) {
      const env = findServiceProps(data.data, 'Environment');
      if (env) {
        shadowEl.textContent =
          `温度 Temperature: ${fmt(env.Temperature)} ℃\n` +
          `湿度 Humidity:    ${fmt(env.Humidity)} %\n` +
          `光照 Light:       ${fmt(env.Light)}`;
      } else {
        shadowEl.textContent =
          '暂无 Environment 遥测（设备未上报 / 云模型未建该 service）\n\n' +
          JSON.stringify(data.data, null, 2);
      }
    } else {
      shadowEl.textContent = `获取失败 HTTP ${data.status || '?'} ${data.error_code || ''} ${data.error_msg || ''}`.trim();
    }
  } catch (e) {
    shadowEl.textContent = '获取失败：' + e.message;
  }
}

// 方向按钮：按住持续走，松开/移出停；停止按钮：立即全停
document.querySelectorAll('button.dir').forEach((btn) => {
  const dir = btn.dataset.dir;
  if (dir === 'STOP') {
    btn.addEventListener('click', (e) => { e.preventDefault(); allStop(); });
    return;
  }
  btn.addEventListener('mousedown', (e) => { e.preventDefault(); mouseDown(dir); });
  btn.addEventListener('touchstart', (e) => { e.preventDefault(); mouseDown(dir); }, { passive: false });
  btn.addEventListener('mouseleave', () => mouseUp());
  btn.addEventListener('touchend', (e) => { e.preventDefault(); mouseUp(); });
  btn.addEventListener('touchcancel', () => mouseUp());
});
// 全局兜底：在按钮外松开鼠标也要停
window.addEventListener('mouseup', () => mouseUp());

refreshBtn.addEventListener('click', refreshStatus);

let timer = null;
autoBox.addEventListener('change', () => {
  if (autoBox.checked) {
    refreshStatus();
    timer = setInterval(refreshStatus, 2000);
  } else if (timer) {
    clearInterval(timer);
    timer = null;
  }
});

// 键盘方向键：按住持续走，松开停；空格=立即全停
const KEYMAP = { ArrowUp: 'FWD', ArrowDown: 'BACK', ArrowLeft: 'LEFT', ArrowRight: 'RIGHT' };
document.addEventListener('keydown', (e) => {
  if (e.repeat) return; // 忽略系统自动重复，靠"持续模式"保持前进
  if (KEYMAP[e.key]) { e.preventDefault(); keyDown(KEYMAP[e.key]); }
  else if (e.code === 'Space') { e.preventDefault(); allStop(); }
});
document.addEventListener('keyup', (e) => {
  if (KEYMAP[e.key]) { e.preventDefault(); keyUp(KEYMAP[e.key]); }
});
// 失焦时全部停，避免切窗口后车还在跑
window.addEventListener('blur', () => { if (currentTop()) allStop(); });

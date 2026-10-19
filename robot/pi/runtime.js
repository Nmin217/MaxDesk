'use strict';

const fs = require('fs');
const path = require('path');
const http = require('http');
const { URL } = require('url');

// Fake DOM so the simulator core runs headlessly. Mirrors bridge/bridge_sim.js.
global.els = {};
const mkctx = () => new Proxy({}, {
  get: (t, p) => p in t ? t[p] : (() => {}),
  set: (t, p, v) => { t[p] = v; return true; }
});
function el(id) {
  return global.els[id] || (global.els[id] = {
    id, width: 1600, height: 1000,
    style: {}, dataset: {}, value: '1', checked: false,
    textContent: '', className: '',
    getContext: () => mkctx(),
    addEventListener() {}, appendChild() {}, setPointerCapture() {},
    getBoundingClientRect: () => ({ left: 0, top: 0, width: 800, height: 500 })
  });
}
if (!global.document) {
  global.document = { getElementById: el, documentElement: {}, querySelectorAll: () => [] };
  global.getComputedStyle = () => ({ getPropertyValue: () => '#888' });
  global.matchMedia = () => ({ matches: false, addEventListener() {} });
  global.addEventListener = () => {};
  global.devicePixelRatio = 1;
  global.performance = { now: () => Date.now() };
  global.requestAnimationFrame = () => {};
  global.localStorage = { getItem: () => null, setItem() {}, removeItem() {} };
}

const PX = 0.02; // 1 px = 0.02 m, same as simulator and bridge

function loadSim() {
  const simPath = path.join(__dirname, '..', '..', 'bridge', 'sim_core.js');
  const code = fs.readFileSync(simPath, 'utf8');
  const wrapped = code + `
    ;return {
      update: update, reset: reset, startReturn: startReturn, startTour: startTour,
      getState: function () {
        return {
          T: T, active: active, estop: estop, cordIn: cordIn,
          user: user, people: people, hits: hits, bat: bat,
          mode: mode, parkTh: parkTh, time: time, path: path,
          goal: goal, manualDrive: manualDrive, alertT: alertT, deskH: deskH
        };
      },
      setUser: function (v) { user.x = v.x; user.y = v.y; },
      setEstop: function (v) { estop = v; },
      setCord: function (v) { cordIn = v; },
      setManualDrive: function (v) { manualDrive = v; },
      setStatusText: function (s) { els['st'].textContent = s; },
      getStatusText: function () { return els['st'].textContent; },
      clickGo: function () { els['go'].onclick(); },
      clickBack: function () { els['back'].onclick(); },
      clickStop: function () { els['stop'].onclick(); },
      clickLearn: function () { els['learn'].onclick(); }
    };
  `;
  return new Function(wrapped)();
}

function httpJson(method, urlStr, token, body) {
  return new Promise((resolve, reject) => {
    const u = new URL(urlStr);
    const payload = body === undefined ? null : Buffer.from(JSON.stringify(body));
    const req = http.request({
      hostname: u.hostname, port: u.port, path: u.pathname, method,
      headers: {
        'Authorization': 'Bearer ' + token,
        'Content-Type': 'application/json',
        ...(payload ? { 'Content-Length': payload.length } : {})
      }
    }, (res) => {
      let chunks = '';
      res.on('data', (c) => chunks += c);
      res.on('end', () => {
        let parsed = null;
        try { parsed = chunks ? JSON.parse(chunks) : null; } catch (_) {}
        resolve({ status: res.statusCode, body: parsed });
      });
    });
    req.on('error', reject);
    if (payload) req.write(payload);
    req.end();
  });
}

function createRuntime({
  io, bridgeUrl, token, hz = 60,
  nowMs = () => Date.now(),
  statePushMs = 350, pollMs = 350, staleBridgeMs = 2000
} = {}) {
  if (!io) throw new Error('io is required');
  if (!bridgeUrl) throw new Error('bridgeUrl is required');
  if (!token) throw new Error('token is required');
  if (!Number.isFinite(hz) || hz <= 0) throw new Error('hz must be positive');

  const sim = loadSim();
  let connected = false;
  let lastStatePush = 0;
  let lastPoll = 0;
  let lastBridgeOk = 0;
  let timer = null;
  let running = false;

  async function healthCheck() {
    try {
      const r = await httpJson('GET', bridgeUrl + '/v1/health', token);
      return r.status === 200 && r.body && r.body.ok === true;
    } catch (_) { return false; }
  }

  async function pushStateNow() {
    const s = sim.getState();
    const payload = {
      time: s.time, mode: s.mode, status: sim.getStatusText(), severity: 'info',
      active: !!s.active, estop: !!s.estop,
      pose: { x: +(s.T.x * PX).toFixed(3), y: +(s.T.y * PX).toFixed(3), theta: +s.T.th.toFixed(3) },
      user: { x: +(s.user.x * PX).toFixed(3), y: +(s.user.y * PX).toFixed(3) },
      battery: { robot_pct: +s.bat.r.toFixed(1), ups_pct: +s.bat.a.toFixed(1) },
      charging: false,
      cord: { plugged: !!s.cordIn, interlock_on: true, breakaways: 0 },
      dock_type: 'manual', desk_height_cm: Math.round(s.deskH * 100),
      alert: s.alertT > 0, explored_pct: 0, remembered_cells: 0,
      people: s.people.map(p => ({
        x: +(p.x * PX).toFixed(2), y: +(p.y * PX).toFixed(2), settled: !!p.settled
      }))
    };
    try {
      const r = await httpJson('POST', bridgeUrl + '/v1/sim/state', token, payload);
      if (r.status === 200) lastBridgeOk = nowMs();
      return r.status === 200;
    } catch (_) { return false; }
  }

  function applyCommand(c) {
    if (!c || typeof c.type !== 'string') return;
    if (c.type === 'summon') {
      sim.setUser({ x: c.x / PX, y: c.y / PX, r: 13 });
      sim.clickGo();
    } else if (c.type === 'return') sim.clickBack();
    else if (c.type === 'stop') sim.clickStop();
    else if (c.type === 'learn') sim.clickLearn();
    else if (c.type === 'cord') sim.setCord(!!c.plugged);
    else if (c.type === 'drive') sim.setManualDrive({
      enabled: true, x: c.x, y: c.y, turn: c.turn, speed: c.speed,
      lastCommandAt: nowMs()
    });
  }

  async function pollNow() {
    try {
      const r = await httpJson('GET', bridgeUrl + '/v1/sim/poll', token);
      if (r.status !== 200 || !r.body) return false;
      lastBridgeOk = nowMs();
      const cmds = Array.isArray(r.body.commands) ? r.body.commands : [];
      for (const c of cmds) applyCommand(c);
      return true;
    } catch (_) { return false; }
  }

  function bridgeFresh() {
    return connected && (nowMs() - lastBridgeOk) < staleBridgeMs;
  }

  async function tickOnce(dt) {
    if (dt <= 0) return;
    const fresh = bridgeFresh();
    const estop = io.estopEngaged();
    if (estop) sim.setEstop(true);
    sim.update(dt);
    const s = sim.getState();
    const moving = Math.hypot(s.T.vx, s.T.vy) > 0.01 || Math.abs(s.T.w) > 0.01;
    if (estop || !fresh) { io.stopAll(); return; }
    if (moving || s.active) io.writeWheels(s.T.vx, s.T.vy, s.T.w);
    else io.stopAll();
  }

  async function loop() {
    if (!running) return;
    const t = nowMs();
    if (t - lastStatePush >= statePushMs) { lastStatePush = t; await pushStateNow(); }
    if (t - lastPoll >= pollMs) { lastPoll = t; await pollNow(); }
    await tickOnce(1 / hz);
  }

  return {
    async start() {
      const ok = await healthCheck();
      if (!ok) { connected = false; io.stopAll(); return false; }
      connected = true;
      lastBridgeOk = nowMs();
      running = true;
      if (!timer) timer = setInterval(() => { loop().catch(() => {}); }, Math.round(1000 / hz));
      return true;
    },
    stop() {
      running = false; connected = false;
      if (timer) { clearInterval(timer); timer = null; }
      io.stopAll();
    },
    get connected() { return connected; },
    get sim() { return sim; },
    tickOnce, pollNow, pushStateNow, healthCheck
  };
}

module.exports = { createRuntime };
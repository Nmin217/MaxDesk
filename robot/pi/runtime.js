'use strict';

const fs = require('fs');
const os = require('os');
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
    // Select values default to '1' and checkboxes to off, except: the desk uses a manual charging
    // cord ('dm') with the cord interlock on ('il'). Without these, cordLock() in the sim never engages.
    style: {}, dataset: {}, value: id === 'dm' ? 'manual' : '1', checked: id === 'il',
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
// Hard ceiling on anything sent to writeWheels, whatever the planner asks for (sim VMAX is ~0.8 m/s).
const MAX_V = 1.0;   // m/s
const MAX_W = 2.0;   // rad/s
const HTTP_TIMEOUT_MS = 1500;

// Sim velocity (px/s, screen frame: y down, theta clockwise) -> IO field frame (m/s, y up, omega CCW+),
// clamped to MAX_V / MAX_W. See io.js for the frame.
function toField(vx, vy, w) {
  let fx = vx * PX, fy = -vy * PX, fw = -w;
  const sp = Math.hypot(fx, fy);
  if (!(sp <= MAX_V)) { const k = sp > 0 && Number.isFinite(sp) ? MAX_V / sp : 0; fx *= k; fy *= k; }   // NaN -> 0
  fw = Number.isFinite(fw) ? Math.max(-MAX_W, Math.min(MAX_W, fw)) : 0;
  return { vx: fx, vy: fy, omega: fw };
}

// The sim core keeps the room map and no-go zones in localStorage. On the Pi that is a folder of
// small files, so they survive power-off. Power can be cut at any moment, so each write goes to a
// temp file, is flushed to disk, then renamed over the old one: a cut mid-write leaves the previous
// version, never a half-written file. dataDir null = keep everything in memory (forgotten at exit).
function createStorage(dataDir) {
  const mem = new Map();
  if (dataDir === null) {
    return { getItem: k => mem.has(k) ? mem.get(k) : null, setItem: (k, v) => mem.set(k, String(v)), removeItem: k => mem.delete(k) };
  }
  fs.mkdirSync(dataDir, { recursive: true });
  const file = k => path.join(dataDir, encodeURIComponent(k) + '.txt');
  return {
    getItem(k) {
      try { return fs.readFileSync(file(k), 'utf8'); } catch (_) { return null; }
    },
    setItem(k, v) {
      const f = file(k), tmp = f + '.tmp';
      const fd = fs.openSync(tmp, 'w');
      try { fs.writeSync(fd, String(v)); fs.fsyncSync(fd); } finally { fs.closeSync(fd); }
      fs.renameSync(tmp, f);
    },
    removeItem(k) {
      try { fs.unlinkSync(file(k)); } catch (_) {}
    }
  };
}

function loadSim() {
  const simPath = path.join(__dirname, '..', '..', 'bridge', 'sim_core.js');
  const code = fs.readFileSync(simPath, 'utf8');
  const wrapped = code + `
    ;return {
      update: update, reset: reset, startReturn: startReturn, startTour: startTour,
      saveMap: function () { saveMap(); },
      getExtras: function () { return extraStateJSON(); },
      setZones: function (z) { return setZonesM(z); },
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
      res.on('error', reject);
      let chunks = '';
      res.on('data', (c) => chunks += c);
      res.on('end', () => {
        let parsed = null;
        try { parsed = chunks ? JSON.parse(chunks) : null; } catch (_) {}
        resolve({ status: res.statusCode, body: parsed });
      });
    });
    req.on('error', reject);
    // A hung bridge must not leak a socket per poll: abort after HTTP_TIMEOUT_MS of inactivity.
    req.setTimeout(HTTP_TIMEOUT_MS, () => req.destroy(new Error('http timeout')));
    if (payload) req.write(payload);
    req.end();
  });
}

function createRuntime({
  io, bridgeUrl, token, hz = 60,
  dataDir = process.env.MAXDESK_DATA_DIR || path.join(os.homedir(), '.maxdesk'),   // room map + no-go zones
  nowMs = () => Date.now(),
  statePushMs = 100, pollMs = 100, staleBridgeMs = 2000   // 100 ms: a Stop reaches the wheels within ~0.1 s plus Wi-Fi
} = {}) {
  if (!io) throw new Error('io is required');
  if (!bridgeUrl) throw new Error('bridgeUrl is required');
  if (!token) throw new Error('token is required');
  if (!Number.isFinite(hz) || hz <= 0) throw new Error('hz must be positive');

  global.localStorage = createStorage(dataDir);
  const sim = loadSim();
  let connected = false;
  let lastStatePush = 0;
  let lastPoll = 0;
  let lastBridgeOk = 0;
  let timer = null;
  let running = false;
  let hwEstopLatched = false;   // we set the sim e-stop because the hardware one was engaged
  let hwCordPlugged = null;     // last hardware cord state pushed into the sim (null = never read)

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
      })),
      ...sim.getExtras()   // keep-out zones, battery plan, alert text, trip log
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
    else if (c.type === 'zones') sim.setZones(c.zones);
    else if (c.type === 'drive') sim.setManualDrive({
      enabled: true, x: c.x, y: c.y, turn: c.turn, speed: c.speed,
      lastCommandAt: Date.now()   // the sim's 750 ms deadman compares against Date.now(), not nowMs
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
    try {
      const fresh = bridgeFresh();
      let estop;
      try { estop = !!io.estopEngaged(); } catch (_) { estop = true; }   // unreadable e-stop = engaged
      if (estop) { sim.setEstop(true); hwEstopLatched = true; }
      else if (hwEstopLatched) { sim.setEstop(false); hwEstopLatched = false; }   // released: the sim left active=false, so no auto-resume
      // Cord: {plugged, interlock_on}. Plugged with interlock on = no wheel command. Unreadable = locked.
      let cordLocked;
      try {
        const c = io.readCord();
        cordLocked = !c || (!!c.plugged && !!c.interlock_on);
        if (c && hwCordPlugged !== !!c.plugged) { hwCordPlugged = !!c.plugged; sim.setCord(hwCordPlugged); }   // on change only, so bridge 'cord' commands still work
      } catch (_) { cordLocked = true; }
      sim.update(dt);
      const s = sim.getState();
      if (estop || !fresh || cordLocked) { io.stopAll(); return; }
      const moving = Math.hypot(s.T.vx, s.T.vy) > 0.01 || Math.abs(s.T.w) > 0.01;
      if (moving || s.active) { const v = toField(s.T.vx, s.T.vy, s.T.w); io.writeWheels(v.vx, v.vy, v.omega); }
      else io.stopAll();
    } catch (_) {
      try { io.stopAll(); } catch (_) {}   // any failure in the tick leaves the wheels stopped
    }
  }

  // Network calls run beside the control loop, one of each in flight at a time: a slow or hung
  // request must never hold up the wheel update (the stale-bridge check stops the wheels instead).
  let pushing = false, polling = false;
  async function loop() {
    if (!running) return;
    const t = nowMs();
    if (!pushing && t - lastStatePush >= statePushMs) {
      lastStatePush = t; pushing = true;
      pushStateNow().catch(() => {}).finally(() => { pushing = false; });
    }
    if (!polling && t - lastPoll >= pollMs) {
      lastPoll = t; polling = true;
      pollNow().catch(() => {}).finally(() => { polling = false; });
    }
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
      try { sim.saveMap(); } catch (_) {}   // the sim also saves every 10 s; this catches the last few
    },
    get connected() { return connected; },
    get sim() { return sim; },
    tickOnce, pollNow, pushStateNow, healthCheck
  };
}

module.exports = { createRuntime, createStorage };
'use strict';

const fs = require('fs');
const os = require('os');
const path = require('path');
const http = require('http');
const { URL } = require('url');
const { createHardwareMonitor } = require('./hardware_check');
const { createBlackBox } = require('./black_box');
const { createSlewLimiter, createJamDetector } = require('./motion_guard');

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
// The desk's own outline in its frame (m): LiDAR returns inside it are its own legs and frame, not obstacles.
const DESK_HALF_FWD = 0.36 + 0.03, DESK_HALF_LEFT = 0.56 + 0.03;
const LIDAR_HOLD_MS = 300, LIDAR_FAULT_MS = 2000;   // older scan: hold the wheels / call it a hardware fault
const wrapPi = a => a - 2 * Math.PI * Math.floor((a + Math.PI) / (2 * Math.PI));
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
      jamStop: function (text) { jamStop(text); },
      setHardwareFault: function (text) { setHardwareFault(text); },
      setHardwareInput: function (on) { setHardwareInput(on); },
      hwOdometry: function (dx, dy, dth) { hwOdometry(dx, dy, dth); },
      hwScan: function (rays) { hwScan(rays); },
      hwBattery: function (b) { hwBattery(b); },
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
  statePushMs = 100, pollMs = 100, staleBridgeMs = 2000,   // 100 ms: a Stop reaches the wheels within ~0.1 s plus Wi-Fi
  autoLoop = true,   // false: the caller runs tickOnce/pollNow itself (tests that step a fake world in lockstep)
  blackBox = true,   // record the last 60 s and save it to <dataDir>/blackbox when something goes wrong (black_box.js)
  expectedModules = (process.env.MAXDESK_MODULES || 'FL,FR,RL,RR').split(',').map(x => x.trim()).filter(Boolean)   // the hardware check expects these
} = {}) {
  if (!io) throw new Error('io is required');
  if (!bridgeUrl) throw new Error('bridgeUrl is required');
  // Same computer as the bridge: use its pairing key (bridge/pairing.js) unless one is passed in.
  if (!token) token = process.env.SUMMON_TOKEN || '';
  if (!token && dataDir !== null) { try { token = fs.readFileSync(path.join(dataDir, 'bridge-token.txt'), 'utf8').trim(); } catch (_) {} }
  if (!token) throw new Error('token is required (pass one, set SUMMON_TOKEN, or start the bridge once so it makes bridge-token.txt)');
  if (!Number.isFinite(hz) || hz <= 0) throw new Error('hz must be positive');

  global.localStorage = createStorage(dataDir);
  const sim = loadSim();
  const slew = createSlewLimiter();
  const jam = createJamDetector();
  const hw = createHardwareMonitor(expectedModules);
  let hwCheckT = 0, hwState = { blocked: false, text: '' };
  // Hardware input: an IO that drives real motors (pi_io.js) also feeds the brain real sensors instead of its pretend room.
  const hwMode = !!io.isHardware;
  if (hwMode) sim.setHardwareInput(true);
  let lastPose = null, lastScanSeq, lidarAgeMs = Infinity, batT = 0;
  // Black box: what the desk did over the last minute, saved when something goes wrong (view: tools/blackbox_viewer.html).
  const bb = blackBox && dataDir !== null ? createBlackBox({ dir: path.join(dataDir, 'blackbox'), nowMs }) : null;
  const BAD_TRIPS = new Set(['jam', 'hardware', 'e-stop', 'blocked', 'cord']);
  let bbPrev = { estop: false, hw: false, stale: false, lost: false, status: '', trip: null }, bbExtras = null, bbExtrasT = 0;
  function blackBoxStep(dt, out, mods, flags) {
    if (!bb) return;
    try {
      const s = sim.getState(), status = sim.getStatusText();
      if ((bbExtrasT -= dt) <= 0) { bbExtrasT = 0.5; bbExtras = sim.getExtras(); }
      const loc = bbExtras && bbExtras.localization || {}, trip = bbExtras && bbExtras.trips && bbExtras.trips[0];
      bb.record({
        x: +(s.T.x * PX).toFixed(3), y: +(s.T.y * PX).toFixed(3), th: +s.T.th.toFixed(3),   // map metres, y down (as the floor plan)
        vx: +(s.T.vx * PX).toFixed(3), vy: +(s.T.vy * PX).toFixed(3), w: +s.T.w.toFixed(3),
        out, mode: s.mode, active: !!s.active, loc: loc.state || '', estop: flags.estop, cord: flags.cord, bridge: flags.fresh,
        lidarMs: Number.isFinite(lidarAgeMs) ? Math.round(lidarAgeMs) : null, hw: !!hwState.blocked, bat: +(+s.bat.r).toFixed(1),
        m: Array.isArray(mods) ? mods.map(m => [m.name, r1(m.angleDeg), r2(m.driveCmdMps), r2(m.driveMps), r1(m.driveAmps), r1(m.steerErrDeg), r1(m.steerAmps), r1(m.driveTempC), r1(m.turnTempC), r1(m.busVolts)]) : null
      });
      if (Array.isArray(s.hits)) bb.keyframe(s.hits.filter((_, i) => i % 2 === 0).map(h => [+(h.x * PX).toFixed(2), +(h.y * PX).toFixed(2)]));
      if (status !== bbPrev.status) { bb.event(status); bbPrev.status = status; }
      // What counts as "something went wrong": a rising edge, so one incident is one trigger.
      if (flags.estop && !bbPrev.estop) bb.trigger('E-stop');
      if (hwState.blocked && !bbPrev.hw) bb.trigger('hardware problem');
      if (flags.stale && !bbPrev.stale) bb.trigger('LiDAR stopped sending');
      if (loc.lost && !bbPrev.lost) bb.trigger('lost position');
      if (trip && trip.id !== bbPrev.trip) { if (bbPrev.trip !== null && BAD_TRIPS.has(trip.outcome)) bb.trigger('trip ended: ' + trip.outcome); bbPrev.trip = trip.id; }
      if (bbPrev.trip === null) bbPrev.trip = trip ? trip.id : undefined;
      Object.assign(bbPrev, { estop: flags.estop, hw: !!hwState.blocked, stale: flags.stale, lost: !!loc.lost });
    } catch (_) {}   // the recorder must never disturb the control loop
  }
  const r1 = v => Number.isFinite(v) ? +v.toFixed(1) : null, r2 = v => Number.isFinite(v) ? +v.toFixed(2) : null;
  function feedSensors(dt) {
    // Odometry: hand over only the change since last tick, in the desk's frame. The brain's position is LiDAR-corrected
    // and the wheels' is not, so passing the wheels' absolute position would undo every correction.
    let p = null;
    try { p = io.readPose(); } catch (_) {}
    if (p && [p.x, p.y, p.theta].every(Number.isFinite)) {
      if (lastPose) {
        const dxF = p.x - lastPose.x, dyF = p.y - lastPose.y, h = lastPose.theta + Math.PI / 2;   // heading of desk-forward
        const c = Math.cos(h), sn = Math.sin(h);
        sim.hwOdometry(c * dxF + sn * dyF, -sn * dxF + c * dyF, wrapPi(p.theta - lastPose.theta));
      }
      lastPose = p;
    }
    // LiDAR: a new scan goes to the brain without the returns that land on the desk itself.
    let l = null;
    try { l = io.readLidar(); } catch (_) {}
    if (l && Array.isArray(l.rays) && Number.isFinite(l.ageMs)) {
      lidarAgeMs = l.ageMs;
      if (l.seq !== lastScanSeq) {
        lastScanSeq = l.seq;
        sim.hwScan(l.rays.filter(r => !(r[4] && Math.abs(r[2]) <= DESK_HALF_FWD && Math.abs(r[3]) <= DESK_HALF_LEFT)));
      }
    } else lidarAgeMs = Infinity;
    if ((batT -= dt) <= 0) { batT = 1; try { sim.hwBattery(io.readBattery()); } catch (_) {} }
  }
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
    else if (c.type === 'blackbox') { if (bb) bb.trigger('saved from the app'); }
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
    // out = what went to the wheels this tick and why, for the black box.
    let out = 'idle', mods = null;
    const flags = { estop: false, cord: false, fresh: true, stale: false };
    try {
      const fresh = bridgeFresh(); flags.fresh = fresh;
      let estop;
      try { estop = !!io.estopEngaged(); } catch (_) { estop = true; }   // unreadable e-stop = engaged
      flags.estop = estop;
      if (estop) { sim.setEstop(true); hwEstopLatched = true; }
      else if (hwEstopLatched) { sim.setEstop(false); hwEstopLatched = false; }   // released: the sim left active=false, so no auto-resume
      // Cord: {plugged, interlock_on}. Plugged with interlock on = no wheel command. Unreadable = locked.
      let cordLocked;
      try {
        const c = io.readCord();
        cordLocked = !c || (!!c.plugged && !!c.interlock_on);
        if (c && hwCordPlugged !== !!c.plugged) { hwCordPlugged = !!c.plugged; sim.setCord(hwCordPlugged); }   // on change only, so bridge 'cord' commands still work
      } catch (_) { cordLocked = true; }
      flags.cord = cordLocked;
      // Per-module readings, if the IO has them (pi_io.js will; null/mock IO may not). null = this IO
      // doesn't report modules (no checks); a throw = it should but can't (the hardware check blocks).
      let modsErr = false;
      if (typeof io.readModules === 'function') { try { mods = io.readModules(); } catch (_) { modsErr = true; } }
      // Hardware check (hardware_check.js), twice a second: a module missing, an encoder not reading, low
      // voltage or a controller fault blocks all motion; the sim ends the trip and refuses new ones.
      if (hwMode) feedSensors(dt);
      hwCheckT -= dt;
      if ((mods || modsErr || hwMode) && hwCheckT <= 0) {
        hwCheckT = 0.5;
        if (mods || modsErr) hwState = hw.step(modsErr ? undefined : mods, 0.5);
        const lidarDead = hwMode && lidarAgeMs > LIDAR_FAULT_MS;
        sim.setHardwareFault(hwState.blocked ? (hwState.text || 'Hardware check failed')
          : lidarDead ? 'LiDAR not sending data: check its cable and power.' : '');
      }
      sim.update(dt);
      const s = sim.getState();
      const lidarStale = hwMode && lidarAgeMs > LIDAR_HOLD_MS;   // never drive on an old picture of the room
      flags.stale = lidarStale;
      if (estop || !fresh || cordLocked || hwState.blocked || lidarStale) {
        out = 'stop:' + (estop ? 'e-stop' : !fresh ? 'bridge silent' : cordLocked ? 'cord' : hwState.blocked ? 'hardware' : 'lidar old');
        io.stopAll(); slew.reset(); return;
      }
      if (mods) {
        const j = jam.step(mods, dt);
        if (j) { out = 'stop:' + j.kind; sim.jamStop(j.text); if (bb) bb.trigger(j.text); io.stopAll(); slew.reset(); return; }
      }
      const moving = Math.hypot(s.T.vx, s.T.vy) > 0.01 || Math.abs(s.T.w) > 0.01;
      if (moving || s.active) {
        const v = toField(s.T.vx, s.T.vy, s.T.w);
        const o = slew.step(v.vx, v.vy, v.omega, dt);
        io.writeWheels(o.vx, o.vy, o.omega, -s.T.th);   // + the heading the brain believes (IO convention: theta = -sim th)
        out = 'drive';
      } else { io.stopAll(); slew.reset(); }
    } catch (e) {
      out = 'stop:error';
      slew.reset();
      try { io.stopAll(); } catch (_) {}   // any failure in the tick leaves the wheels stopped
      if (bb) bb.trigger('control loop error: ' + (e && e.message || e));
    } finally {
      blackBoxStep(dt, out, mods, flags);
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
      if (!timer && autoLoop) timer = setInterval(() => { loop().catch(() => {}); }, Math.round(1000 / hz));
      return true;
    },
    stop() {
      running = false; connected = false;
      if (timer) { clearInterval(timer); timer = null; }
      io.stopAll();
      try { sim.saveMap(); } catch (_) {}   // the sim also saves every 10 s; this catches the last few
      try { if (bb && bb.pending) bb.flush(); } catch (_) {}   // an incident still recording its last seconds: save it now
    },
    get connected() { return connected; },
    get sim() { return sim; },
    get hardware() { return { ...hwState, lidarAgeMs, hwMode }; },
    get blackBox() { return bb; },
    tickOnce, pollNow, pushStateNow, healthCheck
  };
}

module.exports = { createRuntime, createStorage };
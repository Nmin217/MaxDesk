'use strict';

/**
 * The real SPARK bus for the Pi: implements the "SPARK bus" interface pi_io.js uses (same as fake_spark_bus.js)
 * by running spark_can.py, which talks to SPARK MAXes (firmware 24) over SocketCAN (CANable / CANivore as can0).
 *
 *   const bus = createSparkBus({ iface: 'can0' });       // or { sim: true } for pretend SPARKs, no hardware
 *   bus.configure(1, { kind: 'drive', currentLimitA: 40 });
 *   every tick:  bus.heartbeat(true); bus.setVelocity(1, rpm); bus.setPosition(2, rot);
 *   bus.status(2) -> { ageMs, velocityRpm, positionRot, absRot, currentA, tempC, busV, faults }
 *
 * Units: drive = motor rpm, turn = absolute-encoder rotations 0..1. The helper runs the control loops at 100 Hz.
 * Safety lives in the helper: it only sends the CAN enable heartbeat while heartbeat(true) keeps arriving
 * (at most 100 ms apart). If this Node process dies, the helper's stdin closes and it stops everything.
 *
 * Extra for the bench: setDuty(id, -1..1) (open loop, used for the steering direction check), resetFaults(),
 * close(), and alive / error / onStatus for diagnostics.
 */

const { spawn } = require('child_process');
const path = require('path');

const HELPER = path.join(__dirname, 'spark_can.py');
const nowMs = () => (typeof performance !== 'undefined' && performance.now ? performance.now() : Date.now());

function createSparkBus({ iface = 'can0', sim = false, simIds = [1, 2, 3, 4, 5, 6, 7, 8], python = 'python3',
  spawnImpl = spawn, onError = null, log = () => {} } = {}) {
  const args = ['-B', HELPER, ...(sim ? ['--sim', '--sim-ids', simIds.join(',')] : ['--iface', iface])];
  const child = spawnImpl(python, args, { stdio: ['pipe', 'pipe', 'inherit'] });
  const latest = new Map();     // id -> { r: report from the helper, at: our clock when it arrived }
  const kinds = new Map();
  const listeners = [];
  let alive = true, error = null, ready = false, buf = '';

  const fail = msg => { error = msg; log(msg); if (onError) onError(msg); };
  const send = obj => {
    if (!alive) return;
    try { child.stdin.write(JSON.stringify(obj) + '\n'); } catch (e) { fail(`SPARK helper write failed: ${e.message}`); }
  };

  child.stdout.setEncoding('utf8');
  child.stdout.on('data', chunk => {
    buf += chunk;
    let i;
    while ((i = buf.indexOf('\n')) >= 0) {
      const line = buf.slice(0, i); buf = buf.slice(i + 1);
      if (!line.trim()) continue;
      let m; try { m = JSON.parse(line); } catch (_) { continue; }
      if (m.ready) { ready = true; continue; }
      if (m.error) { fail(`SPARK helper: ${m.error}`); continue; }
      if (m.dev) {
        const at = nowMs();
        for (const [id, r] of Object.entries(m.dev)) latest.set(+id, { r, at });
        for (const f of listeners) f(m);
      }
    }
  });
  child.on('exit', code => { alive = false; if (code) fail(`SPARK helper stopped (exit ${code}): the motors are disabled`); });
  child.on('error', e => { alive = false; fail(`cannot start the SPARK helper (${python}): ${e.message}`); });
  if (child.stdin.on) child.stdin.on('error', () => { alive = false; });

  return {
    configure(id, cfg) { kinds.set(id, cfg.kind); send({ op: 'config', id, ...cfg }); return alive; },
    setVelocity(id, rpm) { send({ op: 'vel', id, v: Number.isFinite(+rpm) ? +rpm : 0 }); },
    setPosition(id, rot) { send({ op: 'pos', id, v: Number.isFinite(+rot) ? +rot : 0 }); },
    setDuty(id, duty) { send({ op: 'duty', id, v: Number.isFinite(+duty) ? Math.max(-1, Math.min(1, +duty)) : 0 }); },
    heartbeat(enabled) { send({ op: 'hb', on: !!enabled }); },
    resetFaults() { send({ op: 'reset' }); },

    status(id) {
      const e = latest.get(id);
      if (!e || e.r.age === null || e.r.age === undefined) return null;   // never heard from it
      const r = e.r, num = v => (v === null || v === undefined ? NaN : +v);
      return {
        ageMs: r.age + (nowMs() - e.at) + (alive ? 0 : 1e6),   // a dead helper makes everything stale
        velocityRpm: num(r.vel), positionRot: num(r.pos),
        absRot: kinds.get(id) === 'turn' ? num(r.abs) : undefined,
        currentA: num(r.amps), tempC: num(r.temp), busV: num(r.volts), duty: num(r.duty),
        faults: Array.isArray(r.faults) ? r.faults : []
      };
    },

    onStatus(f) { listeners.push(f); },
    close() {
      if (!alive) return;
      send({ op: 'close' });
      try { child.stdin.end(); } catch (_) { /* already gone */ }
    },
    get alive() { return alive; },
    get ready() { return ready; },
    get error() { return error; }
  };
}

module.exports = { createSparkBus, HELPER };

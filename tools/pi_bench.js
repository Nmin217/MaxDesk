#!/usr/bin/env node
'use strict';

/**
 * Bench test for the swerve frame, run on the Raspberry Pi: no roboRIO, no Driver Station, no laptop.
 * The Pi talks to the SPARK MAXes over CAN (firmware 24, through a CANable or CANivore as can0) and reads the
 * Xbox controller plugged into its USB port. It is the Pi version of firmware/module-test.
 *
 *   node tools/pi_bench.js                  real SPARKs on can0, controller on /dev/input/js0
 *   node tools/pi_bench.js --sim            pretend SPARKs (try it with no hardware; a controller still helps)
 *   options: --iface can0  --pad /dev/input/js0
 *
 * FRAME ON BLOCKS, WHEELS OFF THE GROUND for the first runs.
 *
 * Controls (the same as the roboRIO bench program):
 *   Start / Back      next / previous mode (only while RB is released)
 *   RB (hold)         enable. Let go and everything stops within 0.1 s. Unplugging the controller = let go.
 *   Readout           never moves. Prints every wheel's angle once a second: "robot" (0 = forward, 90 = left)
 *                     and "raw" (the encoder; 0 = where the jig zeroed it).
 *   Self test         hold RB: steering direction check, steer to 0/90/180/270/0, drive forward and back slowly.
 *                     PASS/FAIL per module.
 *   Joystick          hold RB: left stick points and drives every wheel the same way (15 % max).
 *                     A/B/X/Y snap all wheels to 0/90/180/270 (A = all straight forward: the alignment check).
 *   Cart              hold RB: real swerve. Left stick drives, right stick turns.
 * Ctrl+C quits (the SPARKs stop by themselves within 0.1 s).
 */

const path = require('path');
const S = require(path.join(__dirname, '..', 'robot', 'pi', 'swerve.js'));

// The four-module square frame, the same as firmware/module-test Constants.kModules. Keep the two in step.
// zeroPointsDeg: which way the wheel points (robot terms, 0 = forward, 90 = left) when its encoder reads 0, i.e.
// right after the jig zero. x/y: module centre, metres forward / left of the frame centre (measured: ~1 ft from the centre).
const HALF = 0.216;   // modules ~1 ft (0.305 m) from the centre on the diagonals: 0.305 / sqrt 2 each way
const BENCH_MODULES = [
  { name: 'FL', driveId: 1, turnId: 2, x: HALF, y: HALF, zeroPointsDeg: 90 },
  { name: 'FR', driveId: 3, turnId: 4, x: HALF, y: -HALF, zeroPointsDeg: 0 },
  { name: 'BL', driveId: 5, turnId: 6, x: -HALF, y: HALF, zeroPointsDeg: 180 },
  { name: 'BR', driveId: 7, turnId: 8, x: -HALF, y: -HALF, zeroPointsDeg: -90 }   // the V2 encoder (reads the same)
];

const LIMITS = {
  maxDrivePercent: 0.15,      // of free speed, every mode
  scriptDrivePercent: 0.10,
  deadband: 0.08,
  cartTurnScale: 0.5,
  steerToleranceDeg: 5,
  steerTimeoutS: 2,
  benchDriveCurrentA: 20,
  turnCurrentA: 20
};
const MODES = ['Readout', 'Self test', 'Joystick', 'Cart'];
const STALE_MS = 250;
const TAU = 2 * Math.PI;

function createBench({ bus, pad, modules = BENCH_MODULES, limits = LIMITS, now, print = console.log }) {
  const L = { ...LIMITS, ...limits };
  const mods = modules.map(m => ({ ...m, zero: (m.zeroPointsDeg || 0) * Math.PI / 180 }));
  const kin = S.createKinematics(mods.map(m => ({ x: m.x, y: m.y })));
  const maxSpeed = L.maxDrivePercent * S.WHEEL_FREE_SPEED;
  const maxOmega = maxSpeed / Math.max(0.05, ...mods.map(m => Math.hypot(m.x, m.y)));
  const maxDuty = Math.min(1, L.maxDrivePercent * 1.2);   // helper-side cap, a little above the speed cap for the loop
  for (const m of mods) {
    bus.configure(m.driveId, { kind: 'drive', currentLimitA: L.benchDriveCurrentA, idleMode: 'brake', maxDuty });
    bus.configure(m.turnId, { kind: 'turn', currentLimitA: L.turnCurrentA, idleMode: 'brake' });
  }

  let mode = 0, enabled = false, holdAngle = 0, lastPrint = -1e9, lastWarn = '';
  let tests = null;

  const say = msg => print(msg);
  const raw = m => { const s = bus.status(m.turnId); return s && s.ageMs < STALE_MS && Number.isFinite(s.absRot) ? s.absRot * TAU : NaN; };
  const angle = m => { const r = raw(m); return Number.isFinite(r) ? S.wrapAngle(r + m.zero) : NaN; };
  const drivePos = m => { const s = bus.status(m.driveId); return s && s.ageMs < STALE_MS ? s.positionRot : NaN; };
  const encRot = (m, robotAngle) => ((S.wrapAngle(robotAngle - m.zero) / TAU) % 1 + 1) % 1;
  const deg = r => Number.isFinite(r) ? `${Math.round(((r * 180 / Math.PI) % 360 + 360) % 360)}`.padStart(3) : ' --';

  function steer(m, robotAngle) { bus.setPosition(m.turnId, encRot(m, robotAngle)); }

  // Point one module and drive it, flipping instead of turning > 90 deg and slowing while it is still turning.
  function setState(m, speedMps, robotAngle) {
    const cur = angle(m);
    if (!Number.isFinite(cur)) { bus.setVelocity(m.driveId, 0); return; }   // no encoder reading: don't guess
    const o = S.optimize({ speed: speedMps, angle: robotAngle }, cur);
    steer(m, o.angle);
    bus.setVelocity(m.driveId, S.cosineScale(o.speed, o.angle - cur) / S.DRIVE_VELOCITY_FACTOR);
  }

  function missing() {
    const out = [];
    for (const m of mods) {
      const d = bus.status(m.driveId), t = bus.status(m.turnId);
      if (!d || d.ageMs > STALE_MS) out.push(`${m.name} drive (CAN ${m.driveId})`);
      if (!t || t.ageMs > STALE_MS) out.push(`${m.name} turn (CAN ${m.turnId})`);
      else if (!Number.isFinite(t.absRot)) out.push(`${m.name} encoder`);
    }
    return out;
  }

  // ---- Self test, one state machine per module (mirrors firmware/module-test) ----
  const TARGETS = [0, 90, 180, 270, 0];
  const SCRIPT = [[L.scriptDrivePercent, 2], [0, 1], [-L.scriptDrivePercent, 2], [0, 1]];
  function startTests(t) {
    tests = mods.map(m => {
      const a = angle(m);
      if (!Number.isFinite(a)) { say(`[${m.name}] FAIL: no steering encoder reading (unplugged, or SPARK ${m.turnId} not answering)`); return { m, step: 'failed' }; }
      return { m, step: 'direction', t0: t, start: raw(m), index: 0, inTol: -1, startPos: drivePos(m) };
    });
  }
  function runTest(x, t) {
    const { m } = x, dt = t - x.t0;
    const fail = why => { x.step = 'failed'; bus.setVelocity(m.driveId, 0); say(`[${m.name}] FAIL: ${why}`); };
    const next = step => { x.step = step; x.t0 = t; };
    switch (x.step) {
      case 'direction': {
        bus.setDuty(m.turnId, 0.1);
        if (dt > 0.6) {
          const moved = S.wrapAngle(raw(m) - x.start) * 180 / Math.PI;
          say(`[${m.name}] direction check: +0.1 output moved ${moved.toFixed(1)} deg`);
          if (moved > 3) next('pause');
          else if (moved < -3) fail('encoder counts backwards: the absolute encoder must be inverted');
          else fail('angle did not change: encoder unplugged, turn motor not moving, or drive/turn CAN IDs swapped');
        }
        break;
      }
      case 'pause':
        bus.setDuty(m.turnId, 0);
        if (dt > 0.5) { next('steer'); x.index = 0; x.inTol = -1; }
        break;
      case 'steer': {
        const target = TARGETS[x.index] * Math.PI / 180;
        steer(m, target); bus.setVelocity(m.driveId, 0);
        const err = Math.abs(S.wrapAngle(target - angle(m))) * 180 / Math.PI;
        if (err < L.steerToleranceDeg) {
          if (x.inTol < 0) x.inTol = t;
          if (t - x.inTol > 0.3) {
            say(`[${m.name}] steer ${String(TARGETS[x.index]).padStart(3)} deg OK (${dt.toFixed(1)} s, error ${err.toFixed(1)} deg)`);
            x.index++; x.inTol = -1; x.t0 = t;
            if (x.index === TARGETS.length) { next('drive'); x.index = 0; x.startPos = drivePos(m); }
          }
        } else {
          x.inTol = -1;
          if (dt > L.steerTimeoutS) fail(`steer to ${TARGETS[x.index]} deg timed out, still ${Number.isFinite(err) ? err.toFixed(1) : '?'} deg off`);
        }
        break;
      }
      case 'drive': {
        const [pct, secs] = SCRIPT[x.index];
        steer(m, 0);
        bus.setVelocity(m.driveId, pct * S.WHEEL_FREE_SPEED / S.DRIVE_VELOCITY_FACTOR);
        if (dt > secs) {
          const moved = drivePos(m) - x.startPos;
          if (pct !== 0) {
            say(`[${m.name}] drive ${pct > 0 ? '+' : ''}${Math.round(pct * 100)}%: motor turned ${Number.isFinite(moved) ? moved.toFixed(1) : '?'} rotations`);
            if (!(Math.abs(moved) > 2) || Math.sign(moved) !== Math.sign(pct)) { fail('drive encoder did not move the right way: motor not spinning, wrong CAN ID, or wired backwards'); return; }
          }
          x.startPos = drivePos(m); x.index++; x.t0 = t;
          if (x.index === SCRIPT.length) { x.step = 'pass'; bus.setVelocity(m.driveId, 0); say(`[${m.name}] PASS`); }
        }
        break;
      }
      default:   // pass / failed: hold still
        bus.setVelocity(m.driveId, 0); bus.setDuty(m.turnId, 0);
    }
  }

  function printReadout(t) {
    if (t - lastPrint < 1) return;
    lastPrint = t;
    say('Angles:' + mods.map(m => `  ${m.name} robot ${deg(angle(m))} (raw ${deg(raw(m))})`).join(''));
  }

  return {
    step() {
      const t = now();
      // Mode changes only while not enabled, like the roboRIO version.
      const rb = pad.connected && pad.button('rb');
      const nextP = pad.pressed('start'), prevP = pad.pressed('back');
      if ((nextP || prevP) && !rb) {
        mode = (mode + (nextP ? 1 : MODES.length - 1)) % MODES.length;
        say(`Mode ${mode + 1}/${MODES.length}: ${MODES[mode]}. Hold RB to run it. (Start = next, Back = previous)`);
      } else if ((nextP || prevP) && rb) say('Let go of RB first to change the mode.');

      const want = rb && MODES[mode] !== 'Readout' && bus.alive !== false;
      if (want && !enabled) {
        const miss = missing();
        if (miss.length && MODES[mode] !== 'Self test') {
          const w = `Not enabling: no reading from ${miss.join(', ')}. Check CAN wiring and IDs (Readout shows what answers).`;
          if (w !== lastWarn) { say(w); lastWarn = w; }
          return { enabled: false, mode: MODES[mode] };
        }
        enabled = true; lastWarn = '';
        if (bus.resetFaults) bus.resetFaults();
        say(`=== Enabled: ${MODES[mode]} ===`);
        holdAngle = 0;
        if (MODES[mode] === 'Self test') startTests(t);
      } else if (!want && enabled) {
        enabled = false;
        for (const m of mods) { bus.setVelocity(m.driveId, 0); }
        bus.heartbeat(false);
        say('=== Disabled ===');
      }

      if (MODES[mode] === 'Readout') printReadout(t);
      if (!enabled) return { enabled, mode: MODES[mode] };

      bus.heartbeat(true);
      if (MODES[mode] === 'Self test') {
        for (const x of tests) runTest(x, t);
      } else if (MODES[mode] === 'Joystick') {
        const x = -pad.axis('ly'), y = -pad.axis('lx');
        const mag = Math.min(1, Math.hypot(x, y));
        const snap = pad.button('a') ? 0 : pad.button('b') ? 90 : pad.button('x') ? 180 : pad.button('y') ? 270 : null;
        if (snap !== null) holdAngle = snap * Math.PI / 180;
        let speed = 0;
        if (mag > L.deadband) { holdAngle = Math.atan2(y, x); speed = (mag - L.deadband) / (1 - L.deadband) * maxSpeed; }
        for (const m of mods) setState(m, speed, holdAngle);
      } else if (MODES[mode] === 'Cart') {
        let x = -pad.axis('ly'), y = -pad.axis('lx');
        const mag = Math.min(1, Math.hypot(x, y));
        const k = mag > L.deadband ? (mag - L.deadband) / (1 - L.deadband) / Math.hypot(x, y) : 0;
        x *= k; y *= k;
        const rx = -pad.axis('rx'), w = Math.abs(rx) > L.deadband ? Math.sign(rx) * (Math.abs(rx) - L.deadband) / (1 - L.deadband) : 0;
        const cur = mods.map(angle);
        const states = kin.toModuleStates(x * maxSpeed, y * maxSpeed, w * L.cartTurnScale * maxOmega,
          { maxSpeed, currentAngles: cur.map(a => (Number.isFinite(a) ? a : 0)) });
        mods.forEach((m, i) => setState(m, states[i].speed, states[i].angle));
      }
      return { enabled, mode: MODES[mode] };
    },
    get mode() { return MODES[mode]; },
    get enabled() { return enabled; },
    results() { return tests ? tests.map(x => ({ name: x.m.name, step: x.step })) : []; },
    angles() { return mods.map(angle); }
  };
}

function main() {
  const argv = process.argv.slice(2);
  const opt = (name, dflt) => { const i = argv.indexOf(name); return i >= 0 && argv[i + 1] ? argv[i + 1] : dflt; };
  const sim = argv.includes('--sim');
  const { createSparkBus } = require(path.join(__dirname, '..', 'robot', 'pi', 'spark_bus.js'));
  const { createXbox } = require(path.join(__dirname, '..', 'robot', 'pi', 'xbox.js'));
  const t0 = performance.now();
  const now = () => (performance.now() - t0) / 1000;
  const bus = createSparkBus({ iface: opt('--iface', 'can0'), sim, log: m => console.error(m) });
  const pad = createXbox({ device: opt('--pad', '/dev/input/js0'), log: m => console.log(m) });
  console.log(`MaxDesk Pi bench ${sim ? '(PRETEND SPARKs)' : `(CAN: ${opt('--iface', 'can0')})`}. FRAME ON BLOCKS.`);
  console.log('Start/Back = mode (RB released), hold RB = run. Mode 1/4: Readout. Ctrl+C quits.');
  const bench = createBench({ bus, pad, now });
  let lastAlive = true;
  const timer = setInterval(() => {
    bench.step();
    if (lastAlive && !bus.alive) { lastAlive = false; console.error('The SPARK helper stopped: motors disabled. ' + (bus.error || '')); }
  }, 20);
  const quit = () => { clearInterval(timer); bus.close(); pad.close(); setTimeout(() => process.exit(0), 200); };
  process.on('SIGINT', quit);
  process.on('SIGTERM', quit);
}

if (require.main === module) main();

module.exports = { createBench, BENCH_MODULES, LIMITS, MODES };

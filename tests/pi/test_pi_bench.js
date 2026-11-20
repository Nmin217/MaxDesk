'use strict';
// tools/pi_bench.js (the Pi's own bench program) on the fake SPARK bus, and robot/pi/xbox.js: node tests/pi/test_pi_bench.js
const assert = require('node:assert/strict');
const { EventEmitter } = require('node:events');
const { createBench, BENCH_MODULES, LIMITS } = require('../../tools/pi_bench');
const { createFakeSparkBus } = require('../../robot/pi/fake_spark_bus');
const { createXbox, parseEvents } = require('../../robot/pi/xbox');
const S = require('../../robot/pi/swerve');

let passed = 0, failed = 0;
function test(name, fn) { try { fn(); passed++; console.log('PASS ' + name); } catch (e) { failed++; console.log('FAIL ' + name + ': ' + e.message); } }
const near = (a, b, tol, msg = '') => assert.ok(Math.abs(a - b) <= tol, `${msg} expected ${b} +- ${tol}, got ${a}`);
const wrapDeg = d => ((d + 180) % 360 + 360) % 360 - 180;
const DT = 0.02;

function fakePad() {
  const p = { connected: true, axes: {}, buttons: {}, edges: new Set(),
    axis(n) { return p.connected ? p.axes[n] || 0 : 0; }, button(n) { return p.connected && !!p.buttons[n]; },
    pressed(n) { const w = p.edges.has(n); p.edges.delete(n); return w; },
    press(n) { p.edges.add(n); } };
  return p;
}

function rig({ faults = [], angles = {} } = {}) {
  const bus = createFakeSparkBus({ ids: [1, 2, 3, 4, 5, 6, 7, 8] });
  for (const [id, name] of faults) bus.setFault(id, name);
  for (const [id, a] of Object.entries(angles)) bus.setAngle(+id, a);
  const pad = fakePad(), lines = [];
  let hb = [];
  const realHb = bus.heartbeat; bus.heartbeat = on => { hb.push(on); realHb(on); };
  const bench = createBench({ bus, pad, now: () => bus.time, print: m => lines.push(m) });
  const run = (sec, each) => { for (let i = 0; i < Math.round(sec / DT); i++) { if (each) each(); bench.step(); bus.step(DT); } };
  run(0.1);   // let status arrive
  const toMode = name => { while (bench.mode !== name) { pad.press('start'); run(DT); } };
  return { bus, pad, bench, lines, run, toMode, hb: () => hb, clearHb: () => { hb = []; } };
}
const robotDeg = (r, i) => r.bench.angles()[i] * 180 / Math.PI;

test('configures all 8 SPARKs with bench current limits and a drive power cap', () => {
  const r = rig();
  const cfg = r.bus.log.filter(e => e.op === 'configure');
  assert.equal(cfg.length, 8);
  for (const e of cfg) {
    assert.equal(e.cfg.currentLimitA, 20);
    if (e.cfg.kind === 'drive') assert.ok(e.cfg.maxDuty <= 0.2, `maxDuty ${e.cfg.maxDuty}`);
  }
  assert.deepEqual(cfg.filter(e => e.cfg.kind === 'turn').map(e => e.id).sort(), [2, 4, 6, 8]);
});

test('Readout never enables, even with RB held, and prints robot + raw angles', () => {
  const r = rig();
  r.pad.buttons.rb = true;
  r.run(1.5);
  assert.equal(r.bench.enabled, false);
  assert.ok(!r.hb().includes(true));
  assert.ok(r.lines.some(l => /Angles: +FL robot/.test(l)), r.lines.join('\n'));
});

test('Start/Back change the mode only while RB is released', () => {
  const r = rig();
  r.pad.press('start'); r.run(DT); assert.equal(r.bench.mode, 'Self test');
  r.pad.press('back'); r.run(DT); assert.equal(r.bench.mode, 'Readout');
  r.pad.press('back'); r.run(DT); assert.equal(r.bench.mode, 'Cart');
  r.pad.buttons.rb = true; r.pad.press('start'); r.run(DT);
  assert.equal(r.bench.mode, 'Cart');
  assert.ok(r.lines.some(l => /Let go of RB/.test(l)));
});

test('Joystick + A: every wheel lines up front-to-back, using each module\'s jig zero direction', () => {
  const r = rig({ angles: { 2: 1.0, 4: -2.0, 6: 0.5, 8: 3.0 } });
  r.toMode('Joystick');
  r.pad.buttons.rb = true; r.pad.buttons.a = true;
  r.run(1.5);
  // Forward or backward (0 or 180): with no speed asked, the 180 flip is allowed; the drive test checks the roll.
  const line = d => Math.min(Math.abs(wrapDeg(d)), Math.abs(wrapDeg(d + 180)));
  BENCH_MODULES.forEach((m, i) => {
    near(line(robotDeg(r, i)), 0, 3, `${m.name} robot angle`);
    // The encoder itself sits at minus the zero direction (FL -90, FR 0, BL 180, BR +90), or 180 from it.
    near(line(r.bus.truth(m.turnId).angle * 180 / Math.PI + m.zeroPointsDeg), 0, 3, `${m.name} encoder`);
  });
});

test('Joystick stick forward: all wheels drive forward at the same capped speed', () => {
  const r = rig();
  r.toMode('Joystick');
  r.pad.buttons.rb = true; r.pad.axes.ly = -1;   // stick fully up
  r.run(1.5);
  const capRpm = LIMITS.maxDrivePercent * S.WHEEL_FREE_SPEED / S.DRIVE_VELOCITY_FACTOR;
  BENCH_MODULES.forEach((m, i) => {
    const t = r.bus.truth(m.driveId);
    // Wheel may run backwards if the module flipped 180; then its robot angle is 180.
    const forward = Math.abs(wrapDeg(robotDeg(r, i))) < 5;
    near(forward ? t.rpm : -t.rpm, capRpm, capRpm * 0.1, `${m.name} rpm`);
  });
});

test('letting go of RB stops everything: heartbeat off, wheels spin down', () => {
  const r = rig();
  r.toMode('Joystick');
  r.pad.buttons.rb = true; r.pad.axes.ly = -1;
  r.run(1.0);
  r.pad.buttons.rb = false;
  r.run(0.5);
  assert.equal(r.bench.enabled, false);
  assert.equal(r.hb().at(-1), false);
  for (const m of BENCH_MODULES) assert.ok(Math.abs(r.bus.truth(m.driveId).rpm) < 20, `${m.name} still spinning`);
});

test('unplugging the controller is the same as letting go', () => {
  const r = rig();
  r.toMode('Cart');
  r.pad.buttons.rb = true; r.pad.axes.ly = -1;
  r.run(0.5);
  assert.equal(r.bench.enabled, true);
  r.pad.connected = false;
  r.run(0.3);
  assert.equal(r.bench.enabled, false);
  assert.equal(r.bus.truth(1).enabled, false);
});

test('refuses to enable Joystick/Cart with a SPARK or encoder missing, and says which', () => {
  const r = rig({ faults: [[6, 'encoderUnplugged']] });
  r.toMode('Joystick');
  r.pad.buttons.rb = true;
  r.run(0.5);
  assert.equal(r.bench.enabled, false);
  assert.ok(r.lines.some(l => /Not enabling: .*BL encoder/.test(l)), r.lines.join('\n'));
});

test('Self test passes on four healthy modules', () => {
  const r = rig({ angles: { 2: 2.0, 4: -1.0, 6: 0.3, 8: 2.8 } });
  r.toMode('Self test');
  r.pad.buttons.rb = true;
  r.run(16);
  assert.deepEqual(r.bench.results().map(x => x.step), ['pass', 'pass', 'pass', 'pass'], r.lines.join('\n'));
});

test('Self test: a backwards steering encoder fails that module only, before any closed-loop steering', () => {
  const r = rig({ faults: [[4, 'backwards']] });
  r.toMode('Self test');
  r.pad.buttons.rb = true;
  r.run(16);
  const res = Object.fromEntries(r.bench.results().map(x => [x.name, x.step]));
  assert.equal(res.FR, 'failed');
  assert.ok(r.lines.some(l => /\[FR\] FAIL: encoder counts backwards/.test(l)));
  assert.equal(res.FL, 'pass'); assert.equal(res.BL, 'pass'); assert.equal(res.BR, 'pass');
  assert.equal(r.bus.log.filter(e => e.id === 4 && e.op === 'position').length, 0, 'FR got closed-loop steering');
});

test('Cart: turning on the spot points every wheel along the circle', () => {
  const r = rig();
  r.toMode('Cart');
  r.pad.buttons.rb = true; r.pad.axes.rx = -1;   // right stick left = turn left (CCW)
  r.run(1.5);
  BENCH_MODULES.forEach((m, i) => {
    const want = Math.atan2(m.x, -m.y) * 180 / Math.PI;   // direction of omega x r
    const got = robotDeg(r, i);
    near(Math.min(Math.abs(wrapDeg(got - want)), Math.abs(wrapDeg(got - want + 180))), 0, 4, m.name);
  });
});

// ---- xbox.js ----
function ev(value, type, number, init = false) { const b = Buffer.alloc(8); b.writeUInt32LE(123, 0); b.writeInt16LE(value, 4); b[6] = type | (init ? 0x80 : 0); b[7] = number; return b; }

test('xbox: parses Linux joystick events, keeps a partial event for the next chunk', () => {
  const buf = Buffer.concat([ev(32767, 2, 1), ev(1, 1, 5, true), Buffer.from([1, 2, 3])]);
  const r = parseEvents(buf);
  assert.equal(r.events.length, 2);
  assert.deepEqual(r.events[1], { time: 123, value: 1, type: 1, number: 5 });
  assert.equal(r.rest.length, 3);
});

test('xbox: buttons, axes, single press edges, and everything released when unplugged', () => {
  const stream = new EventEmitter();
  const fsImpl = { createReadStream: () => stream };
  const pad = createXbox({ fsImpl, retryMs: 1e9 });
  stream.emit('open');
  assert.equal(pad.connected, true);
  stream.emit('data', Buffer.concat([ev(1, 1, 5), ev(-32767, 2, 1), ev(1, 1, 7)]));
  stream.emit('data', ev(0, 1, 7).subarray(0, 5));
  stream.emit('data', ev(0, 1, 7).subarray(5));
  assert.equal(pad.button('rb'), true);
  near(pad.axis('ly'), -1, 1e-6);
  assert.equal(pad.button('start'), false);
  assert.equal(pad.pressed('start'), true);
  assert.equal(pad.pressed('start'), false);
  stream.emit('error', Object.assign(new Error('gone'), { code: 'ENODEV' }));
  assert.equal(pad.connected, false);
  assert.equal(pad.button('rb'), false);
  assert.equal(pad.axis('ly'), 0);
  pad.close();
});

console.log(`\n${passed} passed, ${failed} failed`);
process.exitCode = failed ? 1 : 0;

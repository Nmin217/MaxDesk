'use strict';
// pi_io.js against the fake SPARK bus: node tests/pi/test_pi_io.js
// Each test is one way the real wiring or code could be wrong: IDs, units, directions, the 180-degree flip,
// the heartbeat, odometry, missing controllers, unplugged encoders, jams, low battery, unwired safety inputs.
const assert = require('node:assert/strict');
const S = require('../../robot/pi/swerve');
const { createPiIO, DEFAULT_MODULES } = require('../../robot/pi/pi_io');
const { createFakeSparkBus } = require('../../robot/pi/fake_spark_bus');
const { createJamDetector } = require('../../robot/pi/motion_guard');
const { checkModules } = require('../../robot/pi/hardware_check');

let passed = 0, failed = 0;
function test(name, fn) { try { fn(); passed++; console.log('PASS ' + name); } catch (e) { failed++; console.log('FAIL ' + name + ': ' + e.message); } }
const dt = 1 / 60, near = (a, b, tol, msg = '') => assert.ok(Math.abs(a - b) <= tol, `${msg} expected ${b} +- ${tol}, got ${a}`);
const inputs = { estopEngaged: () => false, readCord: () => ({ plugged: false, interlock_on: true }) };
function rig(opts = {}) {
  const bus = createFakeSparkBus();
  const io = createPiIO({ bus, inputs, ...opts });
  const run = (sec, vx, vy, w, theta) => { for (let i = 0; i < Math.round(sec / dt); i++) { if (vx === null) io.stopAll(); else io.writeWheels(vx, vy, w, theta); bus.step(dt); } };
  return { bus, io, run };
}
// IO field frame (io.js): x right, y up. Desk at theta_io = 0 faces +y (its forward is the monitor back).

test('every SPARK is configured once with its role and current limit (drive/turn: FL 1/2, FR 3/4, RL 5/6, RR 7/8)', () => {
  const { bus } = rig();
  const cfg = bus.log.filter(e => e.op === 'configure').map(e => [e.id, e.cfg.kind, e.cfg.currentLimitA]);
  assert.deepEqual(cfg.sort((a, b) => a[0] - b[0]), [[1, 'drive', 40], [2, 'turn', 20], [3, 'drive', 40], [4, 'turn', 20], [5, 'drive', 40], [6, 'turn', 20], [7, 'drive', 40], [8, 'turn', 20]]);
});

test('drive forward 0.5 m/s: all wheels point forward and spin at the right motor rpm', () => {
  const { bus, io, run } = rig();
  run(1, 0, 0.5, 0, 0);   // field +y = desk forward at theta 0
  const rpm = 0.5 / S.DRIVE_VELOCITY_FACTOR;
  near(rpm, 0.5 / (Math.PI * 0.0762) * S.DRIVE_REDUCTION * 60, 1e-6, 'conversion');
  for (const m of DEFAULT_MODULES) {
    near(bus.truth(m.turnId).angle, 0, 1e-3, m.name + ' angle');
    near(bus.truth(m.driveId).rpm, rpm, 2, m.name + ' rpm');
  }
});

test('field-relative: same field command with the desk turned 90 degrees CCW points the wheels 90 degrees clockwise', () => {
  const { bus, run } = rig();
  run(1, 0, 0.5, 0, Math.PI / 2);   // desk faces field -x; field +y is now to the desk's right
  for (const m of DEFAULT_MODULES) {
    const a = bus.truth(m.turnId).angle, rpm = bus.truth(m.driveId).rpm;
    // right (-90 deg) driving forward, or the same line flipped (+90 deg) driving backwards: both correct
    assert.ok((Math.abs(S.wrapAngle(a + Math.PI / 2)) < 1e-3 && rpm > 0) || (Math.abs(S.wrapAngle(a - Math.PI / 2)) < 1e-3 && rpm < 0), `${m.name} ${a} ${rpm}`);
  }
});

test('turning in place CCW: each wheel at right angles to its corner, the left side driving backwards', () => {
  const { bus, io, run } = rig();
  run(1.5, 0, 0, 1.0, 0);
  const st = io.readModules();
  for (let i = 0; i < 4; i++) {
    const m = DEFAULT_MODULES[i], want = Math.atan2(m.x, -m.y);   // tangent direction for CCW: (-y, x)
    const a = bus.truth(m.turnId).angle, v = st[i].driveMps;
    const ok = (Math.abs(S.wrapAngle(a - want)) < 0.01 && v > 0) || (Math.abs(S.wrapAngle(a - want - Math.PI)) < 0.01 && v < 0);
    assert.ok(ok, `${m.name}: angle ${a.toFixed(3)} want ${want.toFixed(3)} speed ${v.toFixed(3)}`);
    near(Math.abs(v), Math.hypot(m.x, m.y) * 1.0, 0.01, m.name + ' speed');
  }
});

test('the 180-degree flip: asked to point 170 degrees away, a wheel turns 10 degrees and runs backwards', () => {
  const { bus, run } = rig();
  run(1, 0, 0.5, 0, 0);
  const target = 170 * Math.PI / 180;   // a field direction 170 degrees CCW from desk-forward
  run(0.05, -0.5 * Math.sin(target), 0.5 * Math.cos(target), 0, 0);
  const pos = bus.log.filter(e => e.op === 'position' && e.id === 2).pop().value;
  near(S.wrapAngle(pos * 2 * Math.PI), S.wrapAngle(target - Math.PI), 1e-6, 'steered to -10 degrees');
  assert.ok(bus.log.filter(e => e.op === 'velocity' && e.id === 1).pop().value <= 0, 'driving backwards');
});

test('a wheel still turning gets less speed (cosine scaling), none while it is far off', () => {
  const { bus, io } = rig();
  io.writeWheels(0.5, 0, 0, 0);   // field +x = desk right: wheels must swing 90 degrees first
  const v = bus.log.filter(e => e.op === 'velocity' && e.id === 1).pop().value;
  near(v, 0, 1e-6, 'no drive while 90 degrees off');
});

test('heartbeat: stopAll disables at once; a hung program stops the SPARKs within 0.1 s by themselves', () => {
  const { bus, io, run } = rig();
  run(1, 0, 0.5, 0, 0);
  io.stopAll(); bus.step(dt);
  assert.equal(bus.truth(1).enabled, false, 'stopAll disables');
  run(0.5, 0, 0.5, 0, 0);
  for (let i = 0; i < 7; i++) bus.step(dt);   // no more writeWheels: the program "hung"
  assert.equal(bus.truth(1).enabled, false, 'disabled after 0.1 s without heartbeat');
  for (let i = 0; i < 30; i++) bus.step(dt);
  near(bus.truth(1).rpm, 0, 5, 'wheel braked to a stop');
});

test('odometry: 2 m forward, then a quarter turn CCW (no gyro), matches what the wheels did', () => {
  const r2 = rig(); r2.io.readPose();
  for (let i = 0; i < 60; i++) { r2.io.writeWheels(0, 0.4, 0, 0); r2.bus.step(dt); r2.io.readPose(); }   // wheels align, rolls a bit
  const p0 = r2.io.readPose();
  for (let i = 0; i < 300; i++) { r2.io.writeWheels(0, 0.4, 0, 0); r2.bus.step(dt); r2.io.readPose(); }
  const p1 = r2.io.readPose();
  near(p1.x - p0.x, 0, 0.005, 'no sideways'); near(p1.y - p0.y, 2.0, 0.03, 'forward 2 m');
  for (let i = 0; i < 40; i++) { r2.io.stopAll(); r2.bus.step(dt); r2.io.readPose(); }
  const a0 = r2.io.readPose().theta;
  for (let i = 0; i < 240; i++) { r2.io.writeWheels(0, 0, Math.PI / 8, r2.io.readPose().theta); r2.bus.step(dt); r2.io.readPose(); }
  for (let i = 0; i < 40; i++) { r2.io.stopAll(); r2.bus.step(dt); r2.io.readPose(); }
  near(S.wrapAngle(r2.io.readPose().theta - a0), Math.PI / 2, 0.06, 'quarter turn');
});

test('with a gyro, heading comes from the gyro', () => {
  let yaw = 0;
  const r = rig({ imu: { yaw: () => yaw } });
  r.io.readPose(); yaw = 0.7; r.bus.step(dt);
  near(S.wrapAngle(r.io.readPose().theta), 0.7, 1e-9);
});

test('a SPARK unplugged: that module reads as missing and the hardware check names it', () => {
  const { bus, io, run } = rig();
  run(0.2, null);
  bus.setFault(4, 'unplugged'); for (let i = 0; i < 20; i++) bus.step(dt);
  const mods = io.readModules();
  assert.equal(mods.find(m => m.name === 'FR').present, false);
  assert.match(checkModules(mods)[0].text, /FR not answering on CAN/);
});

test('an encoder unplugged: encoderOk false, the check says so, odometry stops moving', () => {
  const { bus, io, run } = rig();
  run(0.5, 0, 0.4, 0, 0); io.readPose();
  bus.setFault(6, 'encoderUnplugged');
  const p0 = io.readPose(); run(0.5, 0, 0.4, 0, 0); const p1 = io.readPose();
  assert.equal(io.readModules().find(m => m.name === 'RL').encoderOk, false);
  assert.match(checkModules(io.readModules())[0].text, /RL steering encoder not reading/);
  assert.deepEqual(p1, p0, 'pose held');
});

test('a jammed wheel is caught by the jam detector from pi_io readings', () => {
  const { bus, io } = rig();
  const jam = createJamDetector(); let hit = null;
  bus.setFault(7, 'jammed');
  for (let i = 0; i < 90 && !hit; i++) { io.writeWheels(0, 0.5, 0, 0); bus.step(dt); hit = jam.step(io.readModules(), dt); }
  assert.ok(hit); assert.equal(hit.name, 'RR'); assert.equal(hit.kind, 'drive');
});

test('a motor wired backwards is caught as "turning the wrong way"', () => {
  const { bus, io } = rig();
  const jam = createJamDetector(); let hit = null;
  bus.setFault(5, 'backwards');
  for (let i = 0; i < 120 && !hit; i++) { io.writeWheels(0, 0.5, 0, 0); bus.step(dt); hit = jam.step(io.readModules(), dt); }
  assert.ok(hit, 'detected'); assert.equal(hit.name, 'RL'); assert.equal(hit.kind, 'reverse');
});

test('battery: voltage to percent, low voltage flagged by the check', () => {
  const { bus, io } = rig();
  bus.step(dt);
  near(io.readBattery().robot_pct, (12.6 - 11.5) / 1.2 * 100, 0.01);
  bus.setBusVolts(10.2); bus.step(dt);
  assert.equal(io.readBattery().robot_pct, 0);
  assert.equal(checkModules(io.readModules())[0].kind, 'volts');
});

test('three-module cart: only FL, FR, RL are configured and driven', () => {
  const mods = DEFAULT_MODULES.slice(0, 3).map((m, i) => ({ ...m, ...[{ x: .2, y: .2 }, { x: .2, y: -.2 }, { x: -.2, y: 0 }][i] }));
  const { bus, io, run } = rig({ modules: mods });
  run(1, 0, 0.5, 0, 0);
  assert.ok(!bus.log.some(e => e.id === 7 || e.id === 8), 'RR never touched');
  assert.equal(io.readModules().length, 3);
  assert.deepEqual(checkModules(io.readModules(), ['FL', 'FR', 'RL']), []);
});

test('unwired E-stop or cord input reads as unsafe (the runtime then refuses to drive)', () => {
  const io = createPiIO({ bus: createFakeSparkBus() });
  assert.throws(() => io.estopEngaged()); assert.throws(() => io.readCord()); assert.throws(() => io.readLidar());
});

console.log(`\n${passed} passed, ${failed} failed`);
process.exitCode = failed ? 1 : 0;

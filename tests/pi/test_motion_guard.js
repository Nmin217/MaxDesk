'use strict';
// Slew limiter and jam detector (robot/pi/motion_guard.js): node tests/pi/test_motion_guard.js
const assert = require('node:assert/strict');
const { createSlewLimiter, createJamDetector } = require('../../robot/pi/motion_guard');

let passed = 0, failed = 0;
function test(name, fn) {
  try { fn(); passed++; console.log('PASS ' + name); }
  catch (e) { failed++; console.log('FAIL ' + name + ': ' + e.message); }
}
const dt = 1 / 60, near = (a, b, tol = 1e-9) => Math.abs(a - b) <= tol;

test('slew: a step to 1 m/s ramps at 1.5 m/s^2', () => {
  const s = createSlewLimiter();
  assert.ok(near(s.step(1, 0, 0, dt).vx, 1.5 * dt));
  let o; for (let i = 0; i < 39; i++) o = s.step(1, 0, 0, dt);
  assert.ok(near(o.vx, 1.0) || o.vx < 1.0);
  for (let i = 0; i < 3; i++) o = s.step(1, 0, 0, dt);
  assert.ok(near(o.vx, 1.0));
});

test('slew: slowing down and stopping are never delayed', () => {
  const s = createSlewLimiter();
  for (let i = 0; i < 60; i++) s.step(0.8, 0, 0, dt);
  assert.ok(near(s.step(0.3, 0, 0, dt).vx, 0.3));
  assert.equal(s.step(0, 0, 0, dt).vx, 0);
});

test('slew: a reversal ramps through zero instead of flipping', () => {
  const s = createSlewLimiter();
  for (let i = 0; i < 60; i++) s.step(0.5, 0, 0, dt);
  const o = s.step(-0.5, 0, 0, dt);
  assert.ok(near(o.vx, 0.5 - 1.5 * dt));
});

test('slew: turning rate ramps up, slows instantly, and reverses gradually', () => {
  const s = createSlewLimiter();
  assert.ok(near(s.step(0, 0, 2, dt).omega, 6 * dt));
  for (let i = 0; i < 30; i++) s.step(0, 0, 2, dt);
  assert.equal(s.step(0, 0, 1, dt).omega, 1);
  assert.ok(near(s.step(0, 0, -1, dt).omega, 1 - 6 * dt));
});

test('slew: after reset the next move starts from rest', () => {
  const s = createSlewLimiter();
  for (let i = 0; i < 60; i++) s.step(0.8, 0, 0, dt);
  s.reset();
  assert.ok(near(s.step(0.8, 0, 0, dt).vx, 1.5 * dt));
});

const jammedDrive = { name: 'FL', driveCmdMps: 0.4, driveMps: 0.02, driveAmps: 30, steerErrDeg: 0, steerAmps: 2 };
const freeDrive = { name: 'FL', driveCmdMps: 0.4, driveMps: 0.39, driveAmps: 6, steerErrDeg: 0, steerAmps: 2 };

test('jam: a wheel that will not turn trips after 0.5 s, not before', () => {
  const j = createJamDetector();
  let hit = null, t = 0;
  while (!hit && t < 2) { hit = j.step([freeDrive, { ...jammedDrive, name: 'RR' }], dt); t += dt; }
  assert.ok(hit && hit.name === 'RR' && hit.kind === 'drive', JSON.stringify(hit));
  assert.ok(t >= 0.5 - 1e-9 && t < 0.55, 't=' + t);
  assert.match(hit.text, /RR wheel can't turn/);
});

test('jam: a free wheel, a slow start, or low current never trips', () => {
  const j = createJamDetector();
  for (let i = 0; i < 120; i++) assert.equal(j.step([freeDrive], dt), null);
  for (let i = 0; i < 18; i++) assert.equal(j.step([jammedDrive], dt), null);   // 0.3 s pushing off...
  for (let i = 0; i < 120; i++) assert.equal(j.step([freeDrive], dt), null);    // ...then it rolls
  for (let i = 0; i < 120; i++) assert.equal(j.step([{ ...jammedDrive, driveAmps: 5 }], dt), null);
  assert.equal(j.step(null, dt), null);
});

test('jam: steering stuck far from its target with high current trips; normal slewing does not', () => {
  const j = createJamDetector();
  const slewing = { name: 'FR', driveCmdMps: 0, driveMps: 0, driveAmps: 0, steerErrDeg: 80, steerAmps: 4 };
  for (let i = 0; i < 120; i++) assert.equal(j.step([slewing], dt), null);
  let hit = null;
  for (let i = 0; i < 60 && !hit; i++) hit = j.step([{ ...slewing, steerAmps: 16 }], dt);
  assert.ok(hit && hit.kind === 'steer', JSON.stringify(hit));
});

console.log(`${passed} passed, ${failed} failed`);
if (failed) process.exitCode = 1;

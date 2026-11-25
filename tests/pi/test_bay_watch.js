'use strict';
// robot/pi/bay_watch.js: person vs chair in the open bay, from LiDAR hits.   node tests/pi/test_bay_watch.js
const assert = require('node:assert/strict');
const { createBayWatch, PERSON_CLEAR_MS } = require('../../robot/pi/bay_watch');

let passed = 0, failed = 0;
function test(name, fn) { try { fn(); passed++; console.log('PASS ' + name); } catch (e) { failed++; console.log('FAIL ' + name + ': ' + e.message); } }

// Hit points on the half of a circle facing the desk centre (what a LiDAR inside the frame sees), as rays.
function circle(cx, cy, r) {
  const face = Math.atan2(-cy, -cx), out = [];
  for (let a = -Math.PI / 2; a <= Math.PI / 2 + 1e-9; a += Math.PI / 12) out.push([0, 0, cx + r * Math.cos(face + a), cy + r * Math.sin(face + a), 1]);
  return out;
}
// Bay (defaults): x from -0.39 (open edge) to +0.10, |y| <= 0.50. Chair pushed in sits around x = -0.2.
const column = () => circle(-0.20, 0.00, 0.03);           // office chair gas column, 6 cm
const legL = () => circle(-0.30, 0.12, 0.055);            // lower legs, ~11 cm
const legR = () => circle(-0.30, -0.12, 0.055);

test('empty bay is clear, and a wall far away changes nothing', () => {
  const w = createBayWatch();
  assert.equal(w.step([[0, 0, -2.5, 0, 1], [0, 0, 0, 3, 1]], 0).state, 'clear');
});

test('an empty office chair pushed in (just the gas column) is a chair', () => {
  assert.equal(createBayWatch().step(column(), 0).state, 'chair');
});

test('someone sitting (chair column + two legs) is a person', () => {
  assert.equal(createBayWatch().step([...column(), ...legL(), ...legR()], 0).state, 'person');
});

test('someone standing at the desk (two legs, no chair) is a person', () => {
  assert.equal(createBayWatch().step([...legL(), ...legR()], 0).state, 'person');
});

test('a single leg is a person (when unsure, assume a person)', () => {
  assert.equal(createBayWatch().step(legL(), 0).state, 'person');
});

test('a four-legged chair (thin legs) is a chair', () => {
  const thin = [[-0.32, 0.18], [-0.32, -0.18], [-0.05, 0.18], [-0.05, -0.18]].flatMap(([x, y]) => circle(x, y, 0.012));
  assert.equal(createBayWatch().step(thin, 0).state, 'chair');
});

test('a person stays "person" for 2 s after the legs leave the scan, then clears', () => {
  const w = createBayWatch();
  w.step([...legL(), ...legR()], 0);
  assert.equal(w.step([], 1000).state, 'person');
  assert.equal(w.step([], PERSON_CLEAR_MS - 1).state, 'person');
  assert.equal(w.step([], PERSON_CLEAR_MS + 1).state, 'clear');
});

test('the desk legs and frame beside the bay are ignored', () => {
  const deskLegs = [...circle(-0.02, 0.546, 0.03), ...circle(-0.02, -0.546, 0.03)];
  assert.equal(createBayWatch().step(deskLegs, 0).state, 'clear');
});

test('room to back away: blocked when something is just past the desk\'s forward edge', () => {
  const w = createBayWatch();
  assert.equal(w.step(column(), 0).rearClear, true);
  assert.equal(w.step([...column(), [0, 0, 0.55, 0.1, 1]], 100).rearClear, false);
  assert.equal(w.step([...column(), [0, 0, 1.5, 0.1, 1]], 200).rearClear, true);   // far away is fine
});

test('misses (no hit) and junk rays are ignored', () => {
  assert.equal(createBayWatch().step([[0, 0, -0.2, 0, 0], null, [0, 0, NaN, 0, 1]], 0).state, 'clear');
});

console.log(`\n${passed} passed, ${failed} failed`);
process.exitCode = failed ? 1 : 0;

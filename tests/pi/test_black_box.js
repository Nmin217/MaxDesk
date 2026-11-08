'use strict';
// Black box recorder (robot/pi/black_box.js): node tests/pi/test_black_box.js
const assert = require('node:assert/strict');
const fs = require('fs'), os = require('os'), path = require('path');
const { createBlackBox } = require('../../robot/pi/black_box');

let passed = 0, failed = 0;
function test(name, fn) { try { fn(); passed++; console.log('PASS ' + name); } catch (e) { failed++; console.log('FAIL ' + name + ': ' + e.message); } }
function rig(opts = {}) {
  let ms = 0, n = 0;
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'bb-'));
  const bb = createBlackBox({ dir, nowMs: () => ms, wallClock: () => new Date(Date.UTC(2026, 10, 8, 12, 0, 0) + ms + 1000 * (n++)), ...opts });
  const run = (sec, data = {}) => { for (let i = 0; i < Math.round(sec * 60); i++) { ms += 1000 / 60; bb.record({ ...data, i }); } };
  const files = () => fs.readdirSync(dir).filter(f => f.endsWith('.json')).sort();
  const read = f => JSON.parse(fs.readFileSync(path.join(dir, f), 'utf8'));
  return { bb, run, files, read, dir, get ms() { return ms; } };
}

test('keeps 20 samples a second and only the last 60 s (+ the 5 s after a trigger)', () => {
  const r = rig();
  r.run(120);
  assert.ok(Math.abs(r.bb.size.samples - (60 + 5) * 20) <= 2, 'samples ' + r.bb.size.samples);
});

test('a trigger saves once, 5 s later, with the minute before and the seconds after', () => {
  const r = rig();
  r.run(70, { x: 1 });
  r.bb.event('Driving to you');
  assert.equal(r.bb.trigger('trip ended: jam'), true);
  r.run(4.9);
  assert.equal(r.files().length, 0, 'not yet: still recording what happens next');
  r.run(0.2);
  const f = r.files();
  assert.equal(f.length, 1); assert.match(f[0], /^20261108-1201\d\d-trip-ended-jam\.json$/);
  const d = r.read(f[0]);
  assert.equal(d.kind, 'maxdesk-blackbox'); assert.equal(d.reason, 'trip ended: jam');
  const span = d.samples[d.samples.length - 1].t - d.samples[0].t;
  assert.ok(span > 64 && span < 65.2, 'span ' + span);
  assert.ok(Math.abs(d.samples[d.samples.length - 1].t - d.trigger_t - 5) < 0.1, 'about 5 s after the trigger');
  assert.ok(d.events.some(e => e.text === 'Driving to you') && d.events.some(e => /Trigger: trip ended: jam/.test(e.text)));
});

test('one incident, one file: triggers during the wait or within 10 s after are events only', () => {
  const r = rig();
  r.run(10);
  r.bb.trigger('hardware problem'); r.run(1);
  assert.equal(r.bb.trigger('trip ended: hardware'), false);
  r.run(6);
  assert.equal(r.bb.trigger('lost position'), false, 'within 10 s of the save');
  r.run(12);
  assert.equal(r.bb.trigger('E-stop'), true, 'a new incident');
  r.run(6);
  assert.deepEqual(r.files().map(f => f.replace(/^\d+-\d+-/, '')), ['hardware-problem.json', 'e-stop.json']);
});

test('LiDAR keyframes at most once a second, inside the saved window', () => {
  const r = rig();
  for (let i = 0; i < 600; i++) { r.run(1 / 60); r.bb.keyframe([[i, 0]]); }
  r.bb.trigger('test'); r.run(6);
  const d = r.read(r.files()[0]);
  assert.ok(d.keyframes.length >= 9 && d.keyframes.length <= 11, '10 s of scans at 1 per second: ' + d.keyframes.length);
});

test('keeps only the newest 20 files', () => {
  const r = rig({ keep: 3, quiet: 0 });
  for (let i = 0; i < 5; i++) { r.bb.trigger('n' + i); r.run(5.1); }
  assert.deepEqual(r.files().map(f => f.replace(/^\d+-\d+-/, '')), ['n2.json', 'n3.json', 'n4.json']);
});

test('flush saves a pending incident at once (shutting down)', () => {
  const r = rig();
  r.run(3); r.bb.trigger('E-stop'); r.run(1);
  const f = r.bb.flush();
  assert.ok(f && fs.existsSync(f)); assert.equal(r.bb.pending, null);
});

test('a folder that cannot be written does not throw into the control loop', () => {
  const blocker = path.join(fs.mkdtempSync(path.join(os.tmpdir(), 'bb-')), 'file');
  fs.writeFileSync(blocker, 'x');
  let ms = 0; const bb = createBlackBox({ dir: path.join(blocker, 'sub'), nowMs: () => ms });
  bb.trigger('x');
  for (let i = 0; i < 400; i++) { ms += 16.7; bb.record({}); }
  assert.ok(bb.lastError, 'error kept for inspection'); assert.equal(bb.saved.length, 0);
});

console.log(`\n${passed} passed, ${failed} failed`);
process.exitCode = failed ? 1 : 0;

'use strict';
// Hardware health check (robot/pi/hardware_check.js): node tests/pi/test_hardware_check.js
const assert = require('node:assert/strict');
const { checkModules, createHardwareMonitor } = require('../../robot/pi/hardware_check');

let passed = 0, failed = 0;
function test(name, fn) {
  try { fn(); passed++; console.log('PASS ' + name); }
  catch (e) { failed++; console.log('FAIL ' + name + ': ' + e.message); }
}
const mod = (name, over = {}) => ({ name, present: true, encoderOk: true, angleDeg: 12, busVolts: 12.5, faults: [], ...over });
const four = (over = {}) => ['FL', 'FR', 'RL', 'RR'].map(n => mod(n, over[n]));

test('all four present and reading: no issues', () => assert.deepEqual(checkModules(four()), []));
test('a module not reported at all is missing', () => {
  const i = checkModules(four().filter(m => m.name !== 'RL'));
  assert.equal(i.length, 1); assert.equal(i[0].kind, 'missing'); assert.match(i[0].text, /RL module not found/);
});
test('present:false (a SPARK not answering) is missing, and nothing else is reported for it', () => {
  const i = checkModules(four({ FR: { present: false, encoderOk: false } }));
  assert.equal(i.length, 1); assert.match(i[0].text, /FR not answering on CAN/);
});
test('encoderOk:false or a non-number angle is an encoder problem', () => {
  assert.equal(checkModules(four({ FL: { encoderOk: false } }))[0].kind, 'encoder');
  assert.equal(checkModules(four({ RR: { angleDeg: NaN } }))[0].kind, 'encoder');
});
test('low voltage and controller faults are reported with the module name', () => {
  const i = checkModules(four({ FL: { busVolts: 9.8 }, RR: { faults: ['overcurrent'] } }));
  assert.deepEqual(i.map(x => x.kind), ['volts', 'fault']);
  assert.match(i[0].text, /9\.8 V at FL/); assert.match(i[1].text, /RR controller fault: overcurrent/);
});
test('fields the IO does not report are not checked', () => {
  assert.deepEqual(checkModules([{ name: 'FL' }, { name: 'FR' }, { name: 'RL' }], ['FL', 'FR', 'RL']), []);
});
test('no readings at all (undefined) is a problem', () => assert.equal(checkModules(undefined)[0].kind, 'no-data'));
test('a 3-module cart only expects its three', () => assert.deepEqual(checkModules(four().slice(0, 3), ['FL', 'FR', 'RL']), []));
test('monitor: OK at start-up drives at once; a problem blocks until 2 s of normal readings', () => {
  const m = createHardwareMonitor(['FL', 'FR', 'RL', 'RR']);
  assert.equal(m.state().blocked, true, 'blocked before the first check');
  assert.equal(m.step(four(), .5).blocked, false);
  assert.equal(m.step(four({ FL: { encoderOk: false } }), .5).blocked, true);
  for (let k = 0; k < 3; k++) assert.equal(m.step(four(), .5).blocked, true, 'within 2 s');
  assert.equal(m.step(four(), .5).blocked, false, 'after 2 s');
});

console.log(`\n${passed} passed, ${failed} failed`);
process.exitCode = failed ? 1 : 0;

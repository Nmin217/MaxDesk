'use strict';
// robot/pi/spark_bus.js + spark_can.py (pretend SPARKs, real time) + tools/pi_bench.js end to end.
// Checks the Node and Python halves agree on messages and units, and that a dying helper means "stopped".
// node tests/pi/test_spark_bus.js   (about 6 s)
const assert = require('node:assert/strict');
const { createSparkBus } = require('../../robot/pi/spark_bus');
const { createBench, BENCH_MODULES } = require('../../tools/pi_bench');
const S = require('../../robot/pi/swerve');

let passed = 0, failed = 0;
async function test(name, fn) { try { await fn(); passed++; console.log('PASS ' + name); } catch (e) { failed++; console.log('FAIL ' + name + ': ' + (e.stack || e.message)); } }
const sleep = ms => new Promise(r => setTimeout(r, ms));
const wrapDeg = d => ((d + 180) % 360 + 360) % 360 - 180;
const line = d => Math.min(Math.abs(wrapDeg(d)), Math.abs(wrapDeg(d + 180)));

function pad() {
  const p = { connected: true, axes: {}, buttons: {}, edges: new Set(),
    axis: n => p.axes[n] || 0, button: n => !!p.buttons[n], pressed: n => { const w = p.edges.has(n); p.edges.delete(n); return w; } };
  return p;
}

const buses = [];
const track = b => { buses.push(b); return b; };
async function runFor(ms, fn) { const end = Date.now() + ms; while (Date.now() < end) { fn(); await sleep(20); } }

(async () => {
  await test('status arrives, unknown IDs stay null, the turn SPARK reports its absolute encoder', async () => {
    const bus = track(createSparkBus({ sim: true, simIds: [1, 2] }));
    bus.configure(1, { kind: 'drive' }); bus.configure(2, { kind: 'turn' });
    await sleep(300);
    const d = bus.status(1), t = bus.status(2);
    assert.ok(d && d.ageMs < 100, 'drive status');
    assert.equal(d.absRot, undefined);
    assert.ok(t && Number.isFinite(t.absRot), 'turn absRot');
    assert.ok(t.busV > 11 && t.busV < 13, `busV ${t.busV}`);
    assert.equal(bus.status(9), null);
    bus.close(); await sleep(200);
    assert.equal(bus.alive, false);
  });

  await test('drive rpm and steering rotations get through in the right units', async () => {
    const bus = track(createSparkBus({ sim: true, simIds: [1, 2] }));
    bus.configure(1, { kind: 'drive' }); bus.configure(2, { kind: 'turn' });
    await runFor(1200, () => { bus.heartbeat(true); bus.setVelocity(1, 1500); bus.setPosition(2, 0.25); });
    const d = bus.status(1), t = bus.status(2);
    assert.ok(Math.abs(d.velocityRpm - 1500) < 150, `rpm ${d.velocityRpm}`);
    assert.ok(Math.abs(t.absRot - 0.25) < 0.01, `rot ${t.absRot}`);
    bus.close(); await sleep(100);
  });

  await test('stop sending heartbeat(true): the pretend SPARKs stop by themselves', async () => {
    const bus = track(createSparkBus({ sim: true, simIds: [1] }));
    bus.configure(1, { kind: 'drive' });
    await runFor(800, () => { bus.heartbeat(true); bus.setVelocity(1, 2000); });
    await sleep(600);   // Node "froze": nothing sent
    assert.ok(Math.abs(bus.status(1).velocityRpm) < 100, `still ${bus.status(1).velocityRpm} rpm`);
    bus.close(); await sleep(100);
  });

  await test('a dead helper makes every reading stale (the hardware check then stops the desk)', async () => {
    const bus = track(createSparkBus({ sim: true, simIds: [1] }));
    bus.configure(1, { kind: 'drive' });
    await sleep(300);
    bus.close(); await sleep(300);
    assert.equal(bus.alive, false);
    assert.ok(bus.status(1).ageMs > 1e5);
  });

  await test('missing python reports a clear error instead of crashing', async () => {
    let msg = null;
    const bus = track(createSparkBus({ sim: true, python: 'definitely-not-python', onError: m => { msg = m; } }));
    await sleep(300);
    assert.equal(bus.alive, false);
    assert.match(msg, /cannot start the SPARK helper/);
  });

  await test('pi_bench end to end on the pretend SPARKs: Joystick + A lines every wheel up', async () => {
    const bus = track(createSparkBus({ sim: true }));
    const p = pad(); const lines = [];
    const t0 = Date.now();
    const bench = createBench({ bus, pad: p, now: () => (Date.now() - t0) / 1000, print: m => lines.push(m) });
    await runFor(300, () => bench.step());
    p.edges.add('start'); bench.step();
    p.edges.add('start'); bench.step();
    assert.equal(bench.mode, 'Joystick');
    p.buttons.rb = true; p.buttons.a = true;
    await runFor(1500, () => bench.step());
    assert.equal(bench.enabled, true, lines.join('\n'));
    bench.angles().forEach((a, i) => assert.ok(line(a * 180 / Math.PI) < 3, `${BENCH_MODULES[i].name} at ${a * 180 / Math.PI}`));
    p.buttons.a = false; p.axes.ly = -1;
    await runFor(1000, () => bench.step());
    const rpm = BENCH_MODULES.map(m => bus.status(m.driveId).velocityRpm);
    const cap = 0.15 * S.WHEEL_FREE_SPEED / S.DRIVE_VELOCITY_FACTOR;
    rpm.forEach(v => assert.ok(Math.abs(Math.abs(v) - cap) < cap * 0.15, `rpm ${v} vs ${cap}`));
    p.buttons.rb = false;
    await runFor(500, () => bench.step());
    BENCH_MODULES.forEach(m => assert.ok(Math.abs(bus.status(m.driveId).velocityRpm) < 50));
    bus.close(); await sleep(100);
  });

  for (const b of buses) b.close();
  console.log(`\n${passed} passed, ${failed} failed`);
  process.exitCode = failed ? 1 : 0;
})();

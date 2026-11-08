'use strict';
// End to end, no hardware: runtime -> brain in hardware mode -> pi_io -> fake SPARKs -> fake physics with wheel
// slip -> fake LiDAR -> back into the brain.   node tests/pi/test_hw_loop.js
// If any frame, unit or sign is wrong anywhere in the chain, the desk ends up somewhere else or hits a box.
const http = require('http');
const fs = require('fs'), os = require('os'), path = require('path');
const assert = require('node:assert/strict');
process.env.MAXDESK_DATA_DIR = fs.mkdtempSync(path.join(os.tmpdir(), 'maxdesk-hw-'));
// The brain makes a few random choices (search tie-breaks, noise). A fixed seed makes every run of this file identical.
let seed = 12345;
const SEED_OFFSET = +(process.env.SEED || 0);
const reseed = (n) => { seed = n + SEED_OFFSET; };
Math.random = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
// The planner times itself to decide when to re-plan; on a real clock a busy computer changes the run. Fake clock here.
let fakeMs = 0;
Object.defineProperty(globalThis, 'performance', { value: { now: () => fakeMs }, configurable: true, writable: true });
const { createRuntime } = require('../../robot/pi/runtime');
const { createPiIO, DEFAULT_MODULES } = require('../../robot/pi/pi_io');
const { createFakeSparkBus } = require('../../robot/pi/fake_spark_bus');
const { createFakeWorld, PX } = require('./fake_world');

function stubBridge() {
  const pending = [];
  const server = http.createServer((req, res) => {
    const url = req.url.split('?')[0], send = (c, o) => { const b = JSON.stringify(o); res.writeHead(c, { 'Content-Type': 'application/json' }); res.end(b); };
    if (url === '/v1/health') return send(200, { ok: true });
    if (url === '/v1/sim/poll') return send(200, { commands: pending.splice(0), epoch: 1 });
    let body = ''; req.on('data', c => body += c); req.on('end', () => send(200, { ok: true }));
  });
  return new Promise(r => server.listen(0, '127.0.0.1', () => r({ url: `http://127.0.0.1:${server.address().port}`, queue: c => pending.push(c), close: () => new Promise(q => server.close(q)) })));
}

let passed = 0, failed = 0;
async function test(name, fn) { try { await fn(); passed++; console.log('PASS ' + name); } catch (e) { failed++; console.log('FAIL ' + name + ': ' + e.message); } }
const dt = 1 / 60;
const inputs = { estopEngaged: () => false, readCord: () => ({ plugged: false, interlock_on: true }) };

async function rig(worldOpts = {}, { gyro = !!process.env.GYRO } = {}) {
  const bridge = await stubBridge();
  const bus = createFakeSparkBus();
  const world = createFakeWorld({ bus, modules: DEFAULT_MODULES, seed: 1 + SEED_OFFSET, ...worldOpts });
  const io = createPiIO({ bus, inputs, lidar: world.lidar, imu: gyro ? { yaw: () => world.pose.heading - Math.PI / 2 + world.time * 1e-4 } : null });
  const runtime = createRuntime({ io, bridgeUrl: bridge.url, token: 't', statePushMs: 1e12, pollMs: 1e12, nowMs: () => fakeMs, autoLoop: false,
    dataDir: fs.mkdtempSync(path.join(os.tmpdir(), 'maxdesk-hw-')) });   // each test starts with no saved map
  await runtime.start();
  // Like the real loop: the bridge is polled every 100 ms (6 ticks); the runtime stops the wheels if it goes quiet for 2 s.
  let n6 = 0;
  const tick = async (n = 1) => { for (let i = 0; i < n; i++) { world.step(dt); fakeMs += dt * 1000; if (++n6 % 6 === 0) await runtime.pollNow(); await runtime.tickOnce(dt); } };
  const believed = () => { const T = runtime.sim.getState().T; return { x: T.x * PX, y: -T.y * PX }; };
  const err = () => { const b = believed(), p = world.pose; return Math.hypot(b.x - p.x, b.y - p.y); };
  const summon = async (x, y) => { bridge.queue({ type: 'summon', x, y, epoch: 1, id: Math.random() }); await runtime.pollNow(); };
  const done = async () => { runtime.stop(); await bridge.close(); };
  return { bus, world, io, runtime, tick, believed, err, summon, done, queue: c => bridge.queue(c) };
}

(async () => {
  async function learnThenSummon(gyro, worstLimit) {
    reseed(12345);
    const r = await rig({}, { gyro });
    try {
      await r.tick(120);
      r.runtime.sim.clickLearn();                         // the mapping tour, as the app's "Learn the room" button
      let t = 0, worst = 0;
      for (; t < 400 && (t < 1 || r.runtime.sim.getState().active || r.runtime.sim.getState().parkTh !== null); t += dt) { await r.tick(); worst = Math.max(worst, r.err()); }
      const learnSec = Math.round(t), learnWorst = worst;
      await r.tick(120);
      await r.summon(14, 8.6);
      worst = 0;
      for (t = 0; t < 150 && (t < 1 || r.runtime.sim.getState().active || r.runtime.sim.getState().parkTh !== null); t += dt) { await r.tick(); worst = Math.max(worst, r.err()); }
      const ex = r.runtime.sim.getExtras(), p = r.world.pose, toUser = Math.hypot(p.x - 14, p.y + 8.6);
      console.log(`    learn the room: ${learnSec} s, worst ${(learnWorst * 100).toFixed(0)} cm | summon: ${Math.round(t)} s, ${ex.trips[0].outcome}, ` +
        `${toUser.toFixed(2)} m from the user, worst ${(worst * 100).toFixed(0)} cm, end ${(r.err() * 100).toFixed(0)} cm, contacts ${r.world.contacts}`);
      assert.equal(ex.trips[0].outcome, 'arrived');
      assert.ok(toUser > 0.9 && toUser < 2.0, 'parked next to the user');
      assert.equal(r.world.contacts, 0, 'no contact');
      assert.ok(worst < worstLimit && r.err() < 0.08, `worst ${(worst * 100).toFixed(0)} cm: limit ${worstLimit * 100} cm`);
    } finally { await r.done(); }
  }

  // Limits from runs over several seeds (see robot/pi/README.md): without a gyro the heading creeps between LiDAR
  // corrections (worst 4-32 cm on a summon); with one it stays within ~10 cm.
  await test('the real-life sequence on fake hardware, no gyro: Learn the room, then summon: arrives, no contact, under 40 cm', () => learnThenSummon(false, 0.40));
  await test('the same with a gyro: under 15 cm the whole way', () => learnThenSummon(true, 0.15));

  async function firstTrip(gyro, worstLimit, slip) {
    reseed(12345);
    const r = await rig(slip ? { slip } : {}, { gyro });
    try {
      await r.tick(120);                                  // 2 s at the dock: first scans map the room
      await r.summon(14, 8.6);                            // the user, metres on the floor plan (y down)
      let t = 0, worst = 0;
      for (; t < 150 && (t < 1 || r.runtime.sim.getState().active || r.runtime.sim.getState().parkTh !== null); t += dt) { await r.tick(); worst = Math.max(worst, r.err()); }
      const ex = r.runtime.sim.getExtras(), p = r.world.pose;
      const toUser = Math.hypot(p.x - 14, p.y + 8.6);
      console.log(`    ${Math.round(t)} s, ended ${ex.trips[0] && ex.trips[0].outcome}, ${toUser.toFixed(2)} m from the user, ` +
        `position error worst ${(worst * 100).toFixed(0)} cm / end ${(r.err() * 100).toFixed(0)} cm, contacts ${r.world.contacts}`);
      assert.equal(r.world.contacts, 0, 'no contact');
      // On a first trip the LiDAR correction only trusts map marks at least 8 s old (what keeps bump recovery safe),
      // so there is less to match against; if error ever left ghost marks that block the last metre, the desk must
      // stop and say so ("Boxed in"), never push on.
      if (worstLimit) assert.ok(worst < worstLimit, `worst ${(worst * 100).toFixed(0)} cm: limit ${worstLimit * 100} cm`);
      assert.ok(r.err() < 0.10, 'accurate on arrival');
      assert.ok(['arrived', 'blocked'].includes(ex.trips[0].outcome), 'arrives, or stops and says so');
      if (ex.trips[0].outcome === 'blocked') assert.match(ex.alert_text, /boxed in/i);
    } finally { await r.done(); }
  }

  // An unknown room (no saved map). Measured over seeds, worst error at any moment: no gyro 35-37 cm, gyro 17 cm;
  // heavy slip (5% + 1.7 deg/m heading creep): no gyro 100-125 cm, gyro 39-41 cm. Every run arrived with no contact
  // and ended within ~7 cm. See robot/pi/README.md ("How accurate is it").
  await test('first trip into an unknown room, no gyro: no contact, arrives or stops with a message, under 45 cm', () => firstTrip(false, 0.45));
  await test('first trip into an unknown room, gyro: under 25 cm', () => firstTrip(true, 0.25));
  const heavy = { scale: 0.05, yawPerM: 0.03 };
  await test('heavy wheel slip, gyro: under 50 cm, arrives, no contact', () => firstTrip(true, 0.50, heavy));
  await test('heavy wheel slip, no gyro: badly off mid-trip but still no contact and accurate on arrival', () => firstTrip(false, null, heavy));

  await test('LiDAR unplugged mid-trip: wheels hold within 0.3 s, trip ends after 2 s with a plain message', async () => {
    const r = await rig();
    try {
      await r.tick(120); await r.summon(14, 8.6); await r.tick(240);
      assert.ok(r.runtime.sim.getState().active, 'under way');
      r.world.stopScans();
      await r.tick(Math.round(0.45 / dt));
      assert.ok(!r.bus.truth(1).enabled, 'SPARKs disabled');
      const p0 = r.world.pose; await r.tick(60); const p1 = r.world.pose;
      assert.ok(Math.hypot(p1.x - p0.x, p1.y - p0.y) < 0.02, 'not moving');
      await r.tick(150);
      const ex = r.runtime.sim.getExtras();
      assert.equal(r.runtime.sim.getState().active, false); assert.equal(ex.trips[0].outcome, 'hardware');
      assert.match(ex.hardware.problem, /LiDAR not sending data/);
    } finally { await r.done(); }
  });

  await test('a SPARK unplugged mid-trip: stops, trip ends as a hardware problem naming the module', async () => {
    const r = await rig();
    try {
      await r.tick(120); await r.summon(14, 8.6); await r.tick(240);
      r.bus.setFault(3, 'unplugged');
      await r.tick(60);
      const ex = r.runtime.sim.getExtras();
      assert.equal(r.runtime.sim.getState().active, false);
      assert.match(ex.hardware.problem, /FR not answering on CAN/);
      assert.ok(!r.bus.truth(1).enabled, 'everything disabled');
    } finally { await r.done(); }
  });

  const fs2 = require('fs');
  const bbFiles = (r) => r.runtime.blackBox.saved.map(f => JSON.parse(fs2.readFileSync(f, 'utf8')));

  await test('black box: a jammed wheel mid-trip saves one file naming the module, with the current spike and the stop', async () => {
    reseed(12345);
    const r = await rig();
    try {
      await r.tick(120); await r.summon(14, 8.6); await r.tick(300);
      r.bus.setFault(7, 'jammed');                       // RR drive
      await r.tick(Math.round(7 / dt));                  // jam caught in ~0.5 s, then 5 s more recording
      const files = bbFiles(r);
      assert.equal(files.length, 1, 'one incident, one file');
      const d = files[0];
      assert.match(d.reason, /RR wheel can't turn/);
      const before = d.samples.filter(x => x.t < d.trigger_t), afterT = d.samples.filter(x => x.t > d.trigger_t);
      assert.ok(before.some(x => x.out === 'drive'), 'was driving before');
      assert.ok(afterT.every(x => x.out !== 'drive'), 'never drove after');
      const rrAmps = d.samples.filter(x => Math.abs(x.t - d.trigger_t) < 0.6).map(x => x.m.find(m => m[0] === 'RR')[4]);
      assert.ok(Math.max(...rrAmps) > 20, 'RR current spike recorded: ' + Math.max(...rrAmps));
      assert.ok(d.keyframes.length > 5 && d.keyframes[0].points.length > 50, 'LiDAR snapshots');
      assert.ok(d.events.some(e => /Driving to you/.test(e.text)), 'status messages recorded');
      console.log(`    ${d.samples.length} samples over ${(d.samples[d.samples.length - 1].t - d.samples[0].t).toFixed(1)} s, ${d.keyframes.length} LiDAR snapshots, ${d.events.length} events, ${(JSON.stringify(d).length / 1024).toFixed(0)} KB`);
    } finally { await r.done(); }
  });

  await test('black box: the LiDAR going quiet saves a file saying so', async () => {
    reseed(12345);
    const r = await rig();
    try {
      await r.tick(120); await r.summon(14, 8.6); await r.tick(240);
      r.world.stopScans();
      await r.tick(Math.round(6 / dt));
      const files = bbFiles(r);
      assert.equal(files.length, 1); assert.equal(files[0].reason, 'LiDAR stopped sending');
      assert.ok(files[0].samples.some(x => x.out === 'stop:lidar old'));
    } finally { await r.done(); }
  });

  await test('black box: "save what just happened" from the app', async () => {
    reseed(12345);
    const r = await rig();
    try {
      await r.tick(300);
      r.queue({ type: 'blackbox', epoch: 1, id: 99 }); await r.tick(Math.round(5.5 / dt));
      const files = bbFiles(r);
      assert.equal(files.length, 1); assert.equal(files[0].reason, 'saved from the app');
    } finally { await r.done(); }
  });

  console.log(`\n${passed} passed, ${failed} failed`);
  process.exitCode = failed ? 1 : 0;
})();

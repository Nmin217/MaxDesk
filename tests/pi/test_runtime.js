'use strict';

const http = require('http');
const assert = require('node:assert/strict');
const { createMockIO } = require('../../robot/pi/mock_io');
const { createNullIO } = require('../../robot/pi/null_io');
const { createRuntime } = require('../../robot/pi/runtime');

function startStubBridge() {
  const pending = [];
  let mode = 'ok';
  const server = http.createServer((req, res) => {
    const url = req.url.split('?')[0];
    const send = (code, obj) => {
      const b = JSON.stringify(obj);
      res.writeHead(code, { 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(b) });
      res.end(b);
    };
    if (req.method === 'GET' && url === '/v1/health') {
      return mode === 'ok' ? send(200, { ok: true }) : send(503, { ok: false });
    }
    if (req.method === 'GET' && url === '/v1/sim/poll') {
      if (mode !== 'ok') return send(503, { error: 'unavailable' });
      return send(200, { commands: pending.splice(0), epoch: 1 });
    }
    if (req.method === 'POST' && (url === '/v1/sim/state' || url === '/v1/sim/map')) {
      let body = '';
      req.on('data', c => body += c);
      req.on('end', () => send(200, { ok: true }));
      return;
    }
    return send(404, { error: 'not found' });
  });
  return new Promise((resolve) => {
    server.listen(0, '127.0.0.1', () => {
      const port = server.address().port;
      resolve({
        url: `http://127.0.0.1:${port}`,
        queue: (cmd) => pending.push(cmd),
        setMode: (m) => { mode = m; },
        close: () => new Promise((r) => server.close(r))
      });
    });
  });
}

let passed = 0, failed = 0;
async function test(name, fn) {
  try { await fn(); console.log(`PASS ${name}`); passed++; }
  catch (e) { console.log(`FAIL ${name}: ${e.message}`); failed++; }
}

(async () => {
  await test('null IO refuses movement', () => {
    const io = createNullIO();
    assert.equal(io.estopEngaged(), true);
    io.writeWheels(1, 0, 0);
    io.stopAll();
    assert.equal(io.getWrites(), 1);
    assert.equal(io.getStops(), 1);
  });

  await test('mock IO starts stationary', () => {
    const io = createMockIO();
    const v = io.readVelocity();
    assert.equal(v.vx, 0); assert.equal(v.vy, 0); assert.equal(v.omega, 0);
    assert.equal(io.isStopped(), true);
  });

  await test('runtime refuses when bridge is unreachable', async () => {
    const io = createMockIO();
    const runtime = createRuntime({ io, bridgeUrl: 'http://127.0.0.1:1', token: 't' });
    assert.equal(await runtime.start(), false);
    await runtime.tickOnce(1 / 60);
    assert.equal(io.getWrites().length, 0);
  });

  await test('accepted summon produces motion output', async () => {
    const bridge = await startStubBridge();
    try {
      const io = createMockIO();
      const runtime = createRuntime({ io, bridgeUrl: bridge.url, token: 't' });
      assert.equal(await runtime.start(), true);
      bridge.queue({ type: 'summon', x: 4, y: 3, epoch: 1, id: 1 });
      await runtime.pollNow();
      for (let i = 0; i < 60; i++) await runtime.tickOnce(1 / 60);
      const writes = io.getWrites();
      assert.ok(writes.length > 0, 'expected writeWheels calls');
      const maxMag = writes.reduce((m, w) =>
        Math.max(m, Math.hypot(w.vx, w.vy) + Math.abs(w.omega)), 0);
      assert.ok(maxMag > 0.01, 'expected nonzero commanded velocity');
      runtime.stop();
    } finally { await bridge.close(); }
  });

  await test('Stop command calls stopAll', async () => {
    const bridge = await startStubBridge();
    try {
      const io = createMockIO();
      const runtime = createRuntime({ io, bridgeUrl: bridge.url, token: 't' });
      await runtime.start();
      bridge.queue({ type: 'summon', x: 4, y: 3, epoch: 1, id: 1 });
      await runtime.pollNow();
      for (let i = 0; i < 30; i++) await runtime.tickOnce(1 / 60);
      const stopsBefore = io.getStops().length;
      bridge.queue({ type: 'stop', epoch: 1, id: 2 });
      await runtime.pollNow();
      for (let i = 0; i < 30; i++) await runtime.tickOnce(1 / 60);
      assert.ok(io.getStops().length > stopsBefore, 'stopAll should be called after Stop');
      runtime.stop();
    } finally { await bridge.close(); }
  });

  await test('bridge 503 prevents motion output', async () => {
    const bridge = await startStubBridge();
    try {
      const io = createMockIO();
      const fakeTime = { t: 1_000_000 };
      const runtime = createRuntime({
        io, bridgeUrl: bridge.url, token: 't',
        nowMs: () => fakeTime.t, statePushMs: 1e12, pollMs: 1e12
      });
      assert.equal(await runtime.start(), true);
      bridge.queue({ type: 'summon', x: 4, y: 3, epoch: 1, id: 1 });
      await runtime.pollNow();
      for (let i = 0; i < 30; i++) await runtime.tickOnce(1 / 60);
      assert.ok(io.getWrites().length > 0, 'should move while bridge is fresh');
      bridge.setMode('fail');
      fakeTime.t += 3000; // past staleBridgeMs
      const writesBefore = io.getWrites().length;
      const stopsBefore = io.getStops().length;
      for (let i = 0; i < 30; i++) await runtime.tickOnce(1 / 60);
      assert.equal(io.getWrites().length, writesBefore, 'no new writes after stale bridge');
      assert.ok(io.getStops().length > stopsBefore, 'stopAll should be called');
      runtime.stop();
    } finally { await bridge.close(); }
  });

  await test('e-stop prevents motion output', async () => {
    const bridge = await startStubBridge();
    try {
      const io = createMockIO();
      const runtime = createRuntime({ io, bridgeUrl: bridge.url, token: 't' });
      await runtime.start();
      bridge.queue({ type: 'summon', x: 4, y: 3, epoch: 1, id: 1 });
      await runtime.pollNow();
      for (let i = 0; i < 30; i++) await runtime.tickOnce(1 / 60);
      const writesBefore = io.getWrites().length;
      assert.ok(writesBefore > 0);
      io.setEstop(true);
      for (let i = 0; i < 30; i++) await runtime.tickOnce(1 / 60);
      assert.equal(io.getWrites().length, writesBefore, 'no writes after E-stop');
      runtime.stop();
    } finally { await bridge.close(); }
  });

  // ---- regression tests ----
  // Start a runtime on a stub bridge, summon, run n ticks. Returns { io, runtime, bridge }.
  async function summoned(io, opts = {}, n = 60) {
    const bridge = await startStubBridge();
    const runtime = createRuntime({ io, bridgeUrl: bridge.url, token: 't', ...opts });
    assert.equal(await runtime.start(), true);
    bridge.queue({ type: 'summon', x: 4, y: 3, epoch: 1, id: 1 });
    await runtime.pollNow();
    for (let i = 0; i < n; i++) await runtime.tickOnce(1 / 60).catch(() => {});
    return { io, runtime, bridge };
  }
  const done = async ({ runtime, bridge }) => { runtime.stop(); await bridge.close(); };

  await test('units: summon commands m/s within 1.0 m/s and 2.0 rad/s, nonzero', async () => {
    const io = createMockIO();
    const ctx = await summoned(io, {}, 120);
    try {
      const w = io.getWrites();
      assert.ok(w.length > 0, 'expected writes');
      const vmax = Math.max(...w.map(x => Math.hypot(x.vx, x.vy)));
      const wmax = Math.max(...w.map(x => Math.abs(x.omega)));
      assert.ok(vmax > 0.05, `expected nonzero speed, got ${vmax}`);
      assert.ok(vmax <= 1.0 + 1e-9, `speed ${vmax} > 1.0 m/s`);
      assert.ok(wmax <= 2.0 + 1e-9, `omega ${wmax} > 2.0 rad/s`);
    } finally { await done(ctx); }
  });

  await test('units: field frame is y-up (vy = -sim vy * PX, omega = -sim w)', async () => {
    const io = createMockIO();
    const bridge = await startStubBridge();
    const runtime = createRuntime({ io, bridgeUrl: bridge.url, token: 't' });
    try {
      await runtime.start();
      bridge.queue({ type: 'summon', x: 4, y: 3, epoch: 1, id: 1 });
      await runtime.pollNow();
      for (let i = 0; i < 90; i++) {
        await runtime.tickOnce(1 / 60);
        const s = runtime.sim.getState(), last = io.getWrites().slice(-1)[0];
        if (!last || Math.hypot(s.T.vx, s.T.vy) < 1) continue;
        assert.ok(Math.abs(last.vx - s.T.vx * 0.02) < 1e-6, 'vx = sim vx * PX');
        assert.ok(Math.abs(last.vy + s.T.vy * 0.02) < 1e-6, 'vy = -sim vy * PX');
        assert.ok(Math.abs(last.omega + s.T.w) < 1e-6, 'omega = -sim w');
        return;
      }
      assert.fail('never saw motion');
    } finally { runtime.stop(); await bridge.close(); }
  });

  await test('cord lock: plugged cord (interlock on) -> no wheel motion on summon', async () => {
    const io = createMockIO();
    io.setCord(true, true);
    const ctx = await summoned(io);
    try {
      assert.equal(io.getWrites().length, 0, 'no writeWheels with cord plugged');
      assert.ok(io.getStops().length > 0, 'stopAll while cord plugged');
    } finally { await done(ctx); }
  });

  await test('cord lock: readCord throwing fails closed', async () => {
    const io = createMockIO();
    io.readCord = () => { throw new Error('i2c'); };
    const ctx = await summoned(io);
    try {
      assert.equal(io.getWrites().length, 0, 'no writeWheels when cord state unknown');
      assert.ok(io.getStops().length > 0, 'stopAll when cord state unknown');
    } finally { await done(ctx); }
  });

  await test('exceptions: throwing estopEngaged -> stopAll, never writeWheels, no reject', async () => {
    const io = createMockIO();
    io.estopEngaged = () => { throw new Error('gpio'); };
    const bridge = await startStubBridge();
    const runtime = createRuntime({ io, bridgeUrl: bridge.url, token: 't' });
    try {
      await runtime.start();
      bridge.queue({ type: 'summon', x: 4, y: 3, epoch: 1, id: 1 });
      await runtime.pollNow();
      let rejected = 0;
      for (let i = 0; i < 30; i++) await runtime.tickOnce(1 / 60).catch(() => { rejected++; });
      assert.equal(io.getWrites().length, 0, 'no writeWheels');
      assert.ok(io.getStops().length > 0, 'stopAll called');
      assert.equal(rejected, 0, 'tickOnce must not reject');
    } finally { runtime.stop(); await bridge.close(); }
  });

  await test('exceptions: throwing sim.update mid-trip -> stopAll, no new writes', async () => {
    const io = createMockIO();
    const ctx = await summoned(io, {}, 30);
    try {
      assert.ok(io.getWrites().length > 0);
      const wb = io.getWrites().length, sb = io.getStops().length;
      ctx.runtime.sim.update = () => { throw new Error('sim bug'); };
      for (let i = 0; i < 5; i++) await ctx.runtime.tickOnce(1 / 60).catch(() => {});
      assert.equal(io.getWrites().length, wb, 'no new writes');
      assert.ok(io.getStops().length > sb, 'stopAll called');
      assert.equal(io.isStopped(), true);
    } finally { await done(ctx); }
  });

  await test('e-stop release: clears latch, no auto-resume, next summon moves', async () => {
    const io = createMockIO();
    const ctx = await summoned(io, {}, 30);
    try {
      io.setEstop(true);
      for (let i = 0; i < 10; i++) await ctx.runtime.tickOnce(1 / 60);
      io.setEstop(false);
      const wb = io.getWrites().length;
      for (let i = 0; i < 30; i++) await ctx.runtime.tickOnce(1 / 60);
      assert.equal(io.getWrites().length, wb, 'no motion between release and new summon');
      assert.equal(ctx.runtime.sim.getState().active, false, 'trip not resumed');
      ctx.bridge.queue({ type: 'summon', x: 4, y: 3, epoch: 1, id: 2 });
      await ctx.runtime.pollNow();
      for (let i = 0; i < 60; i++) await ctx.runtime.tickOnce(1 / 60);
      const w = io.getWrites().slice(wb);
      assert.ok(w.some(x => Math.hypot(x.vx, x.vy) > 0.01), 'summon after release should move');
    } finally { await done(ctx); }
  });

  await test('http timeout: hung bridge -> healthCheck resolves false within ~2 s', async () => {
    const net = require('net');
    const socks = [];
    const srv = net.createServer((s) => { socks.push(s); });   // accepts, never answers
    await new Promise(r => srv.listen(0, '127.0.0.1', r));
    try {
      const io = createMockIO();
      const runtime = createRuntime({ io, bridgeUrl: `http://127.0.0.1:${srv.address().port}`, token: 't' });
      let to;
      const r = await Promise.race([
        runtime.start(),
        new Promise(res => { to = setTimeout(() => res('hung'), 3000); })
      ]);
      clearTimeout(to);
      assert.equal(r, false, 'start() should fail on timeout, got ' + r);
      assert.equal(io.getWrites().length, 0);
    } finally { socks.forEach(s => s.destroy()); await new Promise(r => srv.close(r)); }
  });

  await test('injected clock: manual drive works with a fake nowMs (deadman uses Date.now)', async () => {
    const io = createMockIO();
    const fake = { t: 1_000_000 };
    const bridge = await startStubBridge();
    const runtime = createRuntime({ io, bridgeUrl: bridge.url, token: 't', nowMs: () => fake.t });
    try {
      await runtime.start();
      bridge.queue({ type: 'drive', x: 0, y: 1, turn: 0, speed: 0.5, epoch: 1, id: 1 });
      await runtime.pollNow();
      for (let i = 0; i < 30; i++) await runtime.tickOnce(1 / 60);
      assert.ok(io.getWrites().some(x => Math.hypot(x.vx, x.vy) > 0.01), 'manual drive should move');
    } finally { runtime.stop(); await bridge.close(); }
  });

  console.log(`${passed} passed, ${failed} failed`);
  if (failed) process.exitCode = 1;
})().catch((e) => { console.error(e); process.exitCode = 1; });
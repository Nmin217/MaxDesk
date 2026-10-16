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

  console.log(`${passed} passed, ${failed} failed`);
  if (failed) process.exitCode = 1;
})().catch((e) => { console.error(e); process.exitCode = 1; });
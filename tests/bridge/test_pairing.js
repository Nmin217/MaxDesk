'use strict';
// Pairing (bridge/pairing.js) and the bridge's use of it: node tests/bridge/test_pairing.js
const assert = require('node:assert/strict');
const fs = require('fs'), os = require('os'), path = require('path'), http = require('http');
const { spawn } = require('child_process');
const { createPairing } = require('../../bridge/pairing');

let passed = 0, failed = 0;
async function test(name, fn) { try { await fn(); passed++; console.log('PASS ' + name); } catch (e) { failed++; console.log('FAIL ' + name + ': ' + e.message); } }
const tmp = () => fs.mkdtempSync(path.join(os.tmpdir(), 'pair-'));

(async () => {
  await test('first start makes a random 192-bit key, readable only by its owner, and keeps it across restarts', () => {
    const dir = tmp(), a = createPairing({ dataDir: dir }), b = createPairing({ dataDir: dir });
    assert.ok(a.created && !b.created); assert.equal(a.token, b.token); assert.ok(a.token.length >= 32);
    assert.equal(fs.statSync(path.join(dir, 'bridge-token.txt')).mode & 0o077, 0, 'no access for others');
    assert.notEqual(createPairing({ dataDir: tmp() }).token, a.token, 'different desks, different keys');
  });

  await test('only "Bearer <key>" passes; dev-token and near misses do not', () => {
    const p = createPairing({ dataDir: tmp() });
    assert.ok(p.checkAuth('Bearer ' + p.token));
    for (const bad of ['', 'Bearer dev-token', 'Bearer ' + p.token.slice(0, -1), p.token, 'Bearer ' + p.token + 'x']) assert.ok(!p.checkAuth(bad), bad);
  });

  await test('the right code returns the key; wrong, missing and expired codes do not', () => {
    let ms = 0; const p = createPairing({ dataDir: tmp(), now: () => ms });
    assert.equal(p.pair('123456').status, 410, 'no code yet');
    const { code } = p.newCode();
    assert.equal(p.pair('000000' === code ? '111111' : '000000').status, 401);
    assert.equal(p.pair('12345').status, 401, 'not 6 digits');
    const ok = p.pair(code); assert.equal(ok.status, 200); assert.equal(ok.body.token, p.token);
    ms += 10 * 60 * 1000; assert.equal(p.pair(code).status, 410, 'expired after 10 minutes');
  });

  await test('guessing: 5 wrong in a minute blocks that address; 20 wrong in total switch pairing off', () => {
    let ms = 0; const p = createPairing({ dataDir: tmp(), now: () => ms, randomCode: () => '424242' });
    p.newCode();
    for (let i = 0; i < 5; i++) assert.equal(p.pair('000000', 'a').status, 401);
    assert.equal(p.pair('424242', 'a').status, 429, 'blocked even with the right code');
    assert.equal(p.pair('424242', 'b').status, 200, 'another address still can');
    ms += 61000; assert.equal(p.pair('424242', 'a').status, 200, 'unblocked after a minute');
    for (let i = 0; i < 15; i++) p.pair('000000', 'c' + i);
    assert.equal(p.pair('424242', 'z').status, 403, 'switched off');
  });

  await test('SUMMON_TOKEN overrides the key and writes no file', () => {
    const dir = tmp(), p = createPairing({ dataDir: dir, envToken: 'abc' });
    assert.ok(p.checkAuth('Bearer abc')); assert.ok(!fs.existsSync(path.join(dir, 'bridge-token.txt')));
  });

  // ---- the real bridge, started the way it runs at home (no SUMMON_TOKEN) ----
  const dir = tmp(), PORT = 8793;
  const br = spawn('node', [path.join(__dirname, '../../bridge/bridge_sim.js')], { env: { ...process.env, PORT: String(PORT), MAXDESK_DATA_DIR: dir, SUMMON_TOKEN: '' } });
  let out = ''; br.stdout.on('data', d => { out += d; });
  for (let i = 0; i < 50 && !/enter the code\s+\d{6}/.test(out); i++) await new Promise(r => setTimeout(r, 100));
  const req = (method, p, { key, body } = {}) => new Promise((resolve, reject) => {
    const data = body === undefined ? null : JSON.stringify(body);
    const r = http.request({ host: '127.0.0.1', port: PORT, path: p, method, headers: { 'Content-Type': 'application/json', ...(key ? { Authorization: 'Bearer ' + key } : {}) } }, res => {
      let b = ''; res.on('data', c => { b += c; }); res.on('end', () => { let j = null; try { j = JSON.parse(b); } catch (_) {} resolve({ status: res.statusCode, json: j, text: b }); });
    });
    r.on('error', reject); if (data) r.write(data); r.end();
  });
  try {
    await test('bridge: prints the phone address and a 6-digit code; the old dev-token no longer works', async () => {
      assert.match(out, /Pair a phone \(same Wi-Fi\): open http:\/\/.+ and enter the code\s+\d{6}/);
      const r = await req('GET', '/v1/state', { key: 'dev-token' });
      assert.equal(r.status, 401); assert.equal(r.json.pair_required, true);
    });
    await test('bridge: pairing with the printed code returns the key, which then works', async () => {
      const code = out.match(/enter the code\s+(\d{6})/)[1];
      const p = await req('POST', '/v1/pair', { body: { code } });
      assert.equal(p.status, 200); assert.ok(p.json.token);
      assert.equal(p.json.token, fs.readFileSync(path.join(dir, 'bridge-token.txt'), 'utf8').trim());
      assert.equal((await req('GET', '/v1/state', { key: p.json.token })).status, 200);
    });
    await test('bridge: a paired phone makes a code for the next phone', async () => {
      const key = fs.readFileSync(path.join(dir, 'bridge-token.txt'), 'utf8').trim();
      assert.equal((await req('POST', '/v1/pair/new', {})).status, 401, 'not without the key');
      const n = await req('POST', '/v1/pair/new', { key, body: {} });
      assert.equal(n.status, 200); assert.match(n.json.code, /^\d{6}$/);
      assert.equal((await req('POST', '/v1/pair', { body: { code: n.json.code } })).json.token, key);
    });
    await test('bridge: the page opened on this computer has the key built in (no code needed here)', async () => {
      const r = await req('GET', '/');
      assert.ok(r.text.includes('window.SUMMON_BRIDGE_TOKEN='), 'loopback page carries the key');
    });
  } finally { br.kill(); }

  console.log(`\n${passed} passed, ${failed} failed`);
  process.exitCode = failed ? 1 : 0;
})();

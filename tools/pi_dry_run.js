#!/usr/bin/env node
'use strict';
/**
 * Dry run: the whole desk software, in real time, against pretend motors and a pretend room. For checking that a
 * computer (the Raspberry Pi) is fast enough before any CAN adapter or motor is connected.
 *
 *   node tools/pi_dry_run.js            run until Ctrl+C, report every 10 s
 *   node tools/pi_dry_run.js --seconds 60
 *   node tools/pi_dry_run.js --no-demo      don't drive by itself (use the phone)
 *
 * It starts the bridge (the phone app's server, port 8787), then the Pi runtime with pi_io.js on the fake SPARK bus
 * and the fake room from the tests. Open the address the bridge prints on your phone, pair, and summon: the pretend
 * desk drives through the real stack. Every 10 s it reports how long each 60 Hz control step took (it has 16.7 ms),
 * how late the steps started, CPU use and memory.
 */
const path = require('path');
const { spawn } = require('child_process');
const http = require('http');
const root = path.join(__dirname, '..');
const { createRuntime } = require(path.join(root, 'robot/pi/runtime'));
const { createPiIO, DEFAULT_MODULES } = require(path.join(root, 'robot/pi/pi_io'));
const { createFakeSparkBus } = require(path.join(root, 'robot/pi/fake_spark_bus'));
const { createFakeWorld } = require(path.join(root, 'tests/pi/fake_world'));

const arg = (name, dflt) => { const i = process.argv.indexOf(name); return i > 0 ? process.argv[i + 1] : dflt; };
const SECONDS = +arg('--seconds', 0), PORT = +arg('--port', 8787), HZ = 60, DT = 1 / HZ, DEMO = !process.argv.includes('--no-demo');

function waitForBridge(tries = 100) {
  return new Promise((resolve, reject) => {
    const once = () => http.get({ host: '127.0.0.1', port: PORT, path: '/v1/health' }, r => { r.resume(); resolve(); })
      .on('error', () => (--tries > 0 ? setTimeout(once, 100) : reject(new Error('bridge did not start'))));
    once();
  });
}

(async () => {
  console.log('Starting the bridge on port ' + PORT + '...');
  const bridge = spawn(process.execPath, [path.join(root, 'bridge/bridge_sim.js')], { env: { ...process.env, PORT: String(PORT) }, stdio: ['ignore', 'inherit', 'inherit'] });
  bridge.on('exit', code => { console.log('Bridge stopped (' + code + ').'); process.exit(1); });
  await waitForBridge();

  const bus = createFakeSparkBus();
  const world = createFakeWorld({ bus, modules: DEFAULT_MODULES });
  const io = createPiIO({ bus, lidar: world.lidar, inputs: { estopEngaged: () => false, readCord: () => ({ plugged: false, interlock_on: true }) } });
  // The runtime reads the bridge's pairing key from the same folder; we drive its loop ourselves to time each step.
  const rt = createRuntime({ io, bridgeUrl: 'http://127.0.0.1:' + PORT, autoLoop: false });
  if (!(await rt.start())) { console.error('The runtime could not reach the bridge.'); bridge.kill(); process.exit(1); }
  console.log('\nDry run started: pretend motors, pretend room, the real desk software at ' + HZ + ' Hz.');
  console.log(DEMO ? 'Demo: the desk drives across the room and back by itself, so the timing covers real driving.'
    + ' Pair your phone (code above) to watch or take over. --no-demo to stop that. Ctrl+C to stop.\n'
    : 'Pair your phone with the code above, then Summon. Ctrl+C to stop.\n');
  // Demo: summon across the room every 40 s, return to the dock in between.
  let demoT = 0, demoLeg = 0;
  const demo = () => { if (!DEMO) return; demoT += DT; if (demoT < 20) return; demoT = 0;
    if (demoLeg++ % 2 === 0) { rt.sim.setUser({ x: 700, y: 430, r: 13 }); rt.sim.clickGo(); } else rt.sim.clickBack(); };

  let steps = 0, sum = 0, worst = 0, over = 0, lateWorst = 0, n10 = 0, polling = false, pushing = false;
  let lastPoll = 0, lastPush = 0, cpu0 = process.cpuUsage(), t0 = Date.now(), next = performance.now();
  const startedAt = Date.now();
  async function step() {
    const late = performance.now() - next;
    lateWorst = Math.max(lateWorst, late);
    next += 1000 / HZ;
    const now = Date.now();
    if (!polling && now - lastPoll >= 100) { lastPoll = now; polling = true; rt.pollNow().finally(() => { polling = false; }); }
    if (!pushing && now - lastPush >= 100) { lastPush = now; pushing = true; rt.pushStateNow().finally(() => { pushing = false; }); }
    const a = performance.now();
    world.step(DT);
    demo();
    await rt.tickOnce(DT);
    const ms = performance.now() - a;
    steps++; n10++; sum += ms; worst = Math.max(worst, ms); if (ms > 1000 / HZ) over++;
    if (Date.now() - t0 >= 10000) report();
    if (SECONDS && Date.now() - startedAt >= SECONDS * 1000) return finish();
    setTimeout(step, Math.max(0, next - performance.now()));
  }
  function report() {
    const c = process.cpuUsage(cpu0), wall = (Date.now() - t0) * 1000, cpuPct = 100 * (c.user + c.system) / wall;
    const mem = process.memoryUsage().rss / 1048576, p = world.pose, s = rt.sim.getState();
    console.log(`[${Math.round((Date.now() - startedAt) / 1000)} s] step avg ${(sum / n10).toFixed(1)} ms, worst ${worst.toFixed(1)} ms ` +
      `(budget 16.7), over budget ${over}/${n10}, started late by up to ${lateWorst.toFixed(0)} ms | CPU ${cpuPct.toFixed(0)}% of one core, ` +
      `memory ${mem.toFixed(0)} MB | desk ${s.active ? 'driving' : 'parked'} at (${p.x.toFixed(1)}, ${(-p.y).toFixed(1)}) m: ${rt.sim.getStatusText()}`);
    const warm = Date.now() - startedAt < 12000;
    const verdict = warm ? '(warm-up: the first seconds are always slower)' : worst < 12 && over === 0 ? 'OK: plenty of headroom' : worst < 16.7 && over === 0 ? 'OK, but close to the limit' : (over <= 2 ? 'Mostly OK: ' + over + ' slow step(s)' : 'TOO SLOW: ' + over + ' steps missed the 16.7 ms budget');
    console.log('           ' + verdict);
    sum = 0; worst = 0; over = 0; lateWorst = 0; n10 = 0; cpu0 = process.cpuUsage(); t0 = Date.now();
  }
  function finish() { if (n10) report(); rt.stop(); bridge.removeAllListeners('exit'); bridge.kill(); console.log(`Done: ${steps} steps.`); process.exit(0); }
  process.on('SIGINT', finish);
  step();
})().catch(e => { console.error(e); process.exit(1); });

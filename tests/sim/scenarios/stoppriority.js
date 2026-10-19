{
  const checks = [];
  const record = (check, ok, extra = {}) => {
    checks.push({check, ok: !!ok, ...extra});
  };

  const setInterlock = on => { $('il').checked = !!on; };
  const runSec = sec => {
    const dt = 1 / 60;
    for (let t = 0; t < sec; t += dt) update(dt);
  };
  const runUntil = (condition, maxSec) => {
    const dt = 1 / 60;
    let elapsed = 0;
    while (elapsed < maxSec) {
      update(dt);
      elapsed += dt;
      if (condition()) return elapsed;
    }
    return null;
  };
  const speed = () => Math.hypot(T.vx, T.vy);

  // 1. A plugged cord with interlock enabled must refuse Summon.
  reset();
  $('dm').value = 'manual';
  setInterlock(true);
  if (!cordIn) $('cord').onclick();
  runSec(1);
  $('go').onclick();
  const cordRefuses = active === false;
  record('cord_refuses', cordRefuses);

  // 2. Unplugging allows a fresh Summon.
  $('cord').onclick();
  $('go').onclick();
  runSec(5);
  const summonAfterUnplug = active === true;
  record('summon_after_unplug', summonAfterUnplug);

  // 3. Stop clears active motion.
  $('stop').onclick();
  runSec(0.2);
  const stopClearsMotion = active === false;
  record('stop_clears_motion', stopClearsMotion);

  // 4. A fresh Summon immediately after Stop must be accepted.
  $('go').onclick();
  const freshSummon = active === true;
  record('fresh_summon_after_stop', freshSummon);

  // 5. Move the target approximately 4 m from the desk, then Stop mid-trip.
  user.x = Math.min(W - 25, T.x + 200);
  user.y = T.y;
  $('go').onclick();
  runSec(0.5);
  $('stop').onclick();
  const activeStopped = active === false;
  const elapsed = runUntil(() => speed() < 2, 1);
  record('stop_latency', activeStopped && elapsed !== null, {
    ms: elapsed === null ? null : Math.round(elapsed * 1000)
  });

  // A new command sequence must leave the simulator in a single active
  // navigation state, even when duplicate clicks occur in one tick.
  reset();
  user.x = 700;
  user.y = 430;
  const wasIdle = active === false;
  for (let i = 0; i < 5; i++) $('go').onclick();
  const dupAcceptedOnce = wasIdle && active === true;
  update(1 / 60);
  const dupSummonOnePath = dupAcceptedOnce && active === true &&
    Array.isArray(path);
  record('duplicate_clicks_keep_active_state', dupSummonOnePath);

  for (const check of checks) console.log(JSON.stringify(check));
  const passed = checks.length === 6 && checks.every(c => c.ok);
  console.log(passed ? 'STOP-PRIORITY PASS' : 'STOP-PRIORITY FAIL');
  if (!passed) process.exitCode = 1;
}

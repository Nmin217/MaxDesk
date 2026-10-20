// Stuck detection and recovery regressions:
//   1. Turning in place in open space, or turn-and-go along a zig-zag, is never treated as stuck.
//   2. Driving into a wall IS treated as stuck, and recovery starts.
//   3. Being stuck over and over never cancels the trip (no "giving up").
//   4. Stop and E-stop override a recovery that is in progress.
{
  const checks = [];
  const record = (check, ok, extra = {}) => checks.push({ check, ok: !!ok, ...extra });
  const dt = 1 / 60;
  // Put the desk in open floor (no obstacle within 80 px), facing 0, and start a trip there.
  const startAt = (x, y) => {
    reset(); T.x = x; T.y = y; T.th = 0; T.vx = T.vy = T.w = 0;
    user.x = 700; user.y = 430;
    $('go').onclick();
  };
  // Hand the follower a fixed path and keep the planner from replacing it.
  const forcePath = p => { path = p; goal = { x: p[p.length - 1].x, y: p[p.length - 1].y }; pathI = 0; rp = 1e9; };
  const wp = (x, y, k) => ({ x, y, th: k * DK, k });

  // 1. In-place turn of 90 deg (4 steps of 22.5) in open space, then a short move.
  {
    startAt(400, 430);
    const x = (T.x / CS | 0) * CS + CS / 2, y = (T.y / CS | 0) * CS + CS / 2;
    T.x = x; T.y = y;
    forcePath([wp(x, y, 0), wp(x, y, 1), wp(x, y, 2), wp(x, y, 3), wp(x, y, 4), wp(x, y - 10, 4), wp(x, y - 20, 4)]);
    let maxStuck = 0, sawRecovery = false;
    const th0 = T.th;
    for (let t = 0; t < 3; t += dt) {
      update(dt); rp = 1e9;
      maxStuck = Math.max(maxStuck, stuckTime);
      if (recovery) sawRecovery = true;
    }
    const turnedDeg = Math.abs(ang(T.th, th0)) * 180 / Math.PI;
    record('turn_in_place_not_stuck', !sawRecovery && maxStuck < STUCK_TRIGGER && turnedDeg > 80,
      { turnedDeg: Math.round(turnedDeg), maxStuck: +maxStuck.toFixed(2) });
  }

  // 1b. Turn, short move, turn, short move... Each pull-away starts from rest, and the old detector counted the
  // speed-up time as "stuck", so several turns in a row added up to a false stall.
  {
    startAt(400, 430);
    let x = (T.x / CS | 0) * CS + CS / 2, y = (T.y / CS | 0) * CS + CS / 2;
    T.x = x; T.y = y;
    const p = [wp(x, y, 0)];
    let k = 0;
    for (let leg = 0; leg < 6; leg++) {
      const kNew = leg % 2 ? 0 : 2;                // 45 deg one way, then back
      while (k !== kNew) { k += Math.sign(kNew - k); p.push(wp(x, y, k)); }
      for (let s = 0; s < 3; s++) { x += 10; p.push(wp(x, y, k)); }
    }
    forcePath(p);
    let maxStuck = 0, sawRecovery = false;
    for (let t = 0; t < 30 && path.length && Math.hypot(T.x - x, T.y - y) > 4; t += dt) {
      update(dt); rp = 1e9;
      maxStuck = Math.max(maxStuck, stuckTime);
      if (recovery) sawRecovery = true;
    }
    record('turn_and_go_not_stuck', !sawRecovery && maxStuck < STUCK_TRIGGER && Math.hypot(T.x - x, T.y - y) <= 6,
      { maxStuck: +maxStuck.toFixed(2), at: [Math.round(T.x), Math.round(T.y)], end: [x, y] });
  }

  // 2. A path straight into a block: the supervisor refuses, and that must count as stuck.
  {
    startAt(300, 430);
    rects.push({ x: T.x + HW + 6, y: T.y - 40, w: 30, h: 80 });
    const x = T.x, y = T.y;
    const p = []; for (let i = 0; i <= 12; i++) p.push(wp(x + i * 10, y, 0));
    forcePath(p);
    let recT = null;
    for (let t = 0; t < 6 && recT === null; t += dt) {
      update(dt);
      if (!recovery && path.length) rp = 1e9;
      if (recovery) recT = t;
    }
    record('wall_stall_starts_recovery', recT !== null && active, { afterSec: recT === null ? null : +recT.toFixed(1) });

    // 4a. Stop during that recovery.
    $('stop').onclick();
    let stopT = null;
    for (let t = 0; t < 1 && stopT === null; t += dt) { update(dt); if (Math.hypot(T.vx, T.vy) < 2 && Math.abs(T.w) < 0.1) stopT = t; }
    record('stop_overrides_recovery', !active && recovery === null && stopT !== null,
      { ms: stopT === null ? null : Math.round(stopT * 1000) });
  }

  // 3. Keep feeding the same blocked path for 60 s: many stalls, trip must stay active.
  {
    startAt(300, 430);
    rects.push({ x: T.x + HW + 6, y: T.y - 40, w: 30, h: 80 });
    const x = T.x, y = T.y;
    let recoveries = 0, prev = null, inactive = false, gaveUp = false, cycleResets = 0;
    for (let t = 0; t < 60; t += dt) {
      if (!recovery && !path.length) {
        const p = []; for (let i = 0; i <= 12; i++) p.push(wp(T.x + i * 10, T.y, 0));
        forcePath(p);
      }
      update(dt);
      if (!recovery && path.length) rp = 1e9;
      if (recovery && recovery !== prev) recoveries++;
      prev = recovery;
      if (!active) inactive = true;
      const st = $('st').textContent;
      if (/giving up/i.test(st)) gaveUp = true;
      if (/Recovery cycle reset/.test(st)) cycleResets++;
    }
    record('repeated_stalls_never_cancel', recoveries >= 4 && !inactive && !gaveUp,
      { recoveries, cycleResets: cycleResets > 0 });

    // 4b. E-stop during recovery: wheels stop at once and the trip ends.
    for (let t = 0; t < 10 && !recovery; t += dt) update(dt);
    const hadRecovery = !!recovery;
    toggleEstop();
    update(dt);
    record('estop_overrides_recovery', hadRecovery && !active && recovery === null &&
      Math.hypot(T.vx, T.vy) === 0 && T.w === 0);
    toggleEstop();
  }

  for (const c of checks) console.log(JSON.stringify(c));
  const passed = checks.length === 6 && checks.every(c => c.ok);
  console.log(passed ? 'RECOVERY PASS' : 'RECOVERY FAIL');
  if (!passed) process.exitCode = 1;
}

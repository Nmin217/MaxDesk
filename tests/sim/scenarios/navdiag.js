// Navigation diagnostics: route flip-flops, back-and-forth driving, wheel/yaw twitching, recoveries, give-ups.
//   node tests/sim/run.js navdiag            NAVDIAG_N=40 for more trips, NAVDIAG_V=1 to print every trip,
//                                            NAVDIAG_TRACE=<seed> to print a 0.1 s timeline of that trip
// Prints NAVDIAG PASS when every trip arrives and stays inside the limits in NAV_LIMITS (verify.sh --full checks it).
// A "route switch" is a replan whose new path leaves the old one by more than 30 px at its midpoint (often legitimate:
// the lidar found the old route blocked). A "flip-back" is a switch back onto a route it had already abandoned.
// A "reversal" is the velocity direction swinging more than 120 deg within 1 s while moving.
// A "yaw twitch" is the yaw-rate command changing sign at > 0.3 rad/s within 0.4 s.
const NAV_N = +(process.env.NAVDIAG_N || 24), NAV_V = !!process.env.NAVDIAG_V, NAV_TRACE = +(process.env.NAVDIAG_TRACE || -1);
// Limits per trip set (totals). The room set must be clean; the equal-route set may flip back when the lidar reveals
// that the chosen side of the block is narrower than it looked (both remaining cases are that).
const NAV_LIMITS = { room: { flipBacks: 0, twitch: 12, rec: 2 }, 'equal-routes': { flipBacks: 3, twitch: 8, rec: 2 } };
let navPass = true;
function navCheck(name, tot) {
  const lim = NAV_LIMITS[name], ok = tot.arrived === tot.trips && Object.keys(lim).every(k => tot[k] <= lim[k]);
  if (!ok) { navPass = false; console.log('  over limit', name, JSON.stringify(lim)); }
}
function navMid(p) { return p.length ? p[p.length >> 1] : null; }
function navOff(p, q) { let m = 1e9; for (const r of p) m = Math.min(m, Math.hypot(r.x - q.x, r.y - q.y)); return m; }
const navPlan0 = plan;
let navSwitches = 0, navFlips = 0, navTracing = false, navDropped = [];
plan = function () {
  const old = path.slice();
  navPlan0();
  if (old.length > 4 && path.length > 4 && path !== old) {
    const mid = navMid(path);
    if (navOff(old, mid) > 30) {
      navSwitches++;
      if (navDropped.some(p => navOff(p, mid) < 15)) navFlips++;
      navDropped.push(old); if (navDropped.length > 4) navDropped.shift(); if (navTracing) console.log(`   t=${time.toFixed(2)} ROUTE SWITCH old len ${pathLength(old).toFixed(0)} new len ${pathLength(path).toFixed(0)} squeeze=${squeeze}`); }
  }
};
function navTrip(seed, ux, uy, tmax) {
  __seed = seed; reset(); user.x = ux; user.y = uy; active = true; rp = 0;
  navSwitches = 0; navFlips = 0; navDropped = []; navTracing = seed === NAV_TRACE;
  const dt = 1 / 60;
  let t = 0, rev = 0, twitch = 0, rec = 0, resets = 0, prevRec = null, lastDir = null, lastDirT = -9, lastW = 0, lastWT = -9;
  let arrived = null, distance = 0, px = T.x, py = T.y;
  while (t < tmax) {
    update(dt); t += dt;
    distance += Math.hypot(T.x - px, T.y - py); px = T.x; py = T.y;
    const v = Math.hypot(T.vx, T.vy);
    if (v > 5) {
      const d = Math.atan2(T.vy, T.vx);
      if (lastDir !== null && t - lastDirT < 1 && Math.abs(ang(d, lastDir)) > 2.1) { rev++; lastDir = null; }
      else if (lastDir === null || t - lastDirT > 0.25) { lastDir = d; lastDirT = t; }
    }
    const wc = T.w;
    if (Math.abs(wc) > 0.3) {
      if (Math.sign(wc) !== Math.sign(lastW) && t - lastWT < 0.4) twitch++;
      lastW = wc; lastWT = t;
    }
    if (navTracing && Math.round(t * 60) % 6 === 0) console.log(`   t=${t.toFixed(1)} x=${T.x.toFixed(0)} y=${T.y.toFixed(0)} th=${(T.th * 57.3).toFixed(0)} v=${v.toFixed(1)} w=${T.w.toFixed(2)} rec=${recovery || '-'} stuck=${stuckTime.toFixed(2)} path=${path.length} | ${$('st').textContent}`);
    if (recovery && recovery !== prevRec) rec++;
    prevRec = recovery;
    if (/Recovery cycle reset/.test($('st').textContent)) resets++;
    if (!active && parkTh === null && /^(Arrived|Parking|Ready)/.test($('st').textContent)) { arrived = +t.toFixed(1); break; }
  }
  return { arrived, switches: navSwitches, flipBacks: navFlips, rev, twitch, rec, resets, distance: Math.round(distance), st: $('st').textContent };
}
{
  const tot = { trips: 0, arrived: 0, switches: 0, flipBacks: 0, rev: 0, twitch: 0, rec: 0, resets: 0, time: 0 };
  const bad = [];
  let g = 987654321;   // own generator: the sim's rnd() is consumed differently by different code, so trips would not match
  const grnd = () => (g = (g * 1664525 + 1013904223) % 4294967296) / 4294967296;
  reset();
  for (let i = 0; i < NAV_N; i++) {
    const ux = 60 + grnd() * 680, uy = 60 + grnd() * 380;
    if (hitTest(ux, uy) || clearance(ux, uy) < 30 || Math.hypot(ux - HOME.x, uy - HOME.y) < 150) { i--; continue; }
    const seed = 500 + i, r = navTrip(seed, ux, uy, 90);
    tot.trips++; if (r.arrived !== null) { tot.arrived++; tot.time += r.arrived; }
    for (const k of ['switches', 'flipBacks', 'rev', 'twitch', 'rec', 'resets']) tot[k] += r[k];
    const flag = r.arrived === null || r.flipBacks > 0 || r.rev > 2 || r.twitch > 6 || r.resets > 0;
    if (flag) bad.push({ seed, user: [Math.round(ux), Math.round(uy)], ...r });
    if (NAV_V) console.log('trip', seed, Math.round(ux), Math.round(uy), JSON.stringify(r));
  }
  tot.time = +(tot.time / Math.max(1, tot.arrived)).toFixed(1);
  console.log('NAVDIAG room', JSON.stringify(tot)); navCheck('room', tot);
  for (const b of bad) console.log('  problem', JSON.stringify(b));
}
// Equal-cost routes: one block squarely between the start and the person, so going left or right around it costs
// the same. This is the case that made the desk go back and forth between two routes.
{
  const tot = { trips: 0, arrived: 0, switches: 0, flipBacks: 0, rev: 0, twitch: 0, rec: 0, resets: 0, time: 0 };
  const bad = [];
  const users = [[640, 400], [700, 300], [400, 440], [720, 160], [560, 430]];
  const sizes = [50, 80, 120];
  let seed = 900;
  for (const [ux, uy] of users) for (const sz of sizes) {
    const navReset = reset;
    reset = function () {
      navReset();
      rects.length = 0; circs.length = 0;
      const mx = (HOME.x + ux) / 2, my = (HOME.y + uy) / 2;
      rects.push({ x: mx - sz / 2, y: my - sz / 2, w: sz, h: sz });
    };
    const r = navTrip(seed, ux, uy, 90);
    reset = navReset;
    tot.trips++; if (r.arrived !== null) { tot.arrived++; tot.time += r.arrived; }
    for (const k of ['switches', 'flipBacks', 'rev', 'twitch', 'rec', 'resets']) tot[k] += r[k];
    if (r.arrived === null || r.flipBacks > 0 || r.rev > 2 || r.twitch > 6 || r.resets > 0) bad.push({ seed, user: [ux, uy], block: sz, ...r });
    if (NAV_V) console.log('trip', seed, ux, uy, sz, JSON.stringify(r));
    seed++;
  }
  tot.time = +(tot.time / Math.max(1, tot.arrived)).toFixed(1);
  console.log('NAVDIAG equal-routes', JSON.stringify(tot)); navCheck('equal-routes', tot);
  for (const b of bad) console.log('  problem', JSON.stringify(b));
}
console.log(navPass ? 'NAVDIAG PASS' : 'NAVDIAG FAIL');
if (!navPass) process.exitCode = 1;

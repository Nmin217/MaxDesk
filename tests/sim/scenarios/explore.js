// Frontier exploration on the office floor (run with MAXDESK_ROOM=office). "Learn the room" with an empty map and no
// typed-in tour points: the desk finds the edges of what it has seen and drives to them until none are left.
//   1. It finishes by itself ("Room mapped") and parks back at the dock.
//   2. It has seen at least 95% of the open floor (the rest is behind furniture, out of the LiDAR's sight).
//   3. No contact the whole time.
//   4. The battery ran low on the way: it charged and carried on, keeping what it had learned.
{
  const checks = [];
  const record = (check, ok, extra = {}) => checks.push({ check, ok: !!ok, ...extra });
  const dt = 1 / 60;
  if (ROOM !== 'office') throw new Error('run with MAXDESK_ROOM=office');
  __seed = 2; reset(); forgetMap(); clearMapArrays(); bat.r = 100;
  for (let k = 0; k < 30; k++) update(dt);
  let open = 0;
  for (let i = 0; i < GW * GH; i++) {
    const x = (i % GW + .5) * CS, y = ((i / GW | 0) + .5) * CS;
    if (!rects.some(r => x > r.x && x < r.x + r.w && y > r.y && y < r.y + r.h) && !circs.some(c => Math.hypot(x - c.x, y - c.y) < c.r)) open++;
  }
  $('learn').onclick();
  let t = 0, mapped = false, resumed = 0, wasResume = false, wasHome = false, wasAct = true;
  for (; t < 2400 && (active || mode === 'tour' || resumeExplore); t += dt) {
    update(dt);
    const st = $('st').textContent;
    if (exploreDone && !mapped) { mapped = true; console.log('mapped at', Math.round(t), 's'); }
    if (mode === 'home' && active && !wasHome) console.log('return at', Math.round(t), 's', st); wasHome = mode === 'home' && active;
    if (!active && wasAct) console.log('stopped at', Math.round(t), 's', T.x | 0, T.y | 0, st); wasAct = active;
    if (resumeExplore && !wasResume) resumed++;
    wasResume = resumeExplore;
  }
  for (let k = 0; k < 600 && parkTh !== null; k++) update(dt);
  let seenN = 0; for (let i = 0; i < GW * GH; i++) if (seen[i]) seenN++;
  const cover = Math.min(1, seenN / open);
  record('finishes_by_itself_and_docks', mapped && !active && Math.hypot(T.x - HOME.x, T.y - HOME.y) < 20, { sec: Math.round(t), mapped, active, mode, offDock: Math.round(Math.hypot(T.x - HOME.x, T.y - HOME.y)), resumeExplore, st: $('st').textContent });
  record('sees_95pct_of_open_floor', cover >= .95, { coverage_pct: Math.round(cover * 100), spots: fN });
  record('no_contact', contacts === 0, { contacts });
  record('recharges_and_resumes', resumed >= 1 && trips.some(x => x.outcome === 'battery'), { recharges: resumed });
  for (const c of checks) console.log(JSON.stringify(c));
  const passed = checks.length === 4 && checks.every(c => c.ok);
  console.log(passed ? 'EXPLORE PASS' : 'EXPLORE FAIL');
  if (!passed) process.exitCode = 1;
}

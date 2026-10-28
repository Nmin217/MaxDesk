// Wheel drift and LiDAR position correction (sim "Sensors & drift" panel).
//   1. Drift off: the real and believed positions are the same object (the sim behaves exactly as before).
//   2. Realistic drift, correction OFF: after three round trips it is clearly lost and misses the dock.
//   3. Realistic drift, correction ON, in a room it has never seen: stays within 15 cm and docks every time.
//   4. Realistic drift, correction ON, room learned first: stays within 8 cm and docks every time.
//   5. Bad drift (slippery floor, no gyro), correction ON, room learned: stays within 16 cm and docks.
//   6. A bump (the real desk shoved ~0.4 m and several degrees): back within 8 cm (the dock's tolerance) in under 4 s.
{
  const checks = [];
  const record = (check, ok, extra = {}) => checks.push({ check, ok: !!ok, ...extra });
  const dt = 1 / 60;
  const cm = px => Math.round(px * 2);
  function setMode(drift, loc) { $('drift').value = drift; $('locOff').checked = !loc; }

  // A learned map: one "Learn the room" tour with a perfect position, then left to settle.
  __seed = 3; reset(); clearMapArrays(); setMode('off', true); people.length = 0; bat.r = 100;
  startTour(); for (let t = 0; t < 400 && active; t += dt) update(dt);
  for (let k = 0; k < 600; k++) update(dt);
  const learned = { smap: smap.slice(), seen: seen.slice(), born: mapBorn.map(() => -1e9) };
  const useMap = m => { clearMapArrays(); if (m) { smap.set(m.smap); seen.set(m.seen); mapBorn.set(m.born) } };

  function rounds(n, drift, loc, seed, map) {
    __seed = seed; reset(); useMap(map); setMode(drift, loc);
    people.length = 0; bat.r = 100;
    let maxErr = 0, t = 0, docks = 0;
    const run = (cond, max) => { for (let k = 0; k < max / dt; k++) { update(dt); t += dt; maxErr = Math.max(maxErr, posErr().d); if (cond()) return; } };
    for (let r = 0; r < n; r++) {
      user.x = r % 2 ? 700 : 640; user.y = r % 2 ? 430 : 120;
      $('go').onclick(); run(() => !active, 90);
      startReturn(); run(() => !active, 90); run(() => false, 1);
      if (dockedNow()) docks++;
    }
    return { maxErrCm: cm(maxErr), endErrCm: cm(posErr().d), docks, of: n, sec: Math.round(t) };
  }

  __seed = 5; reset(); setMode('off', true);
  for (let k = 0; k < 60; k++) update(dt);
  record('drift_off_is_exact', G === T && posErr().d === 0);

  const noLoc = rounds(3, 'real', false, 11, learned);
  record('realistic_drift_without_correction_gets_lost', noLoc.maxErrCm > 15 && noLoc.docks < noLoc.of, noLoc);
  const fresh = rounds(3, 'real', true, 11, null);
  record('realistic_drift_corrected_in_an_unknown_room', fresh.maxErrCm <= 15 && fresh.docks === fresh.of, fresh);
  const known = rounds(3, 'real', true, 11, learned);
  record('realistic_drift_corrected_in_a_learned_room', known.maxErrCm <= 8 && known.docks === known.of, known);
  const bad = rounds(3, 'bad', true, 11, learned);
  record('bad_drift_corrected_in_a_learned_room', bad.maxErrCm <= 16 && bad.docks === bad.of, bad);

  __seed = 7; reset(); useMap(learned); setMode('real', true); people.length = 0;
  for (let k = 0; k < 300; k++) update(dt);
  kickDesk();
  const kicked = cm(posErr().d);
  let back = null;
  for (let k = 0; k < 8 / dt; k++) { update(dt); if (posErr().d * 2 <= 8) { back = +(k * dt).toFixed(1); break; } }
  record('recovers_from_a_bump', kicked >= 25 && back !== null && back <= 4, { kickedCm: kicked, backWithin8cmSec: back });

  setMode('off', true); update(dt); clearMapArrays();
  for (const c of checks) console.log(JSON.stringify(c));
  const passed = checks.length === 6 && checks.every(c => c.ok);
  console.log(passed ? 'DRIFT PASS' : 'DRIFT FAIL');
  if (!passed) process.exitCode = 1;
}

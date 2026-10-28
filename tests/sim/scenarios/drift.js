// Wheel drift and LiDAR position correction (sim "Sensors & drift" panel).
//   1. Drift off: the real and believed positions are the same object (the sim behaves exactly as before).
//   2. Realistic drift, correction OFF: after three round trips it is clearly lost and misses the dock.
//   3. Realistic drift, correction ON, in a room it has never seen: stays within 15 cm and docks every time.
//   4. Realistic drift, correction ON, room learned first: stays within 8 cm and docks every time.
//   5. Bad drift (slippery floor, no gyro), correction ON, room learned: stays within 16 cm and docks.
//   6. A bump (the real desk shoved ~0.4 m and several degrees): back within 8 cm (the dock's tolerance) in under 4 s.
//   7. Several bumps at once (up to ~2 m and 20+ deg), idle and mid-trip, 12 cases: it notices it's lost when the
//      error is too big to slide back, holds still without touching the map, and finds itself with a whole-room
//      search (at least 10 of 12), then the interrupted trip still arrives. A spot that looks like another can
//      leave it lost: that's allowed (it stays stopped and asks for help), being confidently wrong is not.
//   8. While lost, a summon is refused with a message.
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

  // 7. repeated bumps
  const cases = []; let lostSeen = 0;
  for (let i = 1; i <= 12; i++) {
    __seed = i * 7; reset(); useMap(learned); setMode('real', true); people.length = 0; bat.r = 100;
    const mid = i % 2 === 0;
    if (mid) { user.x = 700; user.y = 430; $('go').onclick(); for (let k = 0; k < (2 + i % 5) / dt; k++) update(dt); }
    else for (let k = 0; k < 120; k++) update(dt);
    const n = 3 + i % 4; for (let j = 0; j < n; j++) kickDesk(); update(dt);
    const shoved = cm(posErr().d); let wasLost = false, rec = null;
    for (let t = 0; t < 10; t += dt) { update(dt); wasLost = wasLost || locLost; if (rec === null && !locLost && posErr().d * 2 <= 8) rec = +t.toFixed(1); }
    if (wasLost) lostSeen++;
    const stillLost = locLost; let alerted = false;
    if (stillLost) for (let k = 0; k < 1 / dt; k++) { update(dt); alerted = alerted || /can't work out where I am/.test(extraStateJSON().alert_text); }
    if (mid && !stillLost) for (let k = 0; k < 60 / dt && active; k++) update(dt);
    cases.push({ mid, bumps: n, shovedCm: shoved, lost: wasLost, stillLost, alerted, backSec: rec, endCm: cm(posErr().d), arrived: mid && !stillLost ? !active : null });
  }
  const bigOnes = cases.filter(c => c.shovedCm > 60);
  const good = c => c.backSec !== null && c.backSec <= 5 && c.endCm <= 12 && c.arrived !== false;
  const safe = c => good(c) || (c.stillLost && c.alerted);   // stopped and asking for help
  record('recovers_from_repeated_bumps', cases.filter(good).length >= 10 && cases.every(safe)
    && lostSeen >= 3 && bigOnes.every(c => c.lost), { recovered: cases.filter(good).length + '/12', stillLostAndAsking: cases.filter(c => !good(c) && safe(c)).length, lostSeen, worst: cases.reduce((a, c) => c.shovedCm > a.shovedCm ? c : a) , cases: cases.map(c => `${c.mid ? 'trip' : 'idle'} ${c.shovedCm}cm ${c.lost ? 'lost ' : ''}-> ${c.stillLost ? 'STILL LOST' + (c.alerted ? ', asking for help' : ', silent') : c.backSec + 's end ' + c.endCm + 'cm'}`) });

  // 8. summon refused while lost
  __seed = 13; reset(); useMap(learned); setMode('real', true); people.length = 0; bat.r = 100;
  for (let k = 0; k < 120; k++) update(dt);
  for (let j = 0; j < 8; j++) kickDesk();
  let lostNow = false; for (let k = 0; k < 30 && !lostNow; k++) { update(dt); lostNow = locLost; }
  user.x = 700; user.y = 430; $('go').onclick();
  record('summon_refused_while_lost', lostNow && !active && /Lost/.test(extraStateJSON().alert_text), { lostNow, alert: extraStateJSON().alert_text });

  setMode('off', true); update(dt); clearMapArrays();
  for (const c of checks) console.log(JSON.stringify(c));
  const passed = checks.length === 8 && checks.every(c => c.ok);
  console.log(passed ? 'DRIFT PASS' : 'DRIFT FAIL');
  if (!passed) process.exitCode = 1;
}

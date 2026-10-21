// Battery planning: the desk must always be able to get back to the charger.
//   1. Enough battery: a summon is accepted.
//   2. Not enough for (here -> you -> charger) + reserve: the summon is refused with an alert, the desk stays docked.
//   3. Battery draining mid-trip: the desk gives up the trip, heads home on its own and raises an alert.
//   4. It won't come to you again until it has charged enough.
//   5. Each trip is logged with how it ended, its distance and the battery it used.
{
  const checks = [];
  const record = (check, ok, extra = {}) => checks.push({ check, ok: !!ok, ...extra });
  const dt = 1 / 60;
  const run = sec => { for (let t = 0; t < sec; t += dt) update(dt); };
  const until = (cond, max) => { for (let t = 0; t < max; t += dt) { update(dt); if (cond()) return t; } return null; };
  const far = () => { user.x = 700; user.y = 430; };

  // 1. plenty of battery
  __seed = 3; reset(); far(); run(1);
  bat.r = 80; $('go').onclick();
  record('summon_accepted_with_enough_battery', active && mode === 'user', { needTrip: +batteryNeedTrip(user.x, user.y).toFixed(1) });
  $('stop').onclick(); run(0.5);

  // 2. just under what the round trip needs, while docked
  reset(); far(); run(1);
  const need = batteryNeedTrip(user.x, user.y) + BAT_RESERVE;
  bat.r = need - 1; $('go').onclick();
  const ex2 = extraStateJSON();
  record('summon_refused_when_round_trip_wont_fit', !active && /needs/i.test(ex2.alert_text) && ex2.battery_plan.can_summon === false,
    { needPct: +need.toFixed(1), had: +(need - 1).toFixed(1), alert: ex2.alert_text, status: $('st').textContent });

  // 3. drains mid-trip: put the battery just above "enough to get home" partway out, then let it drain
  reset(); far(); run(1);
  bat.r = 80; $('go').onclick();
  until(() => Math.hypot(T.x - HOME.x, T.y - HOME.y) > 250, 40);
  bat.r = batteryNeedHome(T.x, T.y) + BAT_RESERVE + 0.3;
  const turned = until(() => mode === 'home', 15);
  const ex3 = extraStateJSON();
  record('turns_back_mid_trip_when_low', turned !== null && active && /charger/i.test(ex3.alert_text) && ex3.trips[0] && ex3.trips[0].outcome === 'battery',
    { afterSec: turned === null ? null : +turned.toFixed(1), alert: ex3.alert_text, lastTrip: ex3.trips[0] });
  const docked = until(() => !active && dockedNow(), 90);
  record('reaches_charger_after_turning_back', docked !== null && bat.r > 0, { batteryAtDock: +bat.r.toFixed(1) });

  // 4. refuses to come out again until charged enough, then accepts
  far();
  bat.r = Math.min(bat.r, batteryNeedTrip(user.x, user.y) + BAT_RESERVE - 2);
  $('go').onclick();
  const refusedLow = !active;
  until(() => bat.r >= batteryNeedTrip(user.x, user.y) + BAT_RESERVE + 1, 120);
  $('go').onclick();
  record('waits_for_charge_then_accepts', refusedLow && active, { battery: +bat.r.toFixed(1) });
  const arrived = until(() => !active, 120);

  // 5. trip log
  const t5 = extraStateJSON().trips;
  const last = t5[0];
  record('trip_logged_with_outcome_distance_battery', arrived !== null && last && last.kind === 'summon' && last.outcome === 'arrived' && last.distance_m > 2 && last.battery_used_pct > 0,
    { last, kinds: t5.map(t => t.kind + ':' + t.outcome) });

  for (const c of checks) console.log(JSON.stringify(c));
  const passed = checks.length === 6 && checks.every(c => c.ok);
  console.log(passed ? 'BATTERYPLAN PASS' : 'BATTERYPLAN FAIL');
  if (!passed) process.exitCode = 1;
}

// Boxed-in watchdog: a trip that can't make progress gives up after BOXED_T seconds and tells the user,
// instead of waiting silently forever.
//   1-3. A block over the whole desk, half of it, or one corner: the trip ends as 'blocked' with an alert, no contact.
//   4.   A person who stands in the way for 8 s and then walks off: the trip still arrives (no false give-up).
{
  const checks = [];
  const record = (check, ok, extra = {}) => checks.push({ check, ok: !!ok, ...extra });
  const dt = 1 / 60;
  const blocks = [['whole_desk', { x: HOME.x - 40, y: HOME.y - 30, w: 80, h: 60 }],
                  ['half_desk', { x: HOME.x, y: HOME.y - 30, w: 60, h: 60 }],
                  ['one_corner', { x: HOME.x + 18, y: HOME.y + 8, w: 20, h: 20 }]];
  for (const [name, b] of blocks) {
    __seed = 5; reset(); clearMapArrays(); people.length = 0; bat.r = 90;
    rects.push({ ...b, _custom: true, z0: 0, z1: 1.2 });
    for (let k = 0; k < 60; k++) update(dt);
    user.x = 700; user.y = 430; $('go').onclick();
    let t = 0; for (; t < 40 && active; t += dt) update(dt);
    const out = trips[0] && trips[0].outcome, alert = extraStateJSON().alert_text;
    record('boxed_' + name + '_gives_up_and_says_so', !active && out === 'blocked' && /boxed in/i.test(alert) && t < BOXED_T + 6,
      { outcome: out, sec: Math.round(t), alert });
  }
  // A person in the way for a while must not trigger it.
  __seed = 5; reset(); clearMapArrays(); people.length = 0; bat.r = 90;
  for (let k = 0; k < 60; k++) update(dt);
  user.x = 700; user.y = 430; $('go').onclick();
  const wall = { x: HOME.x + 45, y: HOME.y - 120, w: 20, h: 240, _custom: true, z0: 0, z1: 1.2 };
  rects.push(wall);
  let t = 0; for (; t < 8; t += dt) update(dt);
  rects.splice(rects.indexOf(wall), 1);
  for (; t < 90 && active; t += dt) update(dt);
  record('temporary_blocker_still_arrives', trips[0] && trips[0].outcome === 'arrived', { outcome: trips[0] && trips[0].outcome, sec: Math.round(t) });
  for (const c of checks) console.log(JSON.stringify(c));
  const passed = checks.length === 4 && checks.every(c => c.ok);
  console.log(passed ? 'BOXED PASS' : 'BOXED FAIL');
  if (!passed) process.exitCode = 1;
}

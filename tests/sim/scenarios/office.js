// Office floor (run with MAXDESK_ROOM=office): 32 x 20 m, offices off a corridor, kitchen, break area, lounge, people.
//   1-5. Summons into an office, the kitchen, the lounge, the meeting room and the far office all arrive, no contact.
//   6.   Walls are walls: no person ends up inside furniture or a wall.
{
  const checks = [];
  const record = (check, ok, extra = {}) => checks.push({ check, ok: !!ok, ...extra });
  const dt = 1 / 60;
  if (ROOM !== 'office') throw new Error('run with MAXDESK_ROOM=office');
  for (const [name, ux, uy] of [['office_2', 480, 170], ['kitchen', 300, 770], ['lounge', 1380, 800], ['meeting_room', 1430, 230], ['office_4', 1100, 170]]) {
    __seed = 3; reset(); clearMapArrays(); bat.r = 95;
    for (let k = 0; k < 30; k++) update(dt);
    user.x = ux; user.y = uy; $('go').onclick();
    let t = 0; for (; t < 240 && active; t += dt) update(dt);
    record('summon_' + name, trips[0] && trips[0].outcome === 'arrived' && contacts === 0, { sec: Math.round(t), contacts, outcome: trips[0] && trips[0].outcome });
  }
  const inside = people.filter(p => rects.some(r => p.x > r.x && p.x < r.x + r.w && p.y > r.y && p.y < r.y + r.h));
  record('people_never_inside_walls_or_furniture', inside.length === 0, { inside: inside.length });
  for (const c of checks) console.log(JSON.stringify(c));
  const passed = checks.length === 6 && checks.every(c => c.ok);
  console.log(passed ? 'OFFICE PASS' : 'OFFICE FAIL');
  if (!passed) process.exitCode = 1;
}

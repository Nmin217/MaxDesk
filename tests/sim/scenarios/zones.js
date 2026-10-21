// Keep-out zones: the desk plans around them and never drives its footprint into one.
//   1. A zone set across the direct route: the desk still arrives, and its footprint never overlaps the zone.
//   2. Without the zone the same trip goes straight through that area (so test 1 proves the zone was respected).
//   3. Bad zone input is rejected or clipped, and zones round-trip through zonesJSON() in metres.
{
  const checks = [];
  const record = (check, ok, extra = {}) => checks.push({ check, ok: !!ok, ...extra });
  const dt = 1 / 60;
  const ZONE = { name: 'Desk', x: 7.0, y: 3.8, w: 2.0, h: 1.6 };   // metres; sits on the straight line from the dock to (14, 8.6)
  const zr = { x: ZONE.x / M_PER_PX, y: ZONE.y / M_PER_PX, w: ZONE.w / M_PER_PX, h: ZONE.h / M_PER_PX };
  const footprintInZone = () => satOverlap(deskPoly(), [[zr.x, zr.y], [zr.x + zr.w, zr.y], [zr.x + zr.w, zr.y + zr.h], [zr.x, zr.y + zr.h]]);
  function trip(withZone) {
    __seed = 21; reset(); setZonesM(withZone ? [ZONE] : []);
    people.length = 0; user.x = 700; user.y = 430; bat.r = 90;
    $('go').onclick();
    let entered = 0, inside = false, t = 0;
    for (; t < 90; t += dt) {
      update(dt);
      const f = footprintInZone(); if (f && !inside) entered++; inside = f;
      if (!active && parkTh === null) break;
    }
    return { arrived: !active && /Ready|Arrived|Parking/.test($('st').textContent), entered, sec: +t.toFixed(1) };
  }
  const withZ = trip(true), noZ = trip(false);
  record('arrives_around_zone_without_entering', withZ.arrived && withZ.entered === 0, withZ);
  record('same_trip_without_zone_crosses_that_area', noZ.entered > 0, noZ);

  reset();
  setZonesM([{ name: 'x'.repeat(40), x: 15.5, y: 9.5, w: 3, h: 3 }, { x: NaN, y: 1, w: 1, h: 1 }, { x: 1, y: 1, w: 0.01, h: 2 }, ZONE]);
  const zj = zonesJSON();
  record('validates_and_round_trips_metres', zj.length === 2 && zj[0].name.length === 24 && zj[0].x + zj[0].w <= 16.0001 && zj[1].x === 7 && zj[1].w === 2, { zones: zj });
  setZonesM([]);

  for (const c of checks) console.log(JSON.stringify(c));
  const passed = checks.length === 3 && checks.every(c => c.ok);
  console.log(passed ? 'ZONES PASS' : 'ZONES FAIL');
  if (!passed) process.exitCode = 1;
}

// 2D vs 3D LiDAR in the "LiDAR test room": a 12 cm box and an 80-90 cm overhang on the straight route.
//   1. 2D LiDAR at 25 cm sees neither: the desk drives into both.
//   2. 3D LiDAR sees both: it goes around and arrives with no contact.
//   3. 2D LiDAR raised to 85 cm sees the overhang but not the low box: it still hits the box.
//   4. 2D at 25 cm, desk lowered to 70 cm: it passes UNDER the overhang (no contact there), still hits the box.
//   5. Every built-in scene obstacle is visible to the default 2D LiDAR (25 cm), so other scenarios are unaffected.
{
  const checks = [];
  const record = (check, ok, extra = {}) => checks.push({ check, ok: !!ok, ...extra });
  const dt = 1 / 60;
  function trip(type, heightCm, deskCm) {
    __seed = 4; buildLidarTest(); clearMapArrays();
    $('lidarType').value = type; $('lidarH').value = String(heightCm); $('drift').value = 'off';
    if (deskCm) { deskH = userH = deskCm / 100; }
    $('go').onclick();
    const hit = new Set(); let t = 0;
    for (; t < 90 && active; t += dt) {
      update(dt);
      const d = deskPoly(G);
      for (const r of rects) if (objZ0(r) < deskH && satOverlap(d, [[r.x, r.y], [r.x + r.w, r.y], [r.x + r.w, r.y + r.h], [r.x, r.y + r.h]])) hit.add(objZ1(r) < .3 ? 'low box' : 'overhang');
    }
    return { arrived: !active && /Arrived|Ready|Parking/.test($('st').textContent), hit: [...hit], sec: Math.round(t) };
  }
  const flat = trip('2d', 25);
  record('2d_at_25cm_misses_low_box_and_overhang', flat.hit.includes('low box') && flat.hit.includes('overhang'), flat);
  const full = trip('3d', 25);
  record('3d_goes_around_both', full.arrived && full.hit.length === 0, full);
  const high = trip('2d', 85);
  record('2d_at_85cm_sees_overhang_not_low_box', high.hit.includes('low box') && !high.hit.includes('overhang'), high);
  const lowDesk = trip('2d', 25, 70);
  record('desk_lowered_to_70cm_fits_under_the_overhang', !lowDesk.hit.includes('overhang') && lowDesk.hit.includes('low box'), lowDesk);

  reset(); $('lidarType').value = '2d'; $('lidarH').value = '25';
  const all = [...rects, ...circs].every(o => lidarSees(objZ0(o), objZ1(o)));
  record('default_scene_fully_visible_to_default_2d', all);
  $('lidarType').value = '1'; $('lidarH').value = '1';   // back to the headless defaults
  for (const c of checks) console.log(JSON.stringify(c));
  const passed = checks.length === 5 && checks.every(c => c.ok);
  console.log(passed ? 'LIDAR PASS' : 'LIDAR FAIL');
  if (!passed) process.exitCode = 1;
}

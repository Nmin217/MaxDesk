'use strict';

const assert = require('node:assert/strict');
const S = require('../../robot/pi/swerve');

let passed = 0, failed = 0;
function test(name, fn) {
  try { fn(); console.log(`PASS ${name}`); passed++; }
  catch (e) { console.log(`FAIL ${name}: ${e.message}`); failed++; }
}
const near = (a, b, tol = 1e-9, msg = '') => assert.ok(Math.abs(a - b) <= tol, `${msg} expected ${b}, got ${a}`);
const nearAngle = (a, b, tol = 1e-9, msg = '') => near(S.wrapAngle(a - b), 0, tol, msg);
const PI = Math.PI;
const kin = S.createKinematics();
const mods = kin.modules;

test('constants: MAXSwerve 13T reduction, wheel free speed, default rectangle', () => {
  near(S.DRIVE_REDUCTION, 990 / 195);
  near(S.WHEEL_FREE_SPEED, (5676 / 60) * PI * 0.0762 / (990 / 195));
  assert.ok(S.WHEEL_FREE_SPEED > 4.4 && S.WHEEL_FREE_SPEED < 4.5);
  assert.deepEqual(mods, [{ x: 0.2, y: 0.3 }, { x: 0.2, y: -0.3 }, { x: -0.2, y: 0.3 }, { x: -0.2, y: -0.3 }]);
});

test('wrapAngle maps to (-pi, pi]', () => {
  near(S.wrapAngle(PI), PI); near(S.wrapAngle(-PI), PI); near(S.wrapAngle(3 * PI), PI);
  near(S.wrapAngle(2 * PI), 0); near(S.wrapAngle(-PI / 2 - 4 * PI), -PI / 2, 1e-12);
  for (let i = 0; i < 1000; i++) { const a = (Math.random() - 0.5) * 40; const w = S.wrapAngle(a);
    assert.ok(w > -PI && w <= PI); near(Math.sin(w), Math.sin(a), 1e-9); near(Math.cos(w), Math.cos(a), 1e-9); }
});

test('pure forward: all wheels angle 0, equal speed', () => {
  for (const s of S.toModuleStates(1.2, 0, 0)) { near(s.angle, 0); near(s.speed, 1.2); }
});

test('pure strafe left: all wheels angle +pi/2', () => {
  for (const s of kin.toModuleStates(0, 0.5, 0)) { near(s.angle, PI / 2); near(s.speed, 0.5); }
});

test('pure rotation: wheels tangent, speed r*omega', () => {
  const w = 1.5, r = Math.hypot(0.2, 0.3);
  const st = kin.toModuleStates(0, 0, w);
  st.forEach((s, i) => {
    const p = mods[i];
    nearAngle(s.angle, Math.atan2(p.x, -p.y), 1e-12, `module ${i} angle`);   // (-y, x) direction for CCW
    near(s.speed, r * w, 1e-12);
  });
  near(st[0].angle, Math.atan2(0.2, -0.3));   // FL points back-left-ish: ~146.3 deg
  assert.ok(st[0].angle > PI / 2);
});

test('desaturation keeps ratios and caps the max', () => {
  const raw = kin.toModuleStates(3, 1, 4);
  const cap = kin.toModuleStates(3, 1, 4, { maxSpeed: 2 });
  const top = Math.max(...raw.map(s => s.speed));
  assert.ok(top > 2);
  near(Math.max(...cap.map(s => s.speed)), 2, 1e-12);
  cap.forEach((s, i) => { near(s.speed / raw[i].speed, 2 / top, 1e-12); near(s.angle, raw[i].angle); });
  // chassis motion direction preserved: same direction and same vx:vy:omega ratio
  const c = kin.toChassisSpeeds(cap);
  near(c.vx / 3, c.vy / 1, 1e-9); near(c.omega / 4, c.vx / 3, 1e-9);
  // below the cap: untouched
  assert.deepEqual(kin.toModuleStates(0.5, 0, 0, { maxSpeed: 2 }), kin.toModuleStates(0.5, 0, 0));
});

test('optimize: flips when error > 90 deg, not otherwise', () => {
  let o = S.optimize({ speed: 1, angle: 0 }, PI / 3);              // 60 deg error
  near(o.speed, 1); near(o.angle, 0);
  o = S.optimize({ speed: 1, angle: 0 }, PI / 2);                  // exactly 90: no flip
  near(o.speed, 1); near(o.angle, 0);
  o = S.optimize({ speed: 1, angle: 0 }, 2 * PI / 3);              // 120 deg: flip
  near(o.speed, -1); near(o.angle, PI);
  o = S.optimize({ speed: 0.7, angle: PI / 4 }, -PI / 2 - 0.2);    // 135+ deg: flip
  near(o.speed, -0.7); near(o.angle, -3 * PI / 4);
});

test('optimize: wrap-around near +-pi', () => {
  let o = S.optimize({ speed: 1, angle: PI - 0.1 }, -PI + 0.1);    // only 0.2 rad apart across pi
  near(o.speed, 1); near(o.angle, PI - 0.1);
  o = S.optimize({ speed: 1, angle: 0.05 }, PI - 0.05);            // ~170 deg: flip to ~pi + 0.05 -> -pi + 0.05
  near(o.speed, -1); near(o.angle, -PI + 0.05);
  o = S.optimize({ speed: 1, angle: -PI + 0.01 }, 0);              // flip to 0.01
  near(o.speed, -1); near(o.angle, 0.01, 1e-12);
  for (let i = 0; i < 500; i++) {   // result always within 90 deg of current, same wheel velocity vector
    const st = { speed: Math.random() * 2, angle: (Math.random() - 0.5) * 2 * PI }, cur = (Math.random() - 0.5) * 2 * PI;
    const r = S.optimize(st, cur);
    assert.ok(Math.abs(S.wrapAngle(r.angle - cur)) <= PI / 2 + 1e-12);
    assert.ok(r.angle > -PI && r.angle <= PI);
    near(r.speed * Math.cos(r.angle), st.speed * Math.cos(st.angle), 1e-12);
    near(r.speed * Math.sin(r.angle), st.speed * Math.sin(st.angle), 1e-12);
  }
});

test('cosineScale', () => {
  near(S.cosineScale(2, 0), 2); near(S.cosineScale(2, PI / 3), 1, 1e-12);
  near(S.cosineScale(2, PI / 2), 0, 1e-12); near(S.cosineScale(2, -PI / 3), 1, 1e-12);
  near(S.cosineScale(2, 2 * PI / 3), 0);   // never reverses
});

test('zero command keeps last angles (and 0 without them)', () => {
  const cur = [0.3, -1.2, PI, 2];
  kin.toModuleStates(0, 0, 0, { currentAngles: cur }).forEach((s, i) => { near(s.speed, 0); near(s.angle, cur[i]); });
  kin.toModuleStates(5e-5, -5e-5, 5e-5, { currentAngles: cur, maxSpeed: 1 }).forEach((s, i) => near(s.angle, cur[i]));
  kin.toModuleStates(0, 0, 0).forEach(s => { near(s.speed, 0); near(s.angle, 0); });
  assert.ok(kin.toModuleStates(1e-3, 0, 0, { currentAngles: cur }).every(s => Math.abs(s.angle) < 1e-12));
});

test('forward(inverse(v)) round trip within 1e-9', () => {
  const k2 = S.createKinematics([{ x: 0.31, y: 0.22 }, { x: 0.29, y: -0.25 }, { x: -0.33, y: 0.2 }, { x: -0.27, y: -0.24 }]);
  for (const k of [kin, k2]) for (let i = 0; i < 1000; i++) {
    const v = { vx: (Math.random() - 0.5) * 6, vy: (Math.random() - 0.5) * 6, omega: (Math.random() - 0.5) * 8 };
    const c = k.toChassisSpeeds(k.toModuleStates(v.vx, v.vy, v.omega));
    near(c.vx, v.vx); near(c.vy, v.vy); near(c.omega, v.omega);
  }
  // optimized (flipped) states give the same chassis speeds
  const st = kin.toModuleStates(0.4, -0.3, 1).map(s => S.optimize(s, s.angle + 2.5));
  const c = kin.toChassisSpeeds(st);
  near(c.vx, 0.4); near(c.vy, -0.3); near(c.omega, 1);
});

test('frame helpers: fieldToRobot / robotToField', () => {
  let r = S.fieldToRobot(1, 0, PI / 2);   // robot facing field +y; field +x is robot right (-y)
  near(r.vx, 0, 1e-12); near(r.vy, -1);
  r = S.robotToField(1, 0, PI / 2); near(r.vx, 0, 1e-12); near(r.vy, 1);
  for (let i = 0; i < 100; i++) {
    const h = (Math.random() - 0.5) * 10, a = Math.random(), b = Math.random();
    const f = S.robotToField(...Object.values(S.fieldToRobot(a, b, h)), h);
    near(f.vx, a, 1e-12); near(f.vy, b, 1e-12);
  }
});

test('IO frame: theta_io = 0 means the desk faces IO +y (up on the plan)', () => {
  near(S.ioThetaToHeading(0), PI / 2);
  const r = S.fieldToRobot(0, 1, S.ioThetaToHeading(0));   // IO "up" -> robot forward
  near(r.vx, 1); near(r.vy, 0, 1e-12);
  const l = S.fieldToRobot(-1, 0, S.ioThetaToHeading(0));  // IO left -> robot left (+y)
  near(l.vx, 0, 1e-12); near(l.vy, 1);
  for (let i = 0; i < 100; i++) { const t = (Math.random() - 0.5) * 2 * PI; nearAngle(S.headingToIoTheta(S.ioThetaToHeading(t)), t, 1e-12); }
});

test('offsets round trip, REV template values', () => {
  assert.deepEqual(S.ANGULAR_OFFSETS, [-PI / 2, 0, PI, PI / 2]);
  near(S.applyOffset(0, 0), -PI / 2); near(S.applyOffset(0, 2), PI); near(S.removeOffset(PI / 2, 3), 0);
  for (let i = 0; i < 400; i++) {
    const m = i % 4, a = (Math.random() - 0.5) * 2 * PI;
    nearAngle(S.removeOffset(S.applyOffset(a, m), m), a, 1e-12);
    const e = S.applyOffset(a, m); assert.ok(e > -PI && e <= PI);
  }
});

// ---- odometry helpers: simulated drive encoders + gyro ----
function sim(odo) {
  const dist = [0, 0, 0, 0];
  const st = { dist, yaw: 0, angles: [0, 0, 0, 0] };
  // Drive a robot-frame twist for t seconds in n steps; wheels follow inverse kinematics.
  st.drive = (vx, vy, w, t, n = 200) => {
    const dt = t / n;
    for (let k = 0; k < n; k++) {
      const s = kin.toModuleStates(vx, vy, w, { currentAngles: st.angles });
      s.forEach((m, i) => { dist[i] += m.speed * dt; st.angles[i] = m.angle; });
      st.yaw += w * dt;
      odo.update(dist, st.angles, st.yaw);
    }
  };
  return st;
}

test('odometry: drive straight 1 m with gyro 0 -> x = 1', () => {
  const odo = new S.Odometry(kin);
  sim(odo).drive(0.5, 0, 0, 2);
  const p = odo.getPose(); near(p.x, 1, 1e-9); near(p.y, 0, 1e-12); near(p.theta, 0);
});

test('odometry: strafe uses heading (theta = pi/2 start, strafe left -> field -x)', () => {
  const odo = new S.Odometry(kin, { pose: { x: 1, y: 2, theta: PI / 2 } });
  sim(odo).drive(0, 0.25, 0, 2);
  const p = odo.getPose(); near(p.x, 0.5, 1e-9); near(p.y, 2, 1e-9); near(p.theta, PI / 2);
});

test('odometry: rotation in place leaves x, y ~ 0', () => {
  const odo = new S.Odometry(kin);
  sim(odo).drive(0, 0, 1, 2 * PI);
  const p = odo.getPose(); near(p.x, 0, 1e-9); near(p.y, 0, 1e-9); nearAngle(p.theta, 0, 1e-9);
});

test('odometry: 1 m square with 90 deg turns returns to start', () => {
  const odo = new S.Odometry(kin);
  const s = sim(odo);
  const corners = [];
  for (let side = 0; side < 4; side++) {
    s.drive(0.5, 0, 0, 2);
    corners.push(odo.getPose());
    s.drive(0, 0, PI / 4, 2);   // +90 deg in place
  }
  near(corners[0].x, 1, 1e-9); near(corners[0].y, 0, 1e-9);
  near(corners[1].x, 1, 1e-9); near(corners[1].y, 1, 1e-9);
  near(corners[2].x, 0, 1e-9); near(corners[2].y, 1, 1e-9);
  const p = odo.getPose(); near(p.x, 0, 1e-9); near(p.y, 0, 1e-9); nearAngle(p.theta, 0, 1e-9);
});

test('odometry: arc (drive + turn together) matches the exact circle', () => {
  const odo = new S.Odometry(kin);
  sim(odo).drive(1, 0, PI / 2, 1, 7);   // quarter circle, radius 2/pi, coarse steps: arc integration is exact
  const R = 2 / PI, p = odo.getPose();
  near(p.x, R, 1e-9); near(p.y, R, 1e-9); near(p.theta, PI / 2, 1e-12);
});

test('odometry: resetPose rebases distances and gyro', () => {
  const odo = new S.Odometry(kin);
  const s = sim(odo);
  s.drive(0.5, 0, 0.3, 2);
  odo.resetPose({ x: 5, y: -1, theta: 0 });
  near(odo.getPose().x, 5);
  s.drive(0.5, 0, 0, 2);   // gyro still reads 0.6 rad, but heading was reset to 0
  const p = odo.getPose(); near(p.x, 6, 1e-9); near(p.y, -1, 1e-9); near(p.theta, 0, 1e-12);
});

console.log(`${passed} passed, ${failed} failed`);
if (failed) process.exitCode = 1;

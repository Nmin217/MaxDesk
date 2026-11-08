'use strict';
/**
 * A pretend room for end-to-end tests of the Pi stack (runtime -> brain in hardware mode -> pi_io -> SPARK bus).
 * The "true" desk moves by what the fake SPARKs ACTUALLY did (with wheel slip the encoders don't see), and two
 * fake LiDARs ray-cast from the true position. The software under test never sees the true position: only
 * encoder counts and LiDAR rays, as on the real desk.
 *
 * Frame: IO field (io.js), metres, x right, y up. The simulator's test room maps to x 0..16, y -10..0
 * (sim px * 0.02, y flipped). Desk frame: +x forward, +y left.
 */
const S = require('../../robot/pi/swerve');

const PX = 0.02;
const pxRect = (x, y, w, h) => ({ x0: x * PX, x1: (x + w) * PX, y0: -(y + h) * PX, y1: -y * PX });
// The simulator's test room (its RECTS0 and CIRCS0), so the same layout the scenarios use.
const ROOM = { w: 16, h: 10, rects: [pxRect(220, 170, 40, 150), pxRect(450, 60, 150, 40), pxRect(520, 300, 120, 70)], circs: [{ x: 400 * PX, y: -260 * PX, r: 18 * PX }] };
const HALF_FWD = 0.36, HALF_LEFT = 0.56;                 // the desk's outline
const SENSORS = [[0.30, 0.50], [-0.30, -0.50]];          // two LiDARs on opposite corners (desk frame)

function createFakeWorld({ bus, modules, room = ROOM, start = { x: 85 * PX, y: -95 * PX, heading: Math.PI / 2 },
  slip = { scale: 0.02, yawPerM: 0.01 }, rays = 180, maxRange = 6, scanHz = 10, noise = 0.01, seed = 1 } = {}) {
  const kin = S.createKinematics(modules.map(m => ({ x: m.x, y: m.y })));
  const pose = { ...start };
  let t = 0, scanT = -1, seq = 0, scan = null, contacts = 0, inContact = false, scansOn = true;
  let rnd = seed; const rand = () => { rnd = (rnd * 16807) % 2147483647; return rnd / 2147483647; };

  function rayHit(ox, oy, dx, dy) {   // distance to the first wall/box along a field ray, or Infinity
    let best = Infinity;
    const slab = (x0, x1, y0, y1) => {
      let tn = -Infinity, tf = Infinity;
      for (const [o, d, a, b] of [[ox, dx, x0, x1], [oy, dy, y0, y1]]) {
        if (Math.abs(d) < 1e-12) { if (o < a || o > b) return; continue; }
        let t1 = (a - o) / d, t2 = (b - o) / d; if (t1 > t2) [t1, t2] = [t2, t1];
        tn = Math.max(tn, t1); tf = Math.min(tf, t2);
      }
      if (tn <= tf && tf > 0) best = Math.min(best, tn > 0 ? tn : tf);
    };
    for (const r of room.rects) slab(r.x0, r.x1, r.y0, r.y1);
    for (const c of room.circs) {
      const fx = ox - c.x, fy = oy - c.y, b = fx * dx + fy * dy, q = fx * fx + fy * fy - c.r * c.r, disc = b * b - q;
      if (disc >= 0) { const s = -b - Math.sqrt(disc); if (s > 0) best = Math.min(best, s); }
    }
    for (const [o, d, lim] of [[ox, dx, 0], [ox, dx, room.w], [oy, dy, -room.h], [oy, dy, 0]]) {   // room walls
      if (Math.abs(d) > 1e-12) { const s = (lim - o) / d; if (s > 0) best = Math.min(best, s); }
    }
    return best;
  }

  function makeScan() {
    const c = Math.cos(pose.heading), s = Math.sin(pose.heading), out = [];
    for (const [sx, sy] of SENSORS) {
      const ox = pose.x + c * sx - s * sy, oy = pose.y + s * sx + c * sy;
      for (let i = 0; i < rays; i++) {
        const a = i / rays * 2 * Math.PI, ldx = Math.cos(a), ldy = Math.sin(a);            // desk-frame direction
        const fdx = c * ldx - s * ldy, fdy = s * ldx + c * ldy;
        const d = rayHit(ox, oy, fdx, fdy), hit = d <= maxRange, r = (hit ? d : maxRange) + (hit ? (rand() - .5) * 2 * noise : 0);
        out.push([sx, sy, sx + ldx * r, sy + ldy * r, hit]);
      }
    }
    return out;
  }

  function touching() {   // the true desk outline overlapping furniture (sampled along its edges)
    const c = Math.cos(pose.heading), s = Math.sin(pose.heading);
    for (let k = 0; k <= 20; k++) for (const [u, v] of [[-1, 2 * k / 20 - 1], [1, 2 * k / 20 - 1], [2 * k / 20 - 1, -1], [2 * k / 20 - 1, 1]]) {
      const lx = u * HALF_FWD, ly = v * HALF_LEFT, x = pose.x + c * lx - s * ly, y = pose.y + s * lx + c * ly;
      if (room.rects.some(r => x > r.x0 && x < r.x1 && y > r.y0 && y < r.y1) || room.circs.some(q => Math.hypot(x - q.x, y - q.y) < q.r)) return true;
    }
    return false;
  }

  return {
    step(dt) {
      bus.step(dt); t += dt;
      // What the wheels really did (truth from the fake controllers), then slip the encoders never see.
      const states = modules.map(m => { const tr = bus.truth(m.turnId), dr = bus.truth(m.driveId);
        return { speed: dr.rpm * S.DRIVE_VELOCITY_FACTOR, angle: S.wrapAngle(tr.angle - (m.angleOffset || 0)) }; });
      const v = kin.toChassisSpeeds(states), k = 1 + slip.scale, dist = Math.hypot(v.vx, v.vy) * dt;
      const f = S.robotToField(v.vx * k * dt, v.vy * k * dt, pose.heading);
      pose.x += f.vx; pose.y += f.vy; pose.heading = S.wrapAngle(pose.heading + v.omega * dt + slip.yawPerM * dist);
      const now = touching(); if (now && !inContact) contacts++; inContact = now;
      if (scansOn && t - scanT >= 1 / scanHz - 1e-9) { scanT = t; seq++; scan = makeScan(); }
    },
    lidar: { read: () => (scan ? { rays: scan, ageMs: (t - scanT) * 1000, seq } : null) },
    stopScans() { scansOn = false; },   // the LiDAR "unplugged": no new scans, the last one ages
    get pose() { return { ...pose }; },
    get contacts() { return contacts; },
    get time() { return t; }
  };
}

module.exports = { createFakeWorld, ROOM, PX };

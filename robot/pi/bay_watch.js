'use strict';

/**
 * Bay watch: is someone sitting at the desk, or is there just a chair pushed in?
 *
 * The desk's frame is open on the side you sit at (no rail there), so a chair can roll in. That side is the desk's
 * BACK in the software's frame (the desk's forward is the back of the monitor; +x forward, +y left, metres). The low
 * LiDARs see into that "bay" at their height (~25 cm): an empty office chair shows up as ONE small round thing (the
 * gas column, ~5-8 cm); a person shows up as legs (two blobs, often wider, often with the chair column too).
 *
 *   person  -> the desk must not move at all.
 *   chair   -> it may move, but only straight away from the chair (desk-frame +x), so the chair slides out of the
 *              open side instead of being pushed by the rails. runtime.js does that "back away" before the trip.
 *   clear   -> nothing in the bay.
 *
 * When unsure it says "person": a missed chair costs a summon, a missed person could drag someone's legs.
 * A person is declared on the first scan that looks like one and only cleared after PERSON_CLEAR_MS without one.
 */

const PERSON_CLEAR_MS = 2000;     // legs swing in and out of the scan plane; don't flicker to "clear"
const CHAIR_CLEAR_MS = 300;
const JOIN_M = 0.05;              // hits closer than this belong to the same object
const THIN_M = 0.05;              // chair legs, rods: at most this wide
const MEDIUM_M = 0.09;            // a chair's gas column is at most this wide (a lower leg is often wider)

function createBayWatch({
  deskHalfFwd = 0.36, deskHalfLeft = 0.56,
  bayDepth = 0.46,                // from the open edge in to the cross brace / belly pan
  bayHalfWidth = 0.50,            // half the clear width between the leg rails
  rearCheck = 0.6                 // how far past the desk's forward edge must be clear to back away
} = {}) {
  const bay = { x0: -deskHalfFwd - 0.03, x1: -deskHalfFwd + bayDepth, y: bayHalfWidth };
  let state = 'clear', lastPersonMs = -Infinity, lastChairMs = -Infinity, last = { blobs: [], rearClear: true };

  // Group hit points into objects: sort by angle round the bay centre, then join neighbours closer than JOIN_M.
  function blobsOf(points) {
    if (!points.length) return [];
    const cx = (bay.x0 + bay.x1) / 2;
    const pts = points.slice().sort((a, b) => Math.atan2(a[1], a[0] - cx) - Math.atan2(b[1], b[0] - cx));
    const groups = [];
    for (const p of pts) {
      const g = groups.find(gr => gr.some(q => Math.hypot(q[0] - p[0], q[1] - p[1]) < JOIN_M));
      if (g) g.push(p); else groups.push([p]);
    }
    // merge groups that ended up touching
    for (let i = 0; i < groups.length; i++) for (let j = i + 1; j < groups.length; j++) {
      if (groups[i].some(p => groups[j].some(q => Math.hypot(q[0] - p[0], q[1] - p[1]) < JOIN_M))) {
        groups[i] = groups[i].concat(groups[j]); groups.splice(j, 1); j = i;
      }
    }
    return groups.map(g => {
      let w = 0;
      for (const a of g) for (const b of g) w = Math.max(w, Math.hypot(a[0] - b[0], a[1] - b[1]));
      const x = g.reduce((s, p) => s + p[0], 0) / g.length, y = g.reduce((s, p) => s + p[1], 0) / g.length;
      return { x, y, width: w, n: g.length };
    });
  }

  function classify(blobs) {
    if (!blobs.length) return 'clear';
    const medium = blobs.filter(b => b.width > THIN_M && b.width <= MEDIUM_M);
    const large = blobs.filter(b => b.width > MEDIUM_M);
    if (large.length) return 'person';           // a lower leg, or legs together
    if (medium.length >= 2) return 'person';     // column + a leg, or two legs
    return 'chair';                              // one column and/or thin chair legs
  }

  return {
    /** rays: [[ox, oy, ex, ey, hit], ...] in the desk frame (m). nowMs: a clock. Returns the current verdict. */
    step(rays, nowMs) {
      const inBay = [], rear = [];
      for (const r of rays || []) {
        if (!r || !r[4] || !Number.isFinite(r[2]) || !Number.isFinite(r[3])) continue;
        const x = r[2], y = r[3];
        if (x >= bay.x0 && x <= bay.x1 && Math.abs(y) <= bay.y) inBay.push([x, y]);
        if (x > deskHalfFwd && x <= deskHalfFwd + rearCheck && Math.abs(y) <= deskHalfLeft + 0.05) rear.push([x, y]);
      }
      const blobs = blobsOf(inBay);
      const seen = classify(blobs);
      if (seen === 'person') lastPersonMs = nowMs;
      if (seen === 'chair') lastChairMs = nowMs;
      state = nowMs - lastPersonMs < PERSON_CLEAR_MS ? 'person'
        : seen === 'chair' || nowMs - lastChairMs < CHAIR_CLEAR_MS ? 'chair' : 'clear';
      last = { blobs, rearClear: rear.length === 0 };
      return this.verdict;
    },
    get verdict() { return { state, blobs: last.blobs, rearClear: last.rearClear }; },
    bay
  };
}

module.exports = { createBayWatch, PERSON_CLEAR_MS };

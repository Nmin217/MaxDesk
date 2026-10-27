'use strict';

/**
 * Last checks between the planner and the motors. Both are plain functions of their inputs, so
 * they are tested without hardware (tests/pi/test_motion_guard.js).
 *
 * Slew limiter: the simulator core already ramps speed (0.5 m/s^2 up, 1.2 down, 2.4 in an
 * emergency). This is a backstop so nothing upstream (a state jump, a bug) can hand the motors a
 * step change: sudden starts and reversals are what bend motor keys and strip gears. Slowing down
 * toward a stop is never delayed, so braking stays as hard as the planner asks.
 *
 * Jam detector: a module whose wheel is told to move but barely turns while its drive draws high
 * current, or whose steering stays far from its target while drawing high current, is jammed or
 * pushing against something. Either way the desk must stop instead of cooking the motor.
 */

const SLEW = { accel: 1.5, alphaMax: 6.0 };   // m/s^2 and rad/s^2: well above the sim's own ramps

function createSlewLimiter({ accel = SLEW.accel, alphaMax = SLEW.alphaMax } = {}) {
  let last = { vx: 0, vy: 0, omega: 0 };
  return {
    /** Next command to send, given what the planner wants this tick. */
    step(vx, vy, omega, dt) {
      const out = {};
      const oldSp = Math.hypot(last.vx, last.vy), newSp = Math.hypot(vx, vy);
      const dot = vx * last.vx + vy * last.vy;
      if (newSp <= oldSp && (newSp < 1e-3 || dot > 0)) {   // slowing (same general direction): never delay
        out.vx = vx; out.vy = vy;
      } else {
        const dx = vx - last.vx, dy = vy - last.vy, d = Math.hypot(dx, dy), lim = accel * dt;
        const k = d > lim ? lim / d : 1;
        out.vx = last.vx + dx * k; out.vy = last.vy + dy * k;
      }
      if (Math.abs(omega) <= Math.abs(last.omega) && omega * last.omega >= 0) out.omega = omega;
      else {
        const lim = alphaMax * dt;
        out.omega = last.omega + Math.max(-lim, Math.min(lim, omega - last.omega));
      }
      last = out;
      return out;
    },
    /** The wheels were stopped some other way (stopAll): start the next ramp from rest. */
    reset() { last = { vx: 0, vy: 0, omega: 0 }; }
  };
}

const JAM = {
  driveCmdMps: 0.10,     // only judge a wheel that is asked to move at least this fast
  driveRatio: 0.25,      // ...and turns at less than this fraction of it
  driveAmps: 20,         // ...while drawing more than this (the robot limit is 40 A)
  steerErrDeg: 30,       // steering this far from its target
  steerAmps: 12,         // ...while the NEO 550 draws more than this (limit 20 A)
  seconds: 0.5           // for this long
};

/**
 * modules: [{ name, driveCmdMps, driveMps, driveAmps, steerErrDeg, steerAmps }] from io.readModules().
 * Returns null, or { name, kind: 'drive' | 'steer', text } once a module has been jammed for JAM.seconds.
 */
function createJamDetector(limits = {}) {
  const L = { ...JAM, ...limits };
  const since = new Map();
  return {
    step(modules, dt) {
      for (const m of modules || []) {
        const cmd = Math.abs(+m.driveCmdMps || 0);
        const driveJam = cmd >= L.driveCmdMps && Math.abs(+m.driveMps || 0) < cmd * L.driveRatio
          && +m.driveAmps > L.driveAmps;
        const steerJam = Math.abs(+m.steerErrDeg || 0) > L.steerErrDeg && +m.steerAmps > L.steerAmps;
        for (const [kind, jammed] of [['drive', driveJam], ['steer', steerJam]]) {
          const key = m.name + ':' + kind;
          const t = jammed ? (since.get(key) || 0) + dt : 0;
          since.set(key, t);
          if (t >= L.seconds) {
            since.clear();
            const what = kind === 'drive' ? 'wheel can\'t turn' : 'steering can\'t turn';
            return { name: m.name, kind, text: `${m.name} ${what}: jammed or pushing against something. Stopped. Check the module.` };
          }
        }
      }
      return null;
    },
    reset() { since.clear(); }
  };
}

module.exports = { createSlewLimiter, createJamDetector, SLEW, JAM };

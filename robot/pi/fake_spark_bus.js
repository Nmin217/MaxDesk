'use strict';

/**
 * A pretend CAN bus full of SPARK MAXes, for testing pi_io.js and the runtime without hardware.
 * It implements the same small "SPARK bus" interface a real backend must (see pi_io.js), and behaves
 * like the real controllers in the ways that catch mistakes:
 *   - drive motors take ~0.1 s to reach a new speed; they report speed (rpm) and cumulative rotations
 *   - steering moves to its target the short way round at a realistic rate; it reports the absolute encoder
 *   - the enable heartbeat: no heartbeat(true) for 100 ms and every SPARK stops driving (as the firmware does)
 *   - a SPARK that was never configured ignores setpoints
 * Faults to switch on per CAN ID: unplugged (goes silent), encoderUnplugged, jammed, backwards (wired or
 * configured the wrong way round), and the battery voltage.
 * Time only moves when step(dt) is called, so tests are exact and repeatable.
 */

const HEARTBEAT_TIMEOUT_S = 0.1;
const DRIVE_TAU_S = 0.08;            // drive speed lag (first order)
const DRIVE_MAX_RPM = 5676;          // NEO free speed
const TURN_MAX_RAD_S = 9;            // module steering rate limit
const TURN_GAIN = 25;                // 1/s: how quickly steering closes the remaining error

const wrapPi = a => a - 2 * Math.PI * Math.floor((a + Math.PI) / (2 * Math.PI));

function createFakeSparkBus({ ids = [], busVolts = 12.6 } = {}) {
  let t = 0, enabledUntil = -1, volts = busVolts;
  const dev = new Map();
  const mk = id => ({ id, cfg: null, kind: null, rpm: 0, rot: 0, angle: 0, target: null, current: 0, temp: 30,
    lastHeard: 0, faults: { unplugged: false, encoderUnplugged: false, jammed: false, backwards: false } });
  for (const id of ids) dev.set(id, mk(id));
  const get = id => { if (!dev.has(id)) dev.set(id, mk(id)); return dev.get(id); };
  const log = [];   // every frame "sent", for tests that check IDs and units

  return {
    // ---- the SPARK bus interface (what pi_io.js uses) ----
    configure(id, cfg) { const d = get(id); if (d.faults.unplugged) return false; d.cfg = { ...cfg }; d.kind = cfg.kind; log.push({ t, id, op: 'configure', cfg: { ...cfg } }); return true; },
    setVelocity(id, rpm) { const d = get(id); log.push({ t, id, op: 'velocity', value: rpm }); if (!d.faults.unplugged && d.cfg) d.target = Math.max(-DRIVE_MAX_RPM, Math.min(DRIVE_MAX_RPM, +rpm || 0)); },
    setPosition(id, rotations) { const d = get(id); log.push({ t, id, op: 'position', value: rotations }); if (!d.faults.unplugged && d.cfg) d.target = +rotations || 0; },
    heartbeat(enabled) { if (enabled) enabledUntil = t + HEARTBEAT_TIMEOUT_S; else enabledUntil = -1; },
    status(id) {
      const d = dev.get(id);
      if (!d || d.lastHeard === 0 && d.faults.unplugged) return null;
      const absRot = d.kind === 'turn' ? (d.faults.encoderUnplugged ? NaN : ((d.angle / (2 * Math.PI)) % 1 + 1) % 1) : undefined;
      return { ageMs: (t - d.lastHeard) * 1000, velocityRpm: d.rpm, positionRot: d.rot, absRot, currentA: d.current,
        tempC: d.temp, busV: volts, faults: d.cfg ? [] : ['not configured'] };
    },
    // ---- test controls ----
    step(dt) {
      t += dt;
      const enabled = t <= enabledUntil;
      for (const d of dev.values()) {
        if (d.faults.unplugged) continue;
        d.lastHeard = t;   // status frames keep arriving while it is plugged in (about every 20 ms on the real thing)
        if (d.kind === 'drive') {
          let want = enabled && d.target !== null ? d.target : 0;
          if (d.faults.backwards) want = -want;
          const before = d.rpm;
          if (d.faults.jammed) d.rpm = 0; else d.rpm += (want - d.rpm) * Math.min(1, dt / DRIVE_TAU_S);
          d.rot += d.rpm / 60 * dt;
          d.current = d.faults.jammed && enabled && Math.abs(want) > 50 ? 35 : 1.5 + Math.abs(d.rpm - before) / dt * 0.004 + Math.abs(d.rpm) * 0.002;
        } else if (d.kind === 'turn') {
          if (enabled && d.target !== null && !d.faults.jammed) {
            const err = wrapPi(d.target * 2 * Math.PI - d.angle);
            d.angle = wrapPi(d.angle + Math.max(-TURN_MAX_RAD_S * dt, Math.min(TURN_MAX_RAD_S * dt, err * TURN_GAIN * dt)));
            d.current = 1 + Math.min(10, Math.abs(err) * 6);
          } else d.current = d.faults.jammed && enabled ? 15 : 0.5;
        }
        d.temp += (d.current * d.current * 0.0004 - (d.temp - 30) * 0.002) * dt * 60;
      }
    },
    setFault(id, name, on = true) { get(id).faults[name] = !!on; if (name === 'unplugged' && on) get(id).lastHeard = get(id).lastHeard; },
    setBusVolts(v) { volts = v; },
    setAngle(id, rad) { get(id).angle = wrapPi(rad); },   // where a module happens to point at power-on
    truth(id) { const d = dev.get(id); return d ? { rpm: d.rpm, rot: d.rot, angle: d.angle, enabled: t <= enabledUntil } : null; },
    get time() { return t; },
    get log() { return log; }
  };
}

module.exports = { createFakeSparkBus, HEARTBEAT_TIMEOUT_S };

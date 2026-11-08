'use strict';

/**
 * Real-hardware IO for the Pi (implements io.js). Turns the brain's "move like this" into each MAXSwerve
 * module's steering angle and wheel speed on the right CAN IDs, in the right units, and reads back what the
 * modules actually did: odometry (for the brain's position), per-module readings (hardware check, jam and
 * wrong-direction detection), battery.
 *
 * It talks to the controllers only through a "SPARK bus", so the way the Pi reaches CAN can be chosen later
 * without touching anything here. A bus must provide:
 *   configure(id, cfg)        cfg: { kind: 'drive'|'turn', currentLimitA, idleMode: 'brake', inverted? }
 *   setVelocity(id, rpm)      drive: motor rpm, run by the SPARK's own velocity loop
 *   setPosition(id, rot)      turn: absolute-encoder rotations 0..1, the SPARK's own position loop (wrapping on)
 *   heartbeat(enabled)        call every tick; SPARKs stop driving if "enabled" stops arriving for 100 ms
 *   status(id)                null if never heard, else { ageMs, velocityRpm, positionRot, absRot, currentA,
 *                              tempC, busV, faults: [] }  (absRot: turn controllers only, NaN if no encoder)
 * Backends: fake_spark_bus.js (tests). Real ones (sparkcan helper, or our own SocketCAN driver) come later;
 * see docs/OPTION-B-PI-CAN.md.
 *
 * Zeroing: set each module's absolute-encoder zero offset (REV Hardware Client) with the wheel pointing at the
 * desk's FRONT. Then encoder angle 0 = desk forward and angleOffset stays 0. A module zeroed some other way
 * (e.g. REV's calibration tool on a module mounted sideways) sets angleOffset = encoder angle when pointing forward.
 */

const S = require('./swerve');

const DEFAULT_MODULES = [   // CAN IDs as wired (odd = drive NEO, even = turn NEO 550); positions are swerve.js placeholders: MEASURE
  { name: 'FL', driveId: 1, turnId: 2 }, { name: 'FR', driveId: 3, turnId: 4 },
  { name: 'RL', driveId: 5, turnId: 6 }, { name: 'RR', driveId: 7, turnId: 8 }
].map((m, i) => ({ ...m, ...S.rectModules()[i], angleOffset: 0 }));

const STALE_MS = 250;   // a controller silent this long counts as missing

function createPiIO({
  bus, modules = DEFAULT_MODULES, maxSpeed = 1.0, imu = null, lidar = null, inputs = null,
  battery = { emptyV: 11.5, fullV: 12.7 }, driveCurrentA = 40, turnCurrentA = 20
} = {}) {
  if (!bus) throw new Error('pi_io needs a SPARK bus');
  const mods = modules.map(m => ({ angleOffset: 0, ...m }));
  const kin = S.createKinematics(mods.map(m => ({ x: m.x, y: m.y })));
  const st = mods.map(() => ({ driveCmdMps: 0, targetAngle: null }));
  for (const m of mods) {
    bus.configure(m.driveId, { kind: 'drive', currentLimitA: driveCurrentA, idleMode: 'brake' });
    bus.configure(m.turnId, { kind: 'turn', currentLimitA: turnCurrentA, idleMode: 'brake', wrapping: true });
  }
  const yaw = () => { if (!imu) return null; try { const v = imu.yaw(); return Number.isFinite(v) ? v : null; } catch (_) { return null; } };
  let odo = null;

  function read(m, i) {   // one module's controllers, in robot units
    const d = bus.status(m.driveId), t = bus.status(m.turnId);
    const present = !!d && !!t && d.ageMs < STALE_MS && t.ageMs < STALE_MS;
    const enc = t && Number.isFinite(t.absRot) ? S.wrapAngle(t.absRot * 2 * Math.PI - m.angleOffset) : NaN;
    return { d, t, present, angle: enc, driveM: d ? d.positionRot * S.DRIVE_POSITION_FACTOR : NaN, driveMps: d ? d.velocityRpm * S.DRIVE_VELOCITY_FACTOR : NaN };
  }

  return {
    isHardware: true,   // the runtime switches the brain to hardware input when it sees this

    // vx, vy, omega: IO field frame (io.js). thetaIo: the heading the brain believes (LiDAR-corrected), IO convention.
    writeWheels(vx, vy, omega, thetaIo) {
      const heading = Number.isFinite(thetaIo) ? S.ioThetaToHeading(thetaIo) : (odo ? odo.getPose().theta : Math.PI / 2);
      const r = S.fieldToRobot(vx, vy, heading);
      const now = mods.map(read);
      const states = kin.toModuleStates(r.vx, r.vy, omega, { maxSpeed, currentAngles: now.map((n, i) => Number.isFinite(n.angle) ? n.angle : (st[i].targetAngle || 0)) });
      bus.heartbeat(true);
      mods.forEach((m, i) => {
        const cur = Number.isFinite(now[i].angle) ? now[i].angle : 0;
        const o = S.optimize(states[i], cur);
        const speed = S.cosineScale(o.speed, o.angle - cur);
        st[i].driveCmdMps = speed; st[i].targetAngle = o.angle;
        const encAngle = S.wrapAngle(o.angle + m.angleOffset);
        bus.setPosition(m.turnId, ((encAngle / (2 * Math.PI)) % 1 + 1) % 1);
        bus.setVelocity(m.driveId, speed / S.DRIVE_VELOCITY_FACTOR);
      });
    },

    stopAll() {
      for (const m of mods) bus.setVelocity(m.driveId, 0);
      bus.heartbeat(false);   // disabled: the SPARKs stop driving (brake) whatever else happens
      st.forEach(s => { s.driveCmdMps = 0; });
    },

    readModules() {
      return mods.map((m, i) => {
        const n = read(m, i), steerErr = st[i].targetAngle === null || !Number.isFinite(n.angle) ? 0 : S.wrapAngle(st[i].targetAngle - n.angle);
        const volts = [n.d, n.t].filter(Boolean).map(x => x.busV).filter(Number.isFinite);
        return {
          name: m.name, present: n.present, encoderOk: Number.isFinite(n.angle),
          angleDeg: n.angle * 180 / Math.PI, driveCmdMps: st[i].driveCmdMps, driveMps: n.driveMps,
          driveAmps: n.d ? n.d.currentA : NaN, steerErrDeg: steerErr * 180 / Math.PI, steerAmps: n.t ? n.t.currentA : NaN,
          driveTempC: n.d ? n.d.tempC : NaN, turnTempC: n.t ? n.t.tempC : NaN,
          busVolts: volts.length ? Math.min(...volts) : NaN,
          faults: [...new Set([...(n.d ? n.d.faults : []), ...(n.t ? n.t.faults : [])])]
        };
      });
    },

    // Odometry, IO field frame. Only moves while every module reads properly: with one missing, the hardware
    // check has already stopped the desk, and half the wheels would give a wrong answer.
    readPose() {
      const now = mods.map(read);
      if (!odo) { odo = new S.Odometry(kin, { gyroYaw: yaw() ?? 0, distances: now.map(n => n.driveM || 0), pose: { x: 0, y: 0, theta: Math.PI / 2 } }); }
      else if (now.every(n => n.present && Number.isFinite(n.angle) && Number.isFinite(n.driveM))) odo.update(now.map(n => n.driveM), now.map(n => n.angle), yaw());
      const p = odo.getPose();
      return { x: p.x, y: p.y, theta: S.headingToIoTheta(p.theta) };
    },

    readVelocity() {
      const now = mods.map(read);
      if (!now.every(n => Number.isFinite(n.angle) && Number.isFinite(n.driveMps))) return { vx: 0, vy: 0, omega: 0 };
      const c = kin.toChassisSpeeds(now.map(n => ({ speed: n.driveMps, angle: n.angle })));
      const f = S.robotToField(c.vx, c.vy, odo ? odo.getPose().theta : Math.PI / 2);
      return { vx: f.vx, vy: f.vy, omega: c.omega };
    },

    readBattery() {
      const v = mods.flatMap(m => [bus.status(m.driveId), bus.status(m.turnId)]).filter(x => x && x.ageMs < STALE_MS).map(x => x.busV).filter(Number.isFinite);
      if (!v.length) return { robot_pct: null, ups_pct: null, charging: null };
      const pct = (Math.min(...v) - battery.emptyV) / (battery.fullV - battery.emptyV) * 100;
      return { robot_pct: Math.max(0, Math.min(100, pct)), ups_pct: null, charging: null };
    },

    // { rays: [[ox,oy,ex,ey,hit],...] desk frame m, ageMs, seq } from the LiDAR driver; throws if there is none.
    readLidar() { if (!lidar) throw new Error('no LiDAR configured'); return lidar.read(); },

    // Not wired = not safe: without inputs, the E-stop reads as engaged and the cord as locked.
    estopEngaged() { if (!inputs) throw new Error('no E-stop input configured'); return !!inputs.estopEngaged(); },
    readCord() { if (!inputs) throw new Error('no cord input configured'); return inputs.readCord(); },

    modules: mods.map(m => ({ ...m }))
  };
}

module.exports = { createPiIO, DEFAULT_MODULES, STALE_MS };

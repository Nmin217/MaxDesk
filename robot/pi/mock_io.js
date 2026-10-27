'use strict';

/**
 * Deterministic mock IO for tests. No timers, no async.
 * writeWheels updates the internal velocity; tick(dt) integrates position.
 */
function createMockIO() {
  const state = {
    x: 0, y: 0, theta: 0,
    vx: 0, vy: 0, omega: 0,
    estop: false,
    cord: { plugged: false, interlock_on: true },
    battery: { robot_pct: 80, ups_pct: 70, charging: false },
    lidar: [],
    modules: null
  };
  const writes = [];
  const stops = [];

  return {
    readPose: () => ({ x: state.x, y: state.y, theta: state.theta }),
    readVelocity: () => ({ vx: state.vx, vy: state.vy, omega: state.omega }),
    readLidar: () => state.lidar.slice(),
    readBattery: () => ({ ...state.battery }),
    readCord: () => ({ ...state.cord }),
    estopEngaged: () => state.estop,
    readModules: () => state.modules && state.modules.map(m => ({ ...m })),

    writeWheels: (vx, vy, omega) => {
      writes.push({ vx, vy, omega });
      state.vx = vx; state.vy = vy; state.omega = omega;
    },
    stopAll: () => {
      stops.push(Date.now());
      state.vx = 0; state.vy = 0; state.omega = 0;
    },

    tick: (dt) => {
      state.x += state.vx * dt;
      state.y += state.vy * dt;
      state.theta += state.omega * dt;
    },
    setEstop: (v) => { state.estop = !!v; },
    setModules: (mods) => { state.modules = mods; },
    setLidar: (pts) => { state.lidar = Array.isArray(pts) ? pts.slice() : []; },
    setCord: (plugged, interlock_on = true) => {
      state.cord = { plugged: !!plugged, interlock_on: !!interlock_on };
    },
    setBattery: (robot_pct, ups_pct, charging = false) => {
      state.battery = { robot_pct, ups_pct, charging };
    },

    getWrites: () => writes.slice(),
    getStops: () => stops.slice(),
    isStopped: () => state.vx === 0 && state.vy === 0 && state.omega === 0
  };
}

module.exports = { createMockIO };
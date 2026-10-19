'use strict';

/**
 * Null IO. Safe default for an unconfigured runtime: refuses all motion.
 * estopEngaged() always returns true, so the runtime calls stopAll() every
 * tick and never calls writeWheels().
 */
function createNullIO() {
  let writes = 0;
  let stops = 0;
  return {
    readPose: () => ({ x: 0, y: 0, theta: 0 }),
    readVelocity: () => ({ vx: 0, vy: 0, omega: 0 }),
    writeWheels: () => { writes++; },
    readLidar: () => [],
    readBattery: () => ({ robot_pct: 0, ups_pct: 0, charging: false }),
    readCord: () => ({ plugged: true, interlock_on: true }),
    estopEngaged: () => true,
    stopAll: () => { stops++; },
    getWrites: () => writes,
    getStops: () => stops
  };
}

module.exports = { createNullIO };
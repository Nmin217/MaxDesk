'use strict';

/**
 * Robot IO interface. The runtime drives the robot only through this.
 *
 * writeWheels is called at the plan rate (~60 Hz) and must not block.
 * stopAll must be callable from any state and must reach a hardware-backed
 * stop (contactor or brake) where the hardware allows it. Neither method
 * may throw under normal conditions.
 *
 * All coordinates are field-relative, in meters and radians.
 *
 * Implementations:
 *   null_io.js   safe default: refuses all motion
 *   mock_io.js   deterministic in-memory mock for tests
 *   pi_io.js     real hardware (WP7, not yet written)
 */
module.exports = {
  readPose()      { throw new Error('readPose not implemented'); },
  readVelocity()  { throw new Error('readVelocity not implemented'); },
  writeWheels(vx, vy, omega) { throw new Error('writeWheels not implemented'); },
  readLidar()     { throw new Error('readLidar not implemented'); },
  readBattery()   { throw new Error('readBattery not implemented'); },
  readCord()      { throw new Error('readCord not implemented'); },
  estopEngaged()  { throw new Error('estopEngaged not implemented'); },
  stopAll()       { throw new Error('stopAll not implemented'); }
};
'use strict';

/**
 * Robot IO interface. The runtime drives the robot only through this.
 *
 * writeWheels is called at the plan rate (~60 Hz) and must not block.
 * stopAll must be callable from any state and must reach a hardware-backed
 * stop (contactor or brake) where the hardware allows it. Neither method
 * may throw under normal conditions.
 *
 * Frame: field-relative, right-handed, meters and radians.
 *   +x  to the right on the floor plan (sim screen +x)
 *   +y  up on the floor plan (sim screen -y; the simulator is y-down)
 *   theta / omega  counter-clockwise positive, seen from above
 * writeWheels(vx, vy, omega, theta): field-frame m/s and rad/s. The runtime converts from simulator px/s
 *   (y-down, clockwise theta) and clamps to |v| <= 1.0 m/s, |omega| <= 2.0 rad/s before calling. theta is the
 *   heading the brain believes (LiDAR-corrected, IO convention); pi_io.js uses it to turn the field command into
 *   the desk's own frame.
 * readCord(): { plugged: bool, interlock_on: bool }. plugged && interlock_on means the wheels must
 *   not move; the runtime calls stopAll() instead of writeWheels(). If it throws, the runtime
 *   treats the cord as locked (fail closed).
 * estopEngaged(): bool. If it throws, the runtime treats the e-stop as engaged.
 * readModules() (optional): [{ name, driveCmdMps, driveMps, driveAmps, steerErrDeg, steerAmps }],
 *   one per swerve module: commanded and measured wheel speed, drive current, how far the steering
 *   is from its target, steering current. The runtime uses it to stop on a jammed module
 *   (motion_guard.js). Without it there is no jam check.
 *   Optional extra fields per module, for the hardware check (hardware_check.js): present (both SPARKs
 *   answering on CAN), encoderOk, angleDeg (absolute steering angle), busVolts, faults (array of names).
 *   Returning null means "this IO doesn't report modules" (no checks). Throwing means "it should but
 *   can't": the hardware check blocks all motion (fail closed).
 *
 * Hardware input (an IO with isHardware: true, i.e. pi_io.js): the runtime also feeds the brain real sensors:
 *   readPose() each tick (odometry; only the change is used), readLidar() -> { rays: [[ox,oy,ex,ey,hit],...] in the
 *   desk frame (m, +x forward, +y left), ageMs, seq }, readBattery() once a second. A scan older than 0.3 s holds
 *   the wheels; older than 2 s is a hardware fault.
 *
 * Implementations:
 *   null_io.js   safe default: refuses all motion
 *   mock_io.js   deterministic in-memory mock for tests
 *   pi_io.js     real hardware: MAXSwerve modules through a SPARK bus (fake_spark_bus.js for tests)
 */
module.exports = {
  readPose()      { throw new Error('readPose not implemented'); },
  readVelocity()  { throw new Error('readVelocity not implemented'); },
  writeWheels(vx, vy, omega) { throw new Error('writeWheels not implemented'); },
  readLidar()     { throw new Error('readLidar not implemented'); },
  readBattery()   { throw new Error('readBattery not implemented'); },
  readCord()      { throw new Error('readCord not implemented'); },
  estopEngaged()  { throw new Error('estopEngaged not implemented'); },
  readModules()   { throw new Error('readModules not implemented'); },
  stopAll()       { throw new Error('stopAll not implemented'); }
};
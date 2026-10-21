'use strict';

/**
 * Swerve kinematics + odometry for the 4-module REV MAXSwerve base. Pure, no dependencies, no IO.
 * Meant for pi_io.js (real hardware, not yet written); the runtime itself never calls this.
 *
 * Robot frame (WPILib): +x forward, +y left, omega CCW+ seen from above, meters / radians.
 * Module order everywhere: 0 FL, 1 FR, 2 RL, 3 RR. Module angles here are robot-frame angles
 * (0 = wheel rolling toward +x), wrapped to (-pi, pi]. Convert to/from the MAXSwerve absolute
 * encoder with applyOffset / removeOffset.
 *
 * ---- Mapping to the runtime's IO field frame (io.js) ----
 * io.js field: +x right on the plan, +y up on the plan, theta CCW+. The desk's forward is the back of
 * the monitor, which the simulator draws as local -y: at sim th = 0 it points up on screen, i.e. IO +y.
 * Sim th is clockwise on a y-down screen, so theta_io = -th_sim, and the desk's forward direction in
 * the IO field is (-sin theta_io, cos theta_io): the robot's WPILib heading (direction of robot +x)
 * is heading = theta_io + pi/2. Robot +y (left) is then IO -x at theta_io = 0, as it should be.
 * pi_io.js should therefore:
 *   writeWheels(vx, vy, omega)   (IO field frame, m/s, rad/s)
 *     const h = odom.getPose().theta        // WPILib heading (= ioThetaToHeading(theta_io))
 *     const r = fieldToRobot(vx, vy, h)      // IO field axes = odometry field axes, so no other change
 *     states = toModuleStates(r.vx, r.vy, omega, { maxSpeed, currentAngles })  // omega: both CCW+, no sign change
 *     each: optimize(state, measuredAngle) -> cosineScale -> applyOffset(angle, i) -> SPARK MAX setpoints
 *   readPose(): keep Odometry in the IO field axes (x right, y up) with theta = WPILib heading; report
 *     { x, y, theta: headingToIoTheta(theta) }. Reset it with resetPose({x, y, theta: ioThetaToHeading(theta_io)}).
 *   readVelocity(): toChassisSpeeds(measured states) gives robot-frame; robotToField(.., heading) for IO.
 *   Gyro: yaw must be CCW+ (many IMUs report CW+; negate those).
 */

// ---- Geometry (PLACEHOLDERS: measure on the real frame, wheel contact centre to centre) ----
// The desk is about 2:1 and wider than it is deep, and forward is the monitor back, so the
// left-right track is the long side.
const WHEELBASE = 0.40;    // m, front-to-back (robot x) distance between module centres. MEASURE.
const TRACK_WIDTH = 0.60;  // m, left-to-right (robot y) distance between module centres. MEASURE.

function rectModules(wheelbase = WHEELBASE, trackWidth = TRACK_WIDTH) {
  const a = wheelbase / 2, b = trackWidth / 2;
  return [{ x: a, y: b }, { x: a, y: -b }, { x: -a, y: b }, { x: -a, y: -b }];   // FL FR RL RR
}

// ---- MAXSwerve constants ----
const DRIVE_PINION_TEETH = 13;                                  // 13T pinion option (12T/14T also exist)
const DRIVE_REDUCTION = (45 * 22) / (DRIVE_PINION_TEETH * 15);  // 45T bevel, 22T spur, 15T bevel pinion: ~5.077:1
const WHEEL_DIAMETER = 0.0762;                                  // m, 3 in wheel
const WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;           // m
const NEO_FREE_RPM = 5676;
const WHEEL_FREE_SPEED = (NEO_FREE_RPM / 60) * WHEEL_CIRCUMFERENCE / DRIVE_REDUCTION;   // ~4.46 m/s (no load)
const DRIVE_POSITION_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_REDUCTION;   // m per motor rev (SPARK MAX encoder factor)
const DRIVE_VELOCITY_FACTOR = DRIVE_POSITION_FACTOR / 60;              // m/s per motor rpm
const TURN_POSITION_FACTOR = 2 * Math.PI;                             // rad per absolute-encoder rev
// Chassis angular offset per module, as in REV's MAXSwerve template (how each module is mounted
// relative to the chassis). Encoder angle = robot angle + offset.
const ANGULAR_OFFSETS = [-Math.PI / 2, 0, Math.PI, Math.PI / 2];      // FL FR RL RR

const ZERO_EPS = 1e-4;   // chassis speed components below this count as "stopped"

// Wrap to (-pi, pi].
function wrapAngle(a) {
  const TAU = 2 * Math.PI;
  let r = a - TAU * Math.floor((a + Math.PI) / TAU);   // [-pi, pi)
  if (r <= -Math.PI) r += TAU;
  if (r > Math.PI) r -= TAU;
  return r;
}

function applyOffset(robotAngle, i) { return wrapAngle(robotAngle + ANGULAR_OFFSETS[i]); }   // -> encoder angle
function removeOffset(encoderAngle, i) { return wrapAngle(encoderAngle - ANGULAR_OFFSETS[i]); } // -> robot angle

function fieldToRobot(vx, vy, heading) {
  const c = Math.cos(heading), s = Math.sin(heading);
  return { vx: c * vx + s * vy, vy: -s * vx + c * vy };
}
function robotToField(vx, vy, heading) {
  const c = Math.cos(heading), s = Math.sin(heading);
  return { vx: c * vx - s * vy, vy: s * vx + c * vy };
}
// IO field theta (io.js) <-> WPILib heading of robot +x in the same field axes. See the header.
function ioThetaToHeading(thetaIo) { return wrapAngle(thetaIo + Math.PI / 2); }
function headingToIoTheta(heading) { return wrapAngle(heading - Math.PI / 2); }

// Scale all wheel speeds by one factor so none exceeds maxSpeed (keeps chassis direction and ratios).
function desaturate(states, maxSpeed) {
  if (!(maxSpeed > 0)) return states;
  const top = Math.max(...states.map(s => Math.abs(s.speed)));
  if (top <= maxSpeed) return states;
  const k = maxSpeed / top;
  return states.map(s => ({ speed: s.speed * k, angle: s.angle }));
}

// Never turn more than 90 degrees: flip the target by pi and reverse the wheel instead.
function optimize(state, currentAngle) {
  const err = wrapAngle(state.angle - currentAngle);
  if (Math.abs(err) > Math.PI / 2) return { speed: -state.speed, angle: wrapAngle(state.angle + Math.PI) };
  return { speed: state.speed, angle: wrapAngle(state.angle) };
}
// Slow a wheel that is not yet pointing where it should (cos of the remaining error, never negative).
function cosineScale(speed, angleError) { return speed * Math.max(0, Math.cos(wrapAngle(angleError))); }

function createKinematics(modules = rectModules()) {
  if (!Array.isArray(modules) || modules.length !== 4) throw new Error('need 4 module positions');
  const m = modules.map(p => ({ x: +p.x, y: +p.y }));
  // A (8x3): rows [1 0 -y_i] and [0 1 x_i]. pinv = (A^T A)^-1 A^T, computed once.
  const A = [];
  for (const p of m) { A.push([1, 0, -p.y]); A.push([0, 1, p.x]); }
  const N = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (const r of A) for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) N[i][j] += r[i] * r[j];
  const Ni = inv3(N);
  const pinv = [0, 1, 2].map(i => A.map(r => Ni[i][0] * r[0] + Ni[i][1] * r[1] + Ni[i][2] * r[2]));

  // Inverse kinematics. currentAngles: kept when the command is ~zero (WPILib behaviour), else 0.
  function toModuleStates(vx, vy, omega, { maxSpeed, currentAngles } = {}) {
    if (Math.abs(vx) < ZERO_EPS && Math.abs(vy) < ZERO_EPS && Math.abs(omega) < ZERO_EPS) {
      return m.map((_, i) => ({ speed: 0, angle: wrapAngle(currentAngles ? currentAngles[i] : 0) }));
    }
    const states = m.map(p => {
      const wx = vx - omega * p.y, wy = vy + omega * p.x;
      return { speed: Math.hypot(wx, wy), angle: wrapAngle(Math.atan2(wy, wx)) };
    });
    return maxSpeed === undefined ? states : desaturate(states, maxSpeed);
  }

  // Forward kinematics, least squares over all 8 wheel velocity components.
  function twistFromVectors(comp) {
    const out = [0, 0, 0];
    for (let i = 0; i < 3; i++) for (let k = 0; k < 8; k++) out[i] += pinv[i][k] * comp[k];
    return out;
  }
  function toChassisSpeeds(states) {
    const comp = [];
    for (const s of states) { comp.push(s.speed * Math.cos(s.angle)); comp.push(s.speed * Math.sin(s.angle)); }
    const [vx, vy, omega] = twistFromVectors(comp);
    return { vx, vy, omega };
  }

  return { modules: m.map(p => ({ ...p })), pinv, toModuleStates, toChassisSpeeds, twistFromVectors };
}

function inv3(a) {
  const [[a0, a1, a2], [b0, b1, b2], [c0, c1, c2]] = a;
  const A0 = b1 * c2 - b2 * c1, A1 = -(b0 * c2 - b2 * c0), A2 = b0 * c1 - b1 * c0;
  const det = a0 * A0 + a1 * A1 + a2 * A2;
  if (Math.abs(det) < 1e-12) throw new Error('module layout is degenerate');
  return [
    [A0 / det, -(a1 * c2 - a2 * c1) / det, (a1 * b2 - a2 * b1) / det],
    [A1 / det, (a0 * c2 - a2 * c0) / det, -(a0 * b2 - a2 * b0) / det],
    [A2 / det, -(a0 * c1 - a1 * c0) / det, (a0 * b1 - a1 * b0) / det]
  ];
}

/**
 * Odometry like WPILib SwerveDriveOdometry: per update, wheel distance deltas (along each module's
 * current angle) -> least-squares robot twist (dx, dy); heading change comes from the gyro, not the
 * wheels; the twist is integrated as a constant-curvature arc (pose exponential), which is exact for
 * a constant twist over the step. Pose {x, y, theta}: field frame, theta = heading of robot +x.
 * moduleDistances: cumulative drive positions (m). moduleAngles: robot-frame angles (offset removed).
 */
class Odometry {
  constructor(kinematics = createKinematics(), { gyroYaw = 0, distances = [0, 0, 0, 0], pose = { x: 0, y: 0, theta: 0 } } = {}) {
    this.kin = kinematics;
    this.lastDist = distances.slice();
    this.lastGyro = gyroYaw;
    this.gyroOffset = pose.theta - gyroYaw;
    this.pose = { x: pose.x, y: pose.y, theta: wrapAngle(pose.theta) };
  }

  // Sets the pose; the next update's distances and gyro are taken relative to the current readings.
  resetPose(pose, { gyroYaw = this.lastGyro, distances = this.lastDist } = {}) {
    this.lastDist = distances.slice();
    this.lastGyro = gyroYaw;
    this.gyroOffset = pose.theta - gyroYaw;
    this.pose = { x: pose.x, y: pose.y, theta: wrapAngle(pose.theta) };
    return this.getPose();
  }

  update(moduleDistances, moduleAngles, gyroYaw) {
    const comp = [];
    for (let i = 0; i < 4; i++) {
      const d = moduleDistances[i] - this.lastDist[i];
      comp.push(d * Math.cos(moduleAngles[i])); comp.push(d * Math.sin(moduleAngles[i]));
    }
    const [dx, dy] = this.kin.twistFromVectors(comp);
    const theta = wrapAngle(gyroYaw + this.gyroOffset);
    const dth = wrapAngle(theta - this.pose.theta);
    let s, c;   // sin(dth)/dth and (1 - cos(dth))/dth, series near 0
    if (Math.abs(dth) < 1e-9) { s = 1 - dth * dth / 6; c = dth / 2; }
    else { s = Math.sin(dth) / dth; c = (1 - Math.cos(dth)) / dth; }
    const lx = dx * s - dy * c, ly = dx * c + dy * s;
    const f = robotToField(lx, ly, this.pose.theta);
    this.pose = { x: this.pose.x + f.vx, y: this.pose.y + f.vy, theta };
    this.lastDist = moduleDistances.slice();
    this.lastGyro = gyroYaw;
    return this.getPose();
  }

  getPose() { return { ...this.pose }; }
}

// Default-geometry instance, for convenience.
const DEFAULT = createKinematics();
const toModuleStates = DEFAULT.toModuleStates, toChassisSpeeds = DEFAULT.toChassisSpeeds;

module.exports = {
  WHEELBASE, TRACK_WIDTH, DRIVE_PINION_TEETH, DRIVE_REDUCTION, WHEEL_DIAMETER, WHEEL_CIRCUMFERENCE,
  NEO_FREE_RPM, WHEEL_FREE_SPEED, DRIVE_POSITION_FACTOR, DRIVE_VELOCITY_FACTOR, TURN_POSITION_FACTOR,
  ANGULAR_OFFSETS, ZERO_EPS,
  rectModules, createKinematics, toModuleStates, toChassisSpeeds, desaturate, optimize, cosineScale, wrapAngle,
  applyOffset, removeOffset, fieldToRobot, robotToField, ioThetaToHeading, headingToIoTheta,
  Odometry
};

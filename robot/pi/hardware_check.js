'use strict';

/**
 * Hardware health check: is every swerve module there and reading sensibly? Run at start-up and
 * twice a second after that (runtime.js). Any problem blocks all motion and ends the current trip;
 * the app shows the first problem in plain words. A pure function, tested without hardware
 * (tests/pi/test_hardware_check.js).
 *
 * Input: io.readModules() entries. Besides the fields motion_guard.js uses, each may carry
 * (all optional; a missing field is not checked):
 *   present    false = one of the module's SPARK MAXes isn't answering on CAN
 *   encoderOk  false = the steering absolute encoder isn't reading
 *   angleDeg   absolute steering angle; not a finite number = encoder not reading
 *   busVolts   supply voltage seen by the module's controllers
 *   faults     array of fault names the controllers report (e.g. 'overcurrent', 'motor type')
 */

const HW = {
  minVolts: 10.5,     // below this the SPARKs brown out under load: don't start driving
  okSeconds: 2        // a problem must stay gone this long before motion is allowed again
};

function checkModules(mods, expected = ['FL', 'FR', 'RL', 'RR'], limits = {}) {
  const L = { ...HW, ...limits };
  const issues = [];
  if (!Array.isArray(mods)) return [{ name: '', kind: 'no-data', text: 'No readings from the motor controllers: check the CAN connection to the computer.' }];
  const byName = new Map(mods.filter(m => m && m.name).map(m => [m.name, m]));
  for (const name of expected) {
    const m = byName.get(name);
    if (!m) { issues.push({ name, kind: 'missing', text: `${name} module not found: check its CAN wires and power.` }); continue; }
    if (m.present === false) { issues.push({ name, kind: 'missing', text: `${name} not answering on CAN: check its CAN wires and power.` }); continue; }
    if (m.encoderOk === false || ('angleDeg' in m && !Number.isFinite(m.angleDeg)))
      issues.push({ name, kind: 'encoder', text: `${name} steering encoder not reading: check the encoder cable and adapter on its turn controller.` });
    if (Number.isFinite(m.busVolts) && m.busVolts < L.minVolts)
      issues.push({ name, kind: 'volts', text: `Battery voltage low (${m.busVolts.toFixed(1)} V at ${name}): charge the battery.` });
    if (Array.isArray(m.faults) && m.faults.length)
      issues.push({ name, kind: 'fault', text: `${name} controller fault: ${m.faults.join(', ')}.` });
  }
  return issues;
}

/** Stateful wrapper: blocked from the first problem until okSeconds without one. */
function createHardwareMonitor(expected, limits = {}) {
  const L = { ...HW, ...limits };
  let issues = [], okFor = Infinity, everChecked = false;
  return {
    step(mods, dt) {
      everChecked = true;
      issues = checkModules(mods, expected, L);
      okFor = issues.length ? 0 : okFor + dt;
      return this.state();
    },
    state() {
      const blocked = !everChecked || issues.length > 0 || okFor < L.okSeconds;
      return { blocked, issues: issues.slice(), text: issues.length ? issues[0].text : (blocked && everChecked ? 'Hardware check: waiting for readings to stay normal' : '') };
    }
  };
}

module.exports = { checkModules, createHardwareMonitor, HW };

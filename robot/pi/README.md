## Swerve kinematics (swerve.js)

Pure, dependency-free kinematics and odometry for the four REV MAXSwerve modules, for the future
`pi_io.js`. The runtime does not use it. Test: `node tests/pi/test_swerve.js`.

- Robot frame follows WPILib: +x forward (the monitor's back), +y left, CCW+. Modules are ordered FL, FR, RL, RR.
- `WHEELBASE` / `TRACK_WIDTH` are placeholders (0.40 m x 0.60 m). Measure them on the real frame.
- IO field frame (io.js): `heading = theta_io + pi/2` (`ioThetaToHeading`). The axes are shared, so
  `writeWheels` does `fieldToRobot(vx, vy, heading)` -> `toModuleStates` -> `optimize` -> `cosineScale` -> `applyOffset`.
  The header of swerve.js has the details.

## Saved data (room map and no-go zones)

The runtime keeps the learned room map and the no-go zones in `~/.maxdesk` on the Pi (set
`MAXDESK_DATA_DIR`, or pass `dataDir` to `createRuntime`, to use another folder; `dataDir: null`
keeps them in memory only). The map is saved every 10 s while it changes and again on `stop()`;
zones are saved as soon as you change them in the app. Every write goes to a temp file, is flushed,
then renamed, so pulling the power mid-save leaves the previous copy intact. Delete the folder to
make the desk forget the room.

## Motion guard (motion_guard.js)

Two checks sit between the planner and `io.writeWheels`:

- **Slew backstop.** Speed may rise by at most 1.5 m/s² and turning rate by 6 rad/s². The simulator
  core already ramps more gently than that (0.5 m/s² and 2 rad/s² up), so in normal driving this
  changes nothing (a test checks a full summon is passed through untouched). It only catches
  sudden jumps from a bug or state reset. Slowing down and stopping are never delayed.
- **Jam stop.** If `io.readModules()` reports a wheel asked to move (≥ 0.1 m/s) that turns at less
  than a quarter of that while its drive draws over 20 A, or steering more than 30° off target
  while the NEO 550 draws over 12 A, for 0.5 s, the trip is cancelled, the wheels stop, the app
  shows "FL wheel can't turn: jammed or pushing against something" and the History tab logs the
  trip as "Wheel jammed". Nothing resumes on its own; a new summon tries again. `pi_io.js` must
  provide `readModules()` for this to work (see `io.js`).

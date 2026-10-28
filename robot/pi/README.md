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

## Position: drift, LiDAR correction and getting lost (prototyped in the simulator)

The simulator's "Sensors & drift" panel models what the real desk will face: wheel odometry and the
gyro drift, so the desk's *believed* position slowly walks away from the *real* one. The fix is in
the simulator core (`robot/simulator/summon_table_sim.html`, search for "LiDAR position correction"),
so `runtime.js` already carries it; on the robot it will run against `pi_io.js`'s real scans.

1. **Tracking**, 5 times a second: slide the scan around the believed pose (±16 cm, ±4°) to line it
   up with the learned map and the walls; small corrections win ties, so it doesn't slide along a
   bare wall.
2. **Suspect**: when the scan disagrees with the map (points on long-known open floor, or beyond a
   wall), or the IMU feels a shove, map updates pause and a wider search (±60 cm, ±12°) runs.
3. **Lost**: if that fails, the desk stops, refuses summons, and searches the whole room every
   second. It only accepts a clear, unique winner (or the same clean winner three times running);
   otherwise it stays stopped and after 10 s asks to be pushed near the dock or for the room to be
   relearned. Being stopped is safe; being confidently wrong is not.

People are tracked separately and left out of the matching. Learn the room first: with a learned map
the error stayed within 6 cm (good floor, gyro) / 15 cm (slippery floor, no gyro) over three round
trips; without correction it reached 68 cm and missed the dock every time. Repeated shoves of up to
~2 m: 97 of 100 random cases found themselves (all but one within 5 s), 2 stayed safely stopped, and 1
on the slippery no-gyro floor ended 66 cm off while showing "weak" (not "locked") after a small
bump: on that floor, long stretches without a good match still drift (`tests/sim/scenarios/drift.js`
runs 12 of the bump cases).

What the real desk needs for this: a LiDAR range of at least ~5 m, a gyro/IMU (also used to feel
shoves), and the room learned once with "Learn the room".

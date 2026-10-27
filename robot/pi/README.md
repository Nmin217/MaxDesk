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

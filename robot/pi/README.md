## Swerve kinematics (swerve.js)

Pure, dependency-free kinematics and odometry for the four REV MAXSwerve modules, for the future
`pi_io.js`. The runtime does not use it. Test: `node tests/pi/test_swerve.js`.

- Robot frame follows WPILib: +x forward (the monitor's back), +y left, CCW+. Modules are ordered FL, FR, RL, RR.
- `WHEELBASE` / `TRACK_WIDTH` are placeholders (0.40 m x 0.60 m). Measure them on the real frame.
- IO field frame (io.js): `heading = theta_io + pi/2` (`ioThetaToHeading`). The axes are shared, so
  `writeWheels` does `fieldToRobot(vx, vy, heading)` -> `toModuleStates` -> `optimize` -> `cosineScale` -> `applyOffset`.
  The header of swerve.js has the details.

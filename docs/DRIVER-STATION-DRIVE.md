# FRC Driver Station drive mode (implementation template)

This adds a separate **manual Driver Station / joystick drive** option for the Summon Desk. It is an integration template for the eventual WPILib robot project, not a claim that real hardware is already supported or tested.

## Intended behavior

- Driver Station joystick control is the manual drive input.
- Normal speed is capped at **0.45 m/s translation** and **0.60 rad/s rotation** (initial commissioning values only).
- Holding the configured speed-override button raises the cap to **0.90 m/s** and **1.00 rad/s**.
- **The speed override never disables lidar supervision.** Lidar checks run every command execution at either speed setting.
- If the lidar frame is invalid, missing, or older than **150 ms**, the command requests a controlled stop and reports a safety hold.
- Obstacle proximity can cap translation or refuse a turn; the drivetrain output callback receives a `safetyHold` flag so the real drive subsystem can apply its configured controlled-braking behavior.
- Releasing the command or interrupting it requests a stop.

These numbers are conservative starting values, not certified safe limits. Test with a weighted dummy load, in a closed area, at low speed first. The physical E-stop must stop the drive independently of the Driver Station, phone, network, lidar, and roboRIO application code.

## Files

- `robot/java-core/wpilib-glue/DriverStationSafetyDriveCommand.java` — WPILib command template.
- Reuses `robot/java-core/src/main/java/frc/robot/summon/core/SafetySupervisor.java` — the same lidar proximity supervisor used by the Java core.

The existing Java-core build does not include WPILib, so this glue file is **not compiled by `scripts/verify.sh`**. Copy it into the actual WPILib robot project and compile there after adapting it to the team's swerve API.

## Required wiring in `RobotContainer`

Set this command as the swerve subsystem's default command in the actual WPILib project. Supply:

1. normalized, sign-corrected Driver Station axes (forward, strafe, rotate);
2. current `Pose2d` from swerve odometry;
3. measured field-relative `ChassisSpeeds` from the drivetrain;
4. the latest fused lidar frame, including its FPGA-time timestamp;
5. a `SafeDriveOutput` adapter that sends field-relative speeds to the swerve subsystem and honors `safetyHold` with a controlled stop/brake.

Example shape (adapt names to the actual robot project):

```java
DriverStationSafetyDriveCommand teleopDrive = new DriverStationSafetyDriveCommand(
    swerve,
    () -> -controller.getLeftY(),
    () -> -controller.getLeftX(),
    () -> -controller.getRightX(),
    () -> controller.getLeftBumper(), // hold-to-enable speed cap increase only
    swerve::getPose,
    swerve::getFieldRelativeSpeeds,
    lidarFusion::latestFrame,
    (speeds, safetyHold) -> swerve.driveFieldRelativeSafely(speeds, safetyHold));
swerve.setDefaultCommand(teleopDrive);
```

`getFieldRelativeSpeeds`, `latestFrame`, and `driveFieldRelativeSafely` above are adapter names to implement; they are not methods already present in this archive. Choose a physical override button only after driver training and make the button state visible on the dashboard.

## Lidar data contract — important

`LidarFrame.fieldHits` must contain obstacle points as `{x, y}` arrays in **field coordinates, meters**, timestamped from the roboRIO FPGA clock. The coprocessor must transform both lidar scans using calibrated sensor positions/orientations and current robot pose, then fuse them. Do not pass raw sensor-relative points into the supervisor. A fresh, healthy scan may legitimately contain zero obstacle returns in a clear area; keep that separate from sensor-health/freshness status. The adapter must mark a frame invalid when the scan stream is unhealthy, not merely because no obstacles were detected.

The current supervisor uses point returns and a rectangular footprint. It is not a safety-rated person detector and cannot guarantee detection of every person or obstacle. A lidar below the tabletop cannot see everything at tabletop height; sensor coverage, desk overhang, braking distance under load, communications loss, and blind zones require physical review and controlled tests. Keep the physical E-stop and any required hardware interlock independent.

## Mode selection

Use the Driver Station command for attended manual testing. Do not allow manual joystick input, phone summon/return, and autonomous commands to drive the modules concurrently. Use WPILib command requirements / an explicit mode state so one drive command owns the drivetrain at a time. Switching modes must stop the previous command before enabling the next one.

## Commissioning checklist

- [ ] Verify MAXSwerve module, motor, encoder, and controller configuration before enabling outputs.
- [ ] Calibrate odometry and both lidar extrinsics; verify a wall point lands at the correct field coordinate while the robot is stationary and rotating.
- [ ] Verify stale/missing lidar produces a stop, including unplugging the coprocessor data connection.
- [ ] Verify a near obstacle blocks forward, reverse, strafe, and rotation in the relevant directions.
- [ ] Verify holding speed override changes only the speed cap, not the lidar behavior.
- [ ] Measure stopping distance at each approved speed with the desk loaded and at the tallest permitted desk height; lower caps if needed.
- [ ] Test the physical E-stop separately from software and network controls.

# Summon Desk: Java core

Java port of the parts of the simulator that decide where the desk goes and whether it is safe to move.
Pure Java (no WPILib dependency), so it builds and tests anywhere with a JDK. Uses only Java 17 language features
and standard-library APIs (WPILib projects have used JDK 17; check yours).

## Status

| Piece | Status |
|---|---|
| `OrientationPlanner` (orientation-aware A*, parking pose, tight-margin fallback) | Ported. Same routes as the simulator on 4 of 5 test scenes (every waypoint identical); the 5th has the same length and turn count and parks in the neighboring equal-cost cell. |
| `SafetySupervisor` (swept-footprint speed cap, yaw guard) | Ported. Tested: stops before a wall at full speed, does not hit a box appearing 1.1 m ahead, refuses a turn into a wall. |
| `OccupancyMemory` (long-term map, people kept out of the furniture map, traffic, save/load) | Ported and tested. |
| Path follower (pure pursuit, yaw lag gating, stuck recovery, trail retreat) | **Not ported.** Lives only in the browser simulator. |
| State machine (summon, return, learn-the-room, battery thresholds, tracking people) | **Not ported.** |
| Charging-cord lock | Sketched in `wpilib-glue/SummonSubsystem.java` (checks `PowerIO.cordPluggedIn()` every loop and cancels trips). Not compiled. The simulator implements and tests it. Also needs the hardware layer (inlet-empty switch in series with the contactor coil). |
| `wpilib-glue/SummonSubsystem.java` | Skeleton, **not compiled or tested** (needs WPILib). |
| Swerve drive, lidar driver, UPS reader | Use the REV MAXSwerve template / vendor libraries; the lidar driver and Network UPS Tools run on the coprocessor and publish over NetworkTables. |

## Build and test

    java tools/Build.java                                   # compiles src/ into out/
    java -cp out frc.robot.summon.core.CoreTests            # 18 checks + cross-check lines

The `XC ...` lines are the cross-check against the simulator: same scenes, same format, compare line by line.

## Use in a WPILib project

Copy `src/main/java/frc/robot/summon/core/` into `src/main/java/frc/robot/summon/core/` of your robot project, add
`wpilib-glue/SummonSubsystem.java` as `frc/robot/summon/SummonSubsystem.java`, and wire its suppliers to your swerve
subsystem. Units are meters / seconds / radians.

## Things to set from the real robot

- `Config.hw`, `Config.hh`: half-extents of the LARGEST outline at any height (desktop overhang, monitor), not just the
  frame. The values here (0.56 x 0.36 m) come from the simulator. The old Python package used different ones.
- `Config.safeDecel`, `emergencyDecel`: must be below what the real, loaded robot can do (measure it, at full desk height).
- Lidar placement: mount on the frame at a fixed low height. A low lidar cannot see obstacles at desktop height, so add a
  depth camera or a second scan plane there.

## Why Java

The roboRIO side of an FRC robot is best served by Java: first-class WPILib support, the swerve templates, simulation and
logging tools, and REVLib. Heavy sensor drivers (lidar) usually live on a coprocessor in whatever language has the SDK.
Planning is light (a couple of milliseconds on a laptop JVM) and can run on either; measure on the roboRIO before choosing.

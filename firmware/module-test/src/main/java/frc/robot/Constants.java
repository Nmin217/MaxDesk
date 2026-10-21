package frc.robot;

import java.util.List;

/**
 * Everything you may need to change for the module bench test lives here.
 *
 * <p>CAN IDs follow REV's MAXSwerve template so this test grows into the full four-module
 * drivetrain without renumbering. Each module is named after the corner it will go on.
 */
public final class Constants {
  private Constants() {}

  /** One module: dashboard name, drive SPARK CAN ID, turn SPARK CAN ID. */
  public record ModuleId(String name, int driveCanId, int turnCanId) {}

  /** The modules on the bench. Comment out any that aren't wired; add RR when you build it. */
  public static final List<ModuleId> kModules = List.of(
      new ModuleId("FL", 11, 10),
      new ModuleId("FR", 15, 14),
      new ModuleId("RL", 13, 12)
      // , new ModuleId("RR", 17, 16)
  );

  public static final class ModuleConstants {
    /** 12, 13 or 14: the pinion actually fitted. These modules have the 13T (medium); PLAN.md suggested 12T. */
    public static final int kDrivingMotorPinionTeeth = 13;

    /**
     * Which Through Bore Encoder is on the module. MAXSwerve kits sold before 2025 have the V1
     * (black housing, 6-pin JST); V2 has a different housing and a status LED. Wrong choice =
     * wrong angle readings.
     */
    public static final boolean kThroughBoreEncoderV2 = false;

    public static final double kWheelDiameterMeters = 0.0762; // 3 in
    // 45T wheel bevel, 22T first-stage spur, 15T bevel pinion (REV MAXSwerve)
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kNeoFreeSpeedRpm = 5676;
    public static final double kDriveWheelFreeSpeedMps =
        (kNeoFreeSpeedRpm / 60.0) * (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction;

    public static final int kDriveCurrentLimitAmps = 40; // PLAN.md §5
    public static final int kTurnCurrentLimitAmps = 20; // NEO 550: never run it unlimited
  }

  public static final class TestLimits {
    /** Hard cap for every drive command in this program, as a fraction of free speed. */
    public static final double kMaxDrivePercent = 0.15;
    /** Speed used by the scripted drive steps. */
    public static final double kScriptDrivePercent = 0.10;
    /** Steer is "at target" when within this many degrees. */
    public static final double kSteerToleranceDeg = 5.0;
    /** A steer move that hasn't reached its target in this long is reported as a fault. */
    public static final double kSteerTimeoutSec = 2.0;
    /** Joystick deadband. */
    public static final double kDeadband = 0.08;
  }

  public static final class OI {
    public static final int kControllerPort = 0;
  }
}

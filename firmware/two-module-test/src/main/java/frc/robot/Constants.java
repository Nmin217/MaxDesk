package frc.robot;

/**
 * Everything you may need to change for the two-module bench test lives here.
 *
 * <p>CAN IDs follow REV's MAXSwerve template (front-left 10/11, front-right 14/15) so this test
 * grows into the full four-module drivetrain without renumbering.
 */
public final class Constants {
  private Constants() {}

  public static final class ModuleIds {
    // Module A = will become FRONT LEFT.
    public static final int kADriveCanId = 11;
    public static final int kATurnCanId = 10;
    // Module B = will become FRONT RIGHT.
    public static final int kBDriveCanId = 15;
    public static final int kBTurnCanId = 14;
  }

  public static final class ModuleConstants {
    /** 12, 13 or 14. PLAN.md calls for the 12T (low) pinion. */
    public static final int kDrivingMotorPinionTeeth = 12;

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

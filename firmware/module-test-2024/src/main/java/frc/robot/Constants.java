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

  /**
   * One module:
   * <ul>
   *   <li>name, drive SPARK CAN ID, turn SPARK CAN ID;
   *   <li>xMeters / yMeters: where its centre (the steering axis) sits, metres forward of the frame's centre and
   *       metres to the left. Only the Cart modes use the position;
   *   <li>zeroPointsDeg: which way this wheel points, in robot terms, when its encoder reads 0, i.e. right after
   *       you zeroed it with the 3D-printed jig. 0 = forward, 90 = left, 180 = back, -90 = right. The jig sits in
   *       the frame corner, and each corner's module is turned 90 deg from the next, so the jig's "zero" is turned
   *       90 deg per corner as well. This number undoes that. Zeroed with every wheel pointing forward instead?
   *       Then it's 0 for all of them;
   *   <li>encoderV2: true if this module has the Through Bore Encoder V2 (newer housing with a status LED),
   *       false for the original V1 (black housing). Each module can differ. (The 2024 program reads both the
   *       same way, so it ignores this.)
   * </ul>
   */
  public record ModuleId(String name, int driveCanId, int turnCanId, double xMeters, double yMeters,
      double zeroPointsDeg, boolean encoderV2) {}

  /** Placeholder half-spacing of the square frame (module centre to frame centre, each way). Measure it. */
  private static final double kHalf = 0.25;

  /**
   * The four-module square frame (photo, 2026-11-19). The zero directions are REV's MAXSwerve pattern for a jig
   * zero: FL points left, FR forward, BL back, BR right. CAN IDs confirmed by the owner (front = top of the photo):
   * top-left 1/2, top-right 3/4, bottom-left 5/6, bottom-right 7/8; odd = drive, even = turn. BR has the V2 encoder.
   */
  public static final List<ModuleId> kModules = List.of(
      new ModuleId("FL", 1, 2,  kHalf,  kHalf,   90, false),
      new ModuleId("FR", 3, 4,  kHalf, -kHalf,    0, false),
      new ModuleId("BL", 5, 6, -kHalf,  kHalf,  180, false),
      new ModuleId("BR", 7, 8, -kHalf, -kHalf,  -90, true)   // the V2 encoder
  );

  /** Set true once the positions above are measured. Until then the Cart modes print a warning. */
  public static final boolean kCartPositionsMeasured = false;  // kHalf is a guess from the photo

  public static final class ModuleConstants {
    /** 12, 13 or 14: the pinion actually fitted. These modules have the 13T (medium); PLAN.md suggested 12T. */
    public static final int kDrivingMotorPinionTeeth = 13;

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
    /** Drive current limit on the bench. Plenty to spin a free wheel; half the robot's 40 A. */
    public static final int kBenchDriveCurrentLimitAmps = 20;
    /**
     * Stall cut-off: if the drive is asked to move but the wheel stays below kStallSpeedMps while
     * drawing more than kStallAmps for kStallSec, that module's drive is shut off until the next
     * enable. A free wheel never draws that much at bench speeds; a jammed one does.
     */
    public static final double kStallAmps = 10.0;
    public static final double kStallSpeedMps = 0.05;
    public static final double kStallSec = 0.5;
    /** Steering stall: NEO 550 over this current with the module turning slower than this. */
    public static final double kTurnStallAmps = 12.0;
    public static final double kTurnStallRadPerSec = 0.5;
    /** Cart modes: full stick = this fraction of the top speed as a turning rate (spin in place). */
    public static final double kCartTurnScale = 1.0;
    /** Phone app mode: stop if no new command has arrived for this long. */
    public static final double kAppTimeoutSec = 0.3;
  }

  public static final class OI {
    public static final int kControllerPort = 0;
  }
}

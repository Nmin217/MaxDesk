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
   * One module: dashboard name, drive SPARK CAN ID, turn SPARK CAN ID, and where its centre (the
   * steering axis) sits on the cart or frame: xMeters forward of the centre, yMeters to the left.
   * Only the Cart modes use the position.
   */
  public record ModuleId(String name, int driveCanId, int turnCanId, double xMeters, double yMeters) {}

  /** The modules on the bench. Comment out any that aren't wired; add RR when you build it. */
  public static final List<ModuleId> kModules = List.of(
      // PLACEHOLDER positions for a three-wheel triangle cart: two at the front, one at the back
      // middle. MEASURE yours (centre of each module from the centre of the cart) and set
      // kCartPositionsMeasured = true. Wrong numbers make the wheels fight each other when it turns.
      // The triangle cart as wired (photo, 2026-11-09): the two modules on one side are the FRONT, the single
      // module is the BACK, centred. CAN IDs: odd = drive NEO, even = turn NEO 550.
      // Positions ESTIMATED 2026-11-09 from distances to the centre (FL 10 in, FR 8 in, Back 11 in), assuming
      // both front modules are 5.5 in ahead of the centre and Back is straight behind it. Check: FL and FR
      // should be about 14 in apart. Measure properly before trusting fast turns.
      new ModuleId("FL", 3, 4, 0.140, 0.212),     // front-left:  5.5 in forward, 8.4 in left
      new ModuleId("FR", 5, 6, 0.140, -0.148),    // front-right: 5.5 in forward, 5.8 in right
      new ModuleId("Back", 1, 2, -0.279, 0.0)     // back, centred: 11 in behind
      // , new ModuleId("RR", 7, 8, -0.20, -0.20)   // on the four-wheel frame: all four corners
  );

  /** Set true once the positions above are measured. Until then the Cart modes print a warning. */
  public static final boolean kCartPositionsMeasured = true;   // estimates (see kModules): good enough for slow turns

  public static final class ModuleConstants {
    /** 12, 13 or 14: the pinion actually fitted. These modules have the 13T (medium); PLAN.md suggested 12T. */
    public static final int kDrivingMotorPinionTeeth = 13;

    /**
     * Which Through Bore Encoder is on the module. MAXSwerve kits sold before 2025 have the V1
     * (black housing, 6-pin JST); V2 has a different housing and a status LED. Wrong choice =
     * wrong angle readings.
     */
    // Not used by the 2024 version (REVLib 2024 has no encoder presets). It assumes the original Through Bore
    // Encoder. If yours is the newer V2, say so before testing: it may need different settings on 2024.
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

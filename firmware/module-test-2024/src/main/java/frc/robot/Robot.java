package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OI;
import frc.robot.Constants.TestLimits;
import java.util.List;

/**
 * Bench test for MAXSwerve modules (the ones listed in {@link Constants#kModules}).
 * MODULES ON BLOCKS, WHEELS OFF THE GROUND.
 *
 * <p>Pick a mode from "Test Mode" on the dashboard, then enable TELEOP in the Driver Station.
 * Every enable restarts the selected test from the beginning. Results print to the Driver
 * Station console and are shown under "Result/..." on the dashboard.
 *
 * <ul>
 *   <li><b>Readout</b>: never moves anything. Turn the wheels by hand and watch the angles.
 *   <li><b>Self test</b>: steering direction check, steer to 0/90/180/270/0, then drive slowly
 *       forward and back at 0 deg. All modules run at once; each passes or fails on its own and a
 *       failed module stops.
 *   <li><b>Joystick</b>: hold RIGHT BUMPER (deadman) and use the left stick; every module
 *       follows the stick. A/B/X/Y point all wheels at 0/90/180/270 deg without driving.
 *   <li><b>Phone app</b>: the same as Joystick, but driven from the test page in
 *       tests/hardware/app-drive-test through its laptop link. The page only sends while a finger
 *       is on it; if commands stop arriving for {@link TestLimits#kAppTimeoutSec}, everything stops.
 *   <li><b>Cart</b> (joystick or phone): MODULES BOLTED TO A CART OR FRAME. Real swerve: each wheel
 *       gets its own angle and speed from WPILib's SwerveDriveKinematics, using the module positions
 *       in {@link Constants#kModules}, so the cart can drive any direction and turn at the same
 *       time. Joystick: hold RB, left stick drives, right stick left/right turns. Robot-relative
 *       (no gyro yet): "forward" is the cart's front.
 * </ul>
 */
public class Robot extends TimedRobot {
  private enum Mode { READOUT, SELF_TEST, JOYSTICK, APP, CART_JOYSTICK, CART_APP }

  private final List<TestModule> modules = Constants.kModules.stream()
      .map(id -> new TestModule(id.name(), id.driveCanId(), id.turnCanId()))
      .toList();
  private final List<SelfTest> selfTests = modules.stream().map(SelfTest::new).toList();

  private final XboxController controller = new XboxController(OI.kControllerPort);
  /** [seq, x, y, pointDeg] from the app link; x forward, y left, -1..1; pointDeg < 0 = none. */
  private final DoubleArraySubscriber appCommand = NetworkTableInstance.getDefault()
      .getDoubleArrayTopic("/AppDrive/cmd").subscribe(new double[0]);
  private double appLastSeq = Double.NaN;
  private double appFreshSince = Double.NEGATIVE_INFINITY;
  /** Null if the module positions can't make a swerve (fewer than 2 modules): the Cart modes refuse. */
  private final SwerveDriveKinematics kinematics = makeKinematics();
  private final double cartMaxSpeedMps =
      TestLimits.kMaxDrivePercent * ModuleConstants.kDriveWheelFreeSpeedMps;
  private final double cartMaxOmega = cartMaxSpeedMps / Math.max(0.05, Constants.kModules.stream()
      .mapToDouble(id -> Math.hypot(id.xMeters(), id.yMeters())).max().orElse(0.05));
  private final SendableChooser<Mode> modeChooser = new SendableChooser<>();
  private Mode mode = Mode.READOUT;
  // The Xbox controller picks the mode too, so no dashboard is needed: Start = next, Back = previous, only while
  // DISABLED (a mode never changes mid-drive). A change made on the dashboard's "Test Mode" chooser wins.
  private Mode picked = null;
  private Mode lastChooser = null;
  private double rumbleUntil = 0;
  private Rotation2d joystickAngle = new Rotation2d();

  public Robot() {
    modeChooser.setDefaultOption("Readout (no motion)", Mode.READOUT);
    modeChooser.addOption("Self test (scripted)", Mode.SELF_TEST);
    modeChooser.addOption("Joystick (hold RB)", Mode.JOYSTICK);
    modeChooser.addOption("Phone app (test link)", Mode.APP);
    modeChooser.addOption("Cart: joystick (RB, right stick turns)", Mode.CART_JOYSTICK);
    modeChooser.addOption("Cart: phone app", Mode.CART_APP);
    SmartDashboard.putData("Test Mode", modeChooser);
    for (TestModule m : modules) {
      if (!m.configOk()) {
        System.err.println("Module " + m.name() + " did not configure. Fix CAN before enabling.");
      }
    }
  }

  @Override
  public void robotPeriodic() {
    for (TestModule m : modules) m.publish();
    pickModeFromController();
    SmartDashboard.putString("Running Mode", isEnabled() ? mode.toString() : "DISABLED");
    SmartDashboard.putString("Next Mode", label(selectedMode()));
  }

  private static String label(Mode m) {
    return switch (m) {
      case READOUT -> "Readout (no motion)";
      case SELF_TEST -> "Self test (scripted)";
      case JOYSTICK -> "Joystick (hold RB)";
      case APP -> "Phone app (test link)";
      case CART_JOYSTICK -> "Cart: joystick (hold RB, right stick turns)";
      case CART_APP -> "Cart: phone app";
    };
  }

  /** The mode the next enable will run: the controller's pick, unless the dashboard chooser changed since. */
  private Mode selectedMode() {
    Mode ch = modeChooser.getSelected();
    if (ch != lastChooser) {
      if (lastChooser != null) picked = null;   // someone used the dashboard: it wins
      lastChooser = ch;
    }
    return picked != null ? picked : (ch != null ? ch : Mode.READOUT);
  }

  private void pickModeFromController() {
    boolean next = controller.getStartButtonPressed(), prev = controller.getBackButtonPressed();
    double now = Timer.getFPGATimestamp();
    if (rumbleUntil > 0 && now > rumbleUntil) { controller.setRumble(RumbleType.kBothRumble, 0); rumbleUntil = 0; }
    if (!next && !prev) return;
    if (isEnabled()) {
      DriverStation.reportWarning("Disable first to change the mode (now running: " + label(mode) + ")", false);
      return;
    }
    Mode[] all = Mode.values();
    int i = selectedMode().ordinal();
    picked = all[(i + (next ? 1 : all.length - 1)) % all.length];
    DriverStation.reportWarning("Mode " + (picked.ordinal() + 1) + "/" + all.length + ": " + label(picked)
        + ". Enable TeleOperated to run it. (Start = next, Back = previous)", false);
    controller.setRumble(RumbleType.kBothRumble, 0.6);
    rumbleUntil = now + 0.25;
  }

  @Override
  public void disabledInit() {
    stopAll();
  }

  @Override
  public void teleopInit() {
    mode = selectedMode();
    stopAll();
    for (TestModule m : modules) m.clearStall();
    System.out.println("=== Enabled: " + label(mode) + " ===");
    DriverStation.reportWarning("Running: " + label(mode), false);
    if (mode == Mode.SELF_TEST) {
      for (SelfTest t : selfTests) t.start();
    }
    if ((mode == Mode.CART_JOYSTICK || mode == Mode.CART_APP) && kinematics == null) {
      System.err.println("Cart mode needs at least 2 modules in Constants.kModules. Not driving.");
    } else if ((mode == Mode.CART_JOYSTICK || mode == Mode.CART_APP) && !Constants.kCartPositionsMeasured) {
      System.err.println("WARNING: cart module positions are placeholders. Measure them (Constants.kModules)"
          + " or turning will make the wheels fight. Driving straight still works.");
    }
    joystickAngle = new Rotation2d();
    // Ignore whatever command was left over from before this enable.
    double[] cmd = appCommand.get();
    appLastSeq = cmd.length >= 4 ? cmd[0] : Double.NaN;
    appFreshSince = Double.NEGATIVE_INFINITY;
  }

  @Override
  public void teleopPeriodic() {
    switch (mode) {
      case READOUT -> stopAll();
      case SELF_TEST -> {
        for (SelfTest t : selfTests) t.run();
      }
      case JOYSTICK -> joystick();
      case APP -> app();
      case CART_JOYSTICK -> cartJoystick();
      case CART_APP -> cartApp();
    }
  }

  @Override
  public void teleopExit() {
    stopAll();
  }

  private void joystick() {
    if (!controller.getRightBumper()) {
      stopAll();
      return;
    }
    double pointDeg = -1;
    if (controller.getAButton()) pointDeg = 0;
    if (controller.getBButton()) pointDeg = 90;
    if (controller.getXButton()) pointDeg = 180;
    if (controller.getYButton()) pointDeg = 270;
    // Robot convention: +x forward (stick up), +y left (stick left). Sticks read up/left as -.
    driveAll(-controller.getLeftY(), -controller.getLeftX(), pointDeg);
  }

  private void app() {
    double[] cmd = freshAppCommand();
    if (cmd == null) {
      stopAll();
      return;
    }
    driveAll(cmd[1], cmd[2], cmd[3]);
  }

  /**
   * The phone's latest [seq, x, y, pointDeg, turn] (turn is optional), or null if it has stopped
   * sending (finger lifted, phone asleep, link closed): the caller must stop.
   */
  private double[] freshAppCommand() {
    double[] cmd = appCommand.get();
    double now = Timer.getFPGATimestamp();
    if (cmd.length >= 4 && cmd[0] != appLastSeq) {
      appLastSeq = cmd[0];
      appFreshSince = now;
    }
    if (cmd.length < 4 || now - appFreshSince > TestLimits.kAppTimeoutSec) return null;
    return cmd;
  }

  private void cartJoystick() {
    if (!controller.getRightBumper()) {
      stopAll();
      return;
    }
    // Sticks read up/left as -. Right stick left = turn left (counter-clockwise, +omega).
    cartDrive(-controller.getLeftY(), -controller.getLeftX(), -controller.getRightX());
  }

  private void cartApp() {
    double[] cmd = freshAppCommand();
    if (cmd == null) {
      stopAll();
      return;
    }
    cartDrive(cmd[1], cmd[2], cmd.length >= 5 ? cmd[4] : 0);
  }

  /**
   * Real swerve: x forward, y left, turn counter-clockwise, each -1..1 of the bench top speed.
   * WPILib works out every wheel's angle and speed from the module positions; when one wheel would
   * have to go faster than the cap, all of them are scaled down together so the motion keeps its shape.
   */
  private void cartDrive(double x, double y, double turn) {
    if (kinematics == null) {
      stopAll();
      return;
    }
    double mag = MathUtil.applyDeadband(Math.min(1.0, Math.hypot(x, y)), TestLimits.kDeadband);
    double scale = mag > 0 ? mag / Math.hypot(x, y) : 0;
    double w = MathUtil.applyDeadband(MathUtil.clamp(turn, -1, 1), TestLimits.kDeadband);
    ChassisSpeeds speeds = new ChassisSpeeds(x * scale * cartMaxSpeedMps, y * scale * cartMaxSpeedMps,
        w * TestLimits.kCartTurnScale * cartMaxOmega);
    // With zero speeds WPILib keeps each wheel's last angle, so they don't snap back to 0 on release.
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, cartMaxSpeedMps);
    for (int i = 0; i < modules.size(); i++) {
      modules.get(i).setState(states[i].speedMetersPerSecond / ModuleConstants.kDriveWheelFreeSpeedMps,
          states[i].angle);
    }
  }

  private static SwerveDriveKinematics makeKinematics() {
    if (Constants.kModules.size() < 2) return null;
    return new SwerveDriveKinematics(Constants.kModules.stream()
        .map(id -> new Translation2d(id.xMeters(), id.yMeters())).toArray(Translation2d[]::new));
  }

  /** Every module follows one stick: x forward, y left (-1..1); pointDeg >= 0 sets the angle. */
  private void driveAll(double x, double y, double pointDeg) {
    if (pointDeg >= 0) joystickAngle = Rotation2d.fromDegrees(pointDeg);
    double magnitude = MathUtil.applyDeadband(Math.min(1.0, Math.hypot(x, y)), TestLimits.kDeadband);
    if (magnitude > 0) joystickAngle = new Rotation2d(x, y);
    double percent = magnitude * TestLimits.kMaxDrivePercent;
    for (TestModule m : modules) m.setState(percent, joystickAngle);
  }

  private void stopAll() {
    for (TestModule m : modules) m.stop();
  }

  /** Scripted pass/fail test for one module. */
  private static final class SelfTest {
    private enum Step { DIRECTION, PAUSE, STEER, DRIVE, DONE, FAILED }

    private static final double[] kSteerTargetsDeg = {0, 90, 180, 270, 0};
    // {percent, seconds}: forward, stop, reverse, stop
    private static final double[][] kDriveScript = {
      {TestLimits.kScriptDrivePercent, 2.0}, {0, 1.0},
      {-TestLimits.kScriptDrivePercent, 2.0}, {0, 1.0},
    };

    private final TestModule module;
    private final Timer stepTimer = new Timer();
    private Step step = Step.DONE;
    private int index;
    private double startAngleRad;
    private double startDriveM;
    private double inToleranceSince;

    SelfTest(TestModule module) {
      this.module = module;
    }

    void start() {
      if (!module.configOk()) {
        fail("SPARKs did not configure (CAN ID / wiring)");
        return;
      }
      startAngleRad = module.angleRad();
      enter(Step.DIRECTION);
      report("running");
    }

    void run() {
      double t = stepTimer.get();
      switch (step) {
        case DIRECTION -> {
          // Small open-loop push. With the encoder inverted correctly, + output => + angle.
          module.steerPercent(0.1);
          if (module.turnStalled()) {
            fail("steering stalled: high current, module not turning. Check the NEO 550 phase"
                + " wires (red/black/white) and that the module turns freely");
            return;
          }
          if (t > 0.6) {
            module.stop();
            double movedDeg =
                Math.toDegrees(MathUtil.angleModulus(module.angleRad() - startAngleRad));
            log(String.format("direction check: +0.1 output moved %.1f deg", movedDeg));
            if (movedDeg > 3) {
              enter(Step.PAUSE);
            } else if (movedDeg < -3) {
              fail("encoder counts backwards: absolute encoder must be inverted (see README)");
            } else {
              fail("angle did not change: encoder unplugged, turn motor not moving,"
                  + " or drive/turn CAN IDs swapped");
            }
          }
        }
        case PAUSE -> {
          module.stop();
          if (t > 0.5) {
            index = 0;
            inToleranceSince = -1;
            enter(Step.STEER);
          }
        }
        case STEER -> {
          double target = Math.toRadians(kSteerTargetsDeg[index]);
          module.steerTo(target);
          if (module.turnStalled()) {
            fail("steering stalled: high current, module not turning. Check the NEO 550 phase"
                + " wires and that the module turns freely");
            return;
          }
          module.drivePercent(0);
          double err = Math.abs(module.angleErrorDeg(target));
          if (err < TestLimits.kSteerToleranceDeg) {
            if (inToleranceSince < 0) inToleranceSince = t;
            if (t - inToleranceSince > 0.3) {
              log(String.format("steer %3.0f deg OK (%.1f s, error %.1f deg)",
                  kSteerTargetsDeg[index], t, err));
              index++;
              inToleranceSince = -1;
              if (index == kSteerTargetsDeg.length) {
                index = 0;
                startDriveM = module.drivePositionMeters();
                enter(Step.DRIVE);
              } else {
                stepTimer.restart();
              }
            }
          } else {
            inToleranceSince = -1;
            if (t > TestLimits.kSteerTimeoutSec) {
              fail(String.format("steer to %.0f deg timed out, still %.1f deg off",
                  kSteerTargetsDeg[index], err));
            }
          }
        }
        case DRIVE -> {
          module.steerTo(0);
          module.drivePercent(kDriveScript[index][0]);
          if (module.stalled()) {
            fail("drive stalled: the wheel would not turn. Check the drive train spins freely by hand");
            return;
          }
          if (t > kDriveScript[index][1]) {
            double moved = module.drivePositionMeters() - startDriveM;
            if (kDriveScript[index][0] != 0) {
              log(String.format("drive %+.0f%%: encoder moved %+.3f m",
                  kDriveScript[index][0] * 100, moved));
              if (Math.abs(moved) < 0.01) {
                fail("drive encoder did not move: drive motor not spinning or wrong CAN ID");
                return;
              }
            }
            startDriveM = module.drivePositionMeters();
            index++;
            if (index == kDriveScript.length) {
              module.stop();
              step = Step.DONE;
              report("PASS");
            } else {
              stepTimer.restart();
            }
          }
        }
        case DONE, FAILED -> module.stop();
      }
    }

    private void enter(Step next) {
      step = next;
      stepTimer.restart();
    }

    private void fail(String why) {
      module.stop();
      step = Step.FAILED;
      report("FAIL: " + why);
    }

    private void report(String result) {
      log(result);
      SmartDashboard.putString("Result/" + module.name(), result);
    }

    private void log(String msg) {
      System.out.println("[Module " + module.name() + "] " + msg);
    }
  }
}

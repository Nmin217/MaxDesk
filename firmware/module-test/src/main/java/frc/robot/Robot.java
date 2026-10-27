package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
 * </ul>
 */
public class Robot extends TimedRobot {
  private enum Mode { READOUT, SELF_TEST, JOYSTICK, APP }

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
  private final SendableChooser<Mode> modeChooser = new SendableChooser<>();
  private Mode mode = Mode.READOUT;
  private Rotation2d joystickAngle = Rotation2d.kZero;

  public Robot() {
    modeChooser.setDefaultOption("Readout (no motion)", Mode.READOUT);
    modeChooser.addOption("Self test (scripted)", Mode.SELF_TEST);
    modeChooser.addOption("Joystick (hold RB)", Mode.JOYSTICK);
    modeChooser.addOption("Phone app (test link)", Mode.APP);
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
    SmartDashboard.putString("Running Mode", isEnabled() ? mode.toString() : "DISABLED");
  }

  @Override
  public void disabledInit() {
    stopAll();
  }

  @Override
  public void teleopInit() {
    mode = modeChooser.getSelected();
    stopAll();
    System.out.println("=== Enabled: " + mode + " ===");
    if (mode == Mode.SELF_TEST) {
      for (SelfTest t : selfTests) t.start();
    }
    joystickAngle = Rotation2d.kZero;
    // Ignore whatever command was left over from before this enable.
    double[] cmd = appCommand.get();
    appLastSeq = cmd.length == 4 ? cmd[0] : Double.NaN;
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
    }
  }

  @Override
  public void teleopExit() {
    stopAll();
  }

  private void joystick() {
    if (!controller.getRightBumperButton()) {
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
    double[] cmd = appCommand.get();
    double now = Timer.getFPGATimestamp();
    if (cmd.length == 4 && cmd[0] != appLastSeq) {
      appLastSeq = cmd[0];
      appFreshSince = now;
    }
    if (cmd.length != 4 || now - appFreshSince > TestLimits.kAppTimeoutSec) {
      stopAll();
      return;
    }
    driveAll(cmd[1], cmd[2], cmd[3]);
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

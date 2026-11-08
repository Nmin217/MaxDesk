package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.TestLimits;

/**
 * One MAXSwerve module (NEO drive on a SPARK MAX, NEO 550 turn on a SPARK MAX, Through Bore
 * absolute encoder plugged into the turn SPARK's data port).
 *
 * <p>2024 version: WPILib 2024 + REVLib 2024, for SPARK MAX firmware 24.x (the firmware the Pi
 * route needs). Same behaviour as firmware/module-test (2026); only the REV calls differ.
 *
 * <p>Deliberately NO restoreFactoryDefaults(): it can wipe the absolute encoder zero offset set in
 * REV Hardware Client. Every setting the module needs is set explicitly instead.
 */
public class TestModule {
  private final String name;
  private final CANSparkMax drive;
  private final CANSparkMax turn;
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;
  private final SparkPIDController turnController;

  private boolean configOk = true;
  private double stallSince = -1;
  private boolean stalled = false;
  private double turnStallSince = -1;
  private boolean turnStalled = false;

  public TestModule(String name, int driveCanId, int turnCanId) {
    this.name = name;
    drive = new CANSparkMax(driveCanId, MotorType.kBrushless);
    turn = new CANSparkMax(turnCanId, MotorType.kBrushless);

    driveEncoder = drive.getEncoder();
    turnEncoder = turn.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    turnController = turn.getPIDController();

    double metersPerMotorRev = ModuleConstants.kWheelDiameterMeters * Math.PI
        / ModuleConstants.kDrivingMotorReduction;
    configOk &= check(drive.setIdleMode(IdleMode.kBrake), "drive idle mode");
    configOk &= check(drive.setSmartCurrentLimit(TestLimits.kBenchDriveCurrentLimitAmps), "drive current limit");
    configOk &= check(driveEncoder.setPositionConversionFactor(metersPerMotorRev), "drive position factor"); // meters
    configOk &= check(driveEncoder.setVelocityConversionFactor(metersPerMotorRev / 60.0), "drive velocity factor"); // m/s

    double radiansPerRev = 2 * Math.PI;
    configOk &= check(turn.setIdleMode(IdleMode.kBrake), "turn idle mode");
    configOk &= check(turn.setSmartCurrentLimit(ModuleConstants.kTurnCurrentLimitAmps), "turn current limit");
    // The MAXSwerve output shaft turns opposite to the steering motor. Without this the position
    // loop has positive feedback and the module spins forever.
    configOk &= check(turnEncoder.setInverted(true), "turn encoder inverted");
    configOk &= check(turnEncoder.setPositionConversionFactor(radiansPerRev), "turn position factor"); // radians
    configOk &= check(turnEncoder.setVelocityConversionFactor(radiansPerRev / 60.0), "turn velocity factor"); // rad/s
    configOk &= check(turnController.setFeedbackDevice(turnEncoder), "turn feedback sensor");
    configOk &= check(turnController.setP(1), "turn P");
    configOk &= check(turnController.setI(0), "turn I");
    configOk &= check(turnController.setD(0), "turn D");
    configOk &= check(turnController.setFF(0), "turn FF");
    // Bench test: cap steering output so a wrong setting can't whip cables around.
    configOk &= check(turnController.setOutputRange(-0.5, 0.5), "turn output range");
    // Take the short way through 0 (350 deg -> 10 deg turns 20 deg, not 340).
    configOk &= check(turnController.setPositionPIDWrappingEnabled(true), "turn wrapping");
    configOk &= check(turnController.setPositionPIDWrappingMinInput(0), "turn wrapping min");
    configOk &= check(turnController.setPositionPIDWrappingMaxInput(radiansPerRev), "turn wrapping max");

    // Keep the settings through a power cycle. The zero offset set in Hardware Client is untouched.
    configOk &= check(drive.burnFlash(), "drive burn flash");
    configOk &= check(turn.burnFlash(), "turn burn flash");
    driveEncoder.setPosition(0);
  }

  private boolean check(REVLibError err, String which) {
    if (err != REVLibError.kOk) {
      System.err.printf("[%s] SPARK setting '%s' failed: %s (check CAN ID / wiring)%n",
          name, which, err);
      return false;
    }
    return true;
  }

  public String name() {
    return name;
  }

  /** False if a SPARK did not answer during configuration (wrong CAN ID, CAN break, no power). */
  public boolean configOk() {
    return configOk;
  }

  /** Module angle in radians, 0..2pi, as the absolute encoder reports it. */
  public double angleRad() {
    return turnEncoder.getPosition();
  }

  /** Angular velocity of the module in rad/s, from the absolute encoder. */
  public double angleVelocityRadPerSec() {
    return turnEncoder.getVelocity();
  }

  public double angleDeg() {
    return Math.toDegrees(angleRad());
  }

  /** Shortest signed distance from the current angle to {@code targetRad}, in degrees. */
  public double angleErrorDeg(double targetRad) {
    return Math.toDegrees(MathUtil.angleModulus(targetRad - angleRad()));
  }

  /** Closed-loop steer to an angle (radians, any range). Drive is left as is. */
  public void steerTo(double targetRad) {
    if (turnGuard()) return;
    turnController.setReference(MathUtil.inputModulus(targetRad, 0, 2 * Math.PI),
        ControlType.kPosition);
  }

  /** Open-loop steer. Only for the direction check. */
  public void steerPercent(double percent) {
    if (turnGuard()) return;
    turn.set(MathUtil.clamp(percent, -0.2, 0.2));
  }

  /**
   * Open-loop drive, always clamped to {@link TestLimits#kMaxDrivePercent}. Shuts the drive off
   * (until {@link #clearStall()}) if the wheel is jammed: see {@link TestLimits#kStallAmps}.
   */
  public void drivePercent(double percent) {
    double out = MathUtil.clamp(percent, -TestLimits.kMaxDrivePercent, TestLimits.kMaxDrivePercent);
    boolean pushing = Math.abs(out) > 0.02
        && Math.abs(driveEncoder.getVelocity()) < TestLimits.kStallSpeedMps
        && drive.getOutputCurrent() > TestLimits.kStallAmps;
    double now = Timer.getFPGATimestamp();
    if (!pushing) {
      stallSince = -1;
    } else if (stallSince < 0) {
      stallSince = now;
    } else if (!stalled && now - stallSince > TestLimits.kStallSec) {
      stalled = true;
      System.err.printf("[Module %s] DRIVE STALLED: %.0f A with the wheel not turning. Drive shut off."
          + " Disable, remove power and check the drive train turns freely by hand.%n",
          name, drive.getOutputCurrent());
    }
    if (stalled) {
      drive.stopMotor();
    } else {
      drive.set(out);
    }
  }

  /**
   * Steering stall cut-off: the turn motor is driven but the module barely turns while the NEO 550
   * draws more than {@link TestLimits#kTurnStallAmps} for {@link TestLimits#kStallSec}. That is a
   * jammed module, or a turn motor with two phase wires swapped (it buzzes instead of turning and
   * cooks within seconds). Returns true, with the turn motor stopped, once it has tripped.
   */
  private boolean turnGuard() {
    boolean pushing = Math.abs(turn.getAppliedOutput()) > 0.05
        && Math.abs(turnEncoder.getVelocity()) < TestLimits.kTurnStallRadPerSec
        && turn.getOutputCurrent() > TestLimits.kTurnStallAmps;
    double now = Timer.getFPGATimestamp();
    if (!pushing) {
      turnStallSince = -1;
    } else if (turnStallSince < 0) {
      turnStallSince = now;
    } else if (!turnStalled && now - turnStallSince > TestLimits.kStallSec) {
      turnStalled = true;
      System.err.printf("[Module %s] STEERING STALLED: %.0f A with the module not turning. Turn motor"
          + " shut off. Check the NEO 550's three phase wires match colour for colour, and that"
          + " the module turns freely by hand.%n", name, turn.getOutputCurrent());
    }
    if (turnStalled) turn.stopMotor();
    return turnStalled;
  }

  /** True once the steering has been shut off for stalling. */
  public boolean turnStalled() {
    return turnStalled;
  }

  /** True once the drive has been shut off for stalling. */
  public boolean stalled() {
    return stalled;
  }

  /** Allow driving again (each enable). */
  public void clearStall() {
    stalled = false;
    stallSince = -1;
    turnStalled = false;
    turnStallSince = -1;
  }

  /**
   * Point the wheel at {@code angle} and drive at {@code percent}, flipping the wheel instead of
   * turning more than 90 deg, and scaling drive by how well the wheel is aligned.
   */
  public void setState(double percent, Rotation2d angle) {
    // WPILib 2024 has no SwerveModuleState.optimize()/cosineScale() instance methods: done here.
    double current = angleRad();
    double target = angle.getRadians();
    if (Math.abs(MathUtil.angleModulus(target - current)) > Math.PI / 2) {   // never turn more than 90 deg
      target += Math.PI;
      percent = -percent;
    }
    percent *= Math.max(0, Math.cos(MathUtil.angleModulus(target - current)));   // less drive while still turning
    steerTo(target);
    drivePercent(percent);
  }

  public void stop() {
    drive.stopMotor();
    turn.stopMotor();
  }

  public double drivePositionMeters() {
    return driveEncoder.getPosition();
  }

  public void publish() {
    String p = name + "/";
    SmartDashboard.putBoolean(p + "ConfigOK", configOk);
    SmartDashboard.putNumber(p + "AngleDeg", angleDeg());
    SmartDashboard.putNumber(p + "DrivePosM", driveEncoder.getPosition());
    SmartDashboard.putNumber(p + "DriveVelMps", driveEncoder.getVelocity());
    SmartDashboard.putNumber(p + "DriveAmps", drive.getOutputCurrent());
    SmartDashboard.putNumber(p + "TurnAmps", turn.getOutputCurrent());
    SmartDashboard.putNumber(p + "DriveTempC", drive.getMotorTemperature());
    SmartDashboard.putNumber(p + "TurnTempC", turn.getMotorTemperature());
    SmartDashboard.putNumber(p + "BusVolts", drive.getBusVoltage());
    SmartDashboard.putBoolean(p + "DriveFault", drive.getFaults() != 0);
    SmartDashboard.putBoolean(p + "TurnFault", turn.getFaults() != 0);
    SmartDashboard.putBoolean(p + "Stalled", stalled || turnStalled);
    SmartDashboard.putBoolean(p + "TurnStalled", turnStalled);
  }
}

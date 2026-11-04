package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.TestLimits;

/**
 * One MAXSwerve module (NEO drive on a SPARK MAX, NEO 550 turn on a SPARK MAX, Through Bore
 * absolute encoder plugged into the turn SPARK's data port).
 *
 * <p>Configuration matches REV's 2026 MAXSwerve template, except the drive motor runs open loop
 * (percent output) so the bench test does not depend on velocity PID tuning.
 */
public class TestModule {
  private final String name;
  private final SparkMax drive;
  private final SparkMax turn;
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;
  private final SparkClosedLoopController turnController;

  private boolean configOk = true;
  private double stallSince = -1;
  private boolean stalled = false;
  private double turnStallSince = -1;
  private boolean turnStalled = false;

  public TestModule(String name, int driveCanId, int turnCanId) {
    this.name = name;
    drive = new SparkMax(driveCanId, MotorType.kBrushless);
    turn = new SparkMax(turnCanId, MotorType.kBrushless);

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    double metersPerMotorRev = ModuleConstants.kWheelDiameterMeters * Math.PI
        / ModuleConstants.kDrivingMotorReduction;
    driveConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(TestLimits.kBenchDriveCurrentLimitAmps);
    driveConfig.encoder
        .positionConversionFactor(metersPerMotorRev) // meters
        .velocityConversionFactor(metersPerMotorRev / 60.0); // meters per second

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    double radiansPerRev = 2 * Math.PI;
    turnConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ModuleConstants.kTurnCurrentLimitAmps);
    turnConfig.absoluteEncoder
        // The MAXSwerve output shaft turns opposite to the steering motor. Without this the
        // position loop has positive feedback and the module spins forever.
        .inverted(true)
        .positionConversionFactor(radiansPerRev) // radians
        .velocityConversionFactor(radiansPerRev / 60.0) // radians per second
        .apply(ModuleConstants.kThroughBoreEncoderV2
            ? AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2
            : AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);
    turnConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(1, 0, 0)
        // Bench test: cap steering output so a wrong setting can't whip cables around.
        .outputRange(-0.5, 0.5)
        // Take the short way through 0 (350 deg -> 10 deg turns 20 deg, not 340).
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, radiansPerRev);

    // The zero offset set with REV Hardware Client (Set Zero Offset) survives this reset.
    configOk &= check(drive.configure(driveConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters), "drive");
    configOk &= check(turn.configure(turnConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters), "turn");

    driveEncoder = drive.getEncoder();
    turnEncoder = turn.getAbsoluteEncoder();
    turnController = turn.getClosedLoopController();
    driveEncoder.setPosition(0);
  }

  private boolean check(REVLibError err, String which) {
    if (err != REVLibError.kOk) {
      System.err.printf("[%s] %s SPARK configure failed: %s (check CAN ID / wiring)%n",
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
    turnController.setSetpoint(MathUtil.inputModulus(targetRad, 0, 2 * Math.PI),
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
    SwerveModuleState state = new SwerveModuleState(percent, angle);
    Rotation2d current = new Rotation2d(angleRad());
    state.optimize(current);
    state.cosineScale(current);
    steerTo(state.angle.getRadians());
    drivePercent(state.speedMetersPerSecond);
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
    SmartDashboard.putBoolean(p + "DriveFault", drive.hasActiveFault());
    SmartDashboard.putBoolean(p + "TurnFault", turn.hasActiveFault());
    SmartDashboard.putBoolean(p + "Stalled", stalled || turnStalled);
    SmartDashboard.putBoolean(p + "TurnStalled", turnStalled);
  }
}

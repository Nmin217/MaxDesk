package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
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

/**
 * MAXSwerve module (REVLib 2025+ API). Mirrors firmware/two-module-test/TestModule.java, which is
 * the bench-tested reference; keep the two in sync.
 */
public class MAXSwerveModule {
    private final SparkMax drive, steer;
    private final AbsoluteEncoder steerEnc;
    private final SparkClosedLoopController steerPid;
    private final double offsetDeg;

    public MAXSwerveModule(int driveId, int steerId, double offsetDeg) {
        this.offsetDeg = offsetDeg;
        drive = new SparkMax(driveId, MotorType.kBrushless);
        steer = new SparkMax(steerId, MotorType.kBrushless);

        SparkMaxConfig driveCfg = new SparkMaxConfig();
        driveCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

        SparkMaxConfig steerCfg = new SparkMaxConfig();
        steerCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(20); // NEO 550
        steerCfg.absoluteEncoder
            .inverted(true) // MAXSwerve output turns opposite to the steer motor
            .positionConversionFactor(360.0)
            .velocityConversionFactor(360.0 / 60.0)
            .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);
        steerCfg.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(1.0 / 57.3, 0, 0) // same gain as REV's 1.0 per radian, in degrees
            .outputRange(-0.5, 0.5)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 360);

        drive.configure(driveCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steer.configure(steerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerEnc = steer.getAbsoluteEncoder();
        steerPid = steer.getClosedLoopController();
    }

    public void stop() { drive.stopMotor(); steer.stopMotor(); }
    public void setDriveSpeed(double s) { drive.set(MathUtil.clamp(s, -1, 1)); }

    /** Chassis-relative angle in degrees; the module's offset is added here. */
    public void setSteerAngleDeg(double deg) {
        steerPid.setSetpoint(MathUtil.inputModulus(deg + offsetDeg, 0, 360), ControlType.kPosition);
    }

    public double getSteerAngleDeg() {
        return MathUtil.inputModulus(steerEnc.getPosition() - offsetDeg, 0, 360);
    }
}

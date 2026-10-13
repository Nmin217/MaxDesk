package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

public class MAXSwerveModule {
    private final CANSparkMax drive, steer;
    private final SparkMaxAbsoluteEncoder steerEnc;

    public MAXSwerveModule(int driveId, int steerId, double offsetDeg) {
        drive = new CANSparkMax(driveId, MotorType.kBrushless);
        steer = new CANSparkMax(steerId, MotorType.kBrushless);
        drive.restoreFactoryDefaults();
        steer.restoreFactoryDefaults();
        drive.setIdleMode(CANSparkMax.IdleMode.kBrake);
        steer.setIdleMode(CANSparkMax.IdleMode.kBrake);
        steerEnc = steer.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        steerEnc.setPositionConversionFactor(360.0);
        drive.burnFlash(); steer.burnFlash();
    }

    public void stop() { drive.set(0); steer.set(0); }
    public void setDriveSpeed(double s) { drive.set(s); }
    public void setSteerAngleDeg(double deg) {
        double err = deg - steerEnc.getPosition();
        steer.set(Math.max(-0.5, Math.min(0.5, err / 180.0)));
    }
    public double getSteerAngleDeg() { return steerEnc.getPosition(); }
}

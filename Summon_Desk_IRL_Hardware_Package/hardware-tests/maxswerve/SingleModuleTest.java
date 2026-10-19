package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

/** Single MAXSwerve module test. Put module on blocks first. */
public class SingleModuleTest extends TimedRobot {
    private static final int DRIVE_CAN_ID = 1;
    private static final int STEER_CAN_ID = 2;

    private CANSparkMax drive, steer;
    private SparkMaxAbsoluteEncoder steerEnc;
    private final Timer timer = new Timer();
    private int step = 0;
    private double stepStart = 0;

    @Override public void robotInit() {
        drive = new CANSparkMax(DRIVE_CAN_ID, MotorType.kBrushless);
        steer = new CANSparkMax(STEER_CAN_ID, MotorType.kBrushless);
        drive.restoreFactoryDefaults();
        steer.restoreFactoryDefaults();
        drive.setIdleMode(CANSparkMax.IdleMode.kBrake);
        steer.setIdleMode(CANSparkMax.IdleMode.kBrake);
        steerEnc = steer.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        steerEnc.setPositionConversionFactor(360.0);
        drive.burnFlash(); steer.burnFlash();
        timer.start(); stepStart = timer.get();
        System.out.println("Module on blocks? Starting single-module test.");
    }

    @Override public void teleopPeriodic() {
        double e = timer.get() - stepStart;
        switch (step) {
            case 0: drive.set(0.10); if (e > 2) next(); break;
            case 1: drive.set(0.0);  if (e > 1) next(); break;
            case 2: drive.set(-0.10); if (e > 2) next(); break;
            case 3: drive.set(0.0);  if (e > 1) next(); break;
            case 4: setSteer(0.0);   if (e > 2) next(); break;
            case 5: setSteer(90.0);  if (e > 2) next(); break;
            case 6: setSteer(180.0); if (e > 2) next(); break;
            case 7: setSteer(270.0); if (e > 2) next(); break;
            default: drive.set(0); steer.set(0); break;
        }
    }

    private void setSteer(double deg) {
        double err = deg - steerEnc.getPosition();
        steer.set(Math.max(-0.3, Math.min(0.3, err / 180.0)));
    }

    private void next() { step++; stepStart = timer.get(); }

    @Override public void disabledInit() {
        if (drive != null) drive.set(0);
        if (steer != null) steer.set(0);
    }
}

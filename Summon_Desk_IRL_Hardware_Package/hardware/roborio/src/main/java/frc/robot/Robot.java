package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Drivetrain drivetrain;
    private NetworkTablesReceiver nt;
    private SafetyStateMachine safety;

    @Override public void robotInit() {
        drivetrain = new Drivetrain();
        nt = new NetworkTablesReceiver();
        safety = new SafetyStateMachine();
    }

    @Override public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        nt.update();
        safety.update(nt);
        if (safety.shouldStop()) drivetrain.stop();
        else drivetrain.drive(nt.getForward(), nt.getStrafe(), nt.getRotation());
    }

    @Override public void teleopInit()   { drivetrain.stop(); }
    @Override public void disabledInit() { drivetrain.stop(); }
    @Override public void testInit()     { drivetrain.stop(); }
}

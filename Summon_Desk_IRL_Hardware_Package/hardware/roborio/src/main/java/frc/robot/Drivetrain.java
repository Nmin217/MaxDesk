package frc.robot;

/** Starter drivetrain. Stops until swerve kinematics are implemented. */
public class Drivetrain {
    private final MAXSwerveModule fl, fr, rl, rr;

    public Drivetrain() {
        fl = new MAXSwerveModule(Constants.FRONT_LEFT_DRIVE_CAN,
                                 Constants.FRONT_LEFT_STEER_CAN,
                                 Constants.FRONT_LEFT_ENCODER_OFFSET_DEG);
        fr = new MAXSwerveModule(Constants.FRONT_RIGHT_DRIVE_CAN,
                                 Constants.FRONT_RIGHT_STEER_CAN,
                                 Constants.FRONT_RIGHT_ENCODER_OFFSET_DEG);
        rl = new MAXSwerveModule(Constants.REAR_LEFT_DRIVE_CAN,
                                 Constants.REAR_LEFT_STEER_CAN,
                                 Constants.REAR_LEFT_ENCODER_OFFSET_DEG);
        rr = new MAXSwerveModule(Constants.REAR_RIGHT_DRIVE_CAN,
                                 Constants.REAR_RIGHT_STEER_CAN,
                                 Constants.REAR_RIGHT_ENCODER_OFFSET_DEG);
    }

    // TODO: implement swerve kinematics for your geometry.
    public void drive(double forward, double strafe, double rotation) { stop(); }

    public void stop() { fl.stop(); fr.stop(); rl.stop(); rr.stop(); }
}

package frc.robot.summon.core;

/** What the planner/controller needs from the swerve drive. Implemented by the team's drive subsystem. */
public interface DriveIO {
    /** Odometry pose, field frame: {x (m), y (m), theta (rad)}. */
    double[] pose();
    /** Field-relative velocity command: m/s, m/s, rad/s. */
    void drive(double vx, double vy, double omega);
    /** Current field-relative velocity: {vx, vy, omega}. */
    double[] velocity();
}

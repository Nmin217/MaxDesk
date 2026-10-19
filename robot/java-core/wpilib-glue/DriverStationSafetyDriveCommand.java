package frc.robot.summon;

// WPILib integration example. Requires a real WPILib robot project and the team's swerve subsystem.
// This file is intentionally kept in wpilib-glue/ and is NOT compiled by the dependency-free Java-core tests.

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.summon.core.Config;
import frc.robot.summon.core.SafetySupervisor;

/**
 * Driver Station joystick drive with a conservative speed cap and continuously active lidar supervision.
 * The override input only raises the speed cap; it NEVER disables lidar checks or stale-sensor stopping.
 *
 * Contract:
 * - x/y joystick suppliers are robot-relative, normalized (-1..1), and sign-corrected by RobotContainer.
 * - pose and velocity are current odometry and field-relative chassis velocity.
 * - LidarFrame points are already transformed/fused into FIELD coordinates (meters) by the lidar coprocessor.
 * - SafeDriveOutput must apply a controlled stop / brake when safetyHold is true; it must not bypass the physical E-stop.
 */
public final class DriverStationSafetyDriveCommand extends CommandBase {
    public static final double DEFAULT_MAX_TRANSLATION_MPS = 0.45;
    public static final double OVERRIDE_MAX_TRANSLATION_MPS = 0.90;
    public static final double DEFAULT_MAX_OMEGA_RADPS = 0.60;
    public static final double OVERRIDE_MAX_OMEGA_RADPS = 1.00;
    public static final double MAX_LIDAR_AGE_SECONDS = 0.15;

    public static final class LidarFrame {
        public final double timestampSeconds;
        public final List<double[]> fieldHits;
        public final boolean valid;

        public LidarFrame(double timestampSeconds, List<double[]> fieldHits, boolean valid) {
            this.timestampSeconds = timestampSeconds;
            this.fieldHits = fieldHits;
            this.valid = valid;
        }
    }

    @FunctionalInterface
    public interface SafeDriveOutput {
        /** Speeds are field-relative. safetyHold=true asks the drivetrain to use its configured controlled-braking behavior. */
        void apply(ChassisSpeeds fieldRelativeSpeeds, boolean safetyHold);
    }

    private final DoubleSupplier forward, strafe, rotate;
    private final BooleanSupplier speedOverrideHeld;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> fieldVelocitySupplier;
    private final Supplier<LidarFrame> lidarSupplier;
    private final SafeDriveOutput output;
    private final Config config;
    private final SafetySupervisor supervisor;

    public DriverStationSafetyDriveCommand(
            Subsystem drivetrainRequirement,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier rotate,
            BooleanSupplier speedOverrideHeld,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldVelocitySupplier,
            Supplier<LidarFrame> lidarSupplier,
            SafeDriveOutput output) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;
        this.speedOverrideHeld = speedOverrideHeld;
        this.poseSupplier = poseSupplier;
        this.fieldVelocitySupplier = fieldVelocitySupplier;
        this.lidarSupplier = lidarSupplier;
        this.output = output;
        this.config = new Config();
        this.supervisor = new SafetySupervisor(config);
        addRequirements(drivetrainRequirement);
    }

    @Override
    public void execute() {
        final double now = Timer.getFPGATimestamp();
        final LidarFrame frame = lidarSupplier.get();
        // Fail closed if lidar is missing, invalid, stale, or has a timestamp unexpectedly in the future.
        if (frame == null || !frame.valid || frame.fieldHits == null
                || !Double.isFinite(frame.timestampSeconds)
                || now - frame.timestampSeconds < -0.02
                || now - frame.timestampSeconds > MAX_LIDAR_AGE_SECONDS) {
            output.apply(new ChassisSpeeds(), true);
            SmartDashboard.putBoolean("SummonDesk/LidarFresh", false);
            SmartDashboard.putBoolean("SummonDesk/SafetyHold", true);
            SmartDashboard.putBoolean("SummonDesk/SpeedOverrideActive", false);
            return;
        }

        final Pose2d pose = poseSupplier.get();
        final ChassisSpeeds measured = fieldVelocitySupplier.get();
        final boolean override = speedOverrideHeld.getAsBoolean();
        final double maxV = override ? OVERRIDE_MAX_TRANSLATION_MPS : DEFAULT_MAX_TRANSLATION_MPS;
        final double maxW = override ? OVERRIDE_MAX_OMEGA_RADPS : DEFAULT_MAX_OMEGA_RADPS;

        double f = clamp(forward.getAsDouble());
        double s = clamp(strafe.getAsDouble());
        double r = clamp(rotate.getAsDouble());
        double robotVx = f * maxV;
        double robotVy = s * maxV;
        double mag = Math.hypot(robotVx, robotVy);
        if (mag > maxV) { robotVx *= maxV / mag; robotVy *= maxV / mag; }

        ChassisSpeeds fieldCommand = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotVx, robotVy, r * maxW, pose.getRotation());

        // Always run the lidar supervisor, including while the operator holds the speed override.
        SafetySupervisor.Result safe = supervisor.supervise(
                fieldCommand.vxMetersPerSecond,
                fieldCommand.vyMetersPerSecond,
                fieldCommand.omegaRadiansPerSecond,
                pose.getX(), pose.getY(), pose.getRotation().getRadians(),
                measured.vxMetersPerSecond, measured.vyMetersPerSecond,
                frame.fieldHits);

        output.apply(new ChassisSpeeds(safe.vx, safe.vy, safe.w), safe.held);
        SmartDashboard.putBoolean("SummonDesk/LidarFresh", true);
        SmartDashboard.putBoolean("SummonDesk/SafetyHold", safe.held);
        SmartDashboard.putBoolean("SummonDesk/SpeedOverrideActive", override);
        SmartDashboard.putNumber("SummonDesk/TeleopSpeedCapMps", maxV);
    }

    @Override
    public void end(boolean interrupted) {
        output.apply(new ChassisSpeeds(), true);
        SmartDashboard.putBoolean("SummonDesk/SpeedOverrideActive", false);
    }

    private static double clamp(double v) {
        if (!Double.isFinite(v)) return 0.0;
        return Math.max(-1.0, Math.min(1.0, v));
    }
}

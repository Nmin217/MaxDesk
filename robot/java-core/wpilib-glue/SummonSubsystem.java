package frc.robot.summon;

// NOT COMPILED OR TESTED in this project: it needs WPILib, which was not available offline.
// Copy src/main/java/frc/robot/summon/core/* into your robot project, then add this file and adjust.
// The follower (pure pursuit + yaw gating + stuck recovery) and the state machine are NOT ported yet:
// see TODOs. What IS ported and tested: OrientationPlanner, SafetySupervisor, OccupancyMemory.

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.summon.core.Config;
import frc.robot.summon.core.OccupancyMemory;
import frc.robot.summon.core.OrientationPlanner;
import frc.robot.summon.core.PowerIO;
import frc.robot.summon.core.SafetySupervisor;
import frc.robot.summon.core.Waypoint;

public class SummonSubsystem extends SubsystemBase {
    /** One lidar sweep in the FIELD frame, already split into hits and free-space rays (the coprocessor does this). */
    public interface ScanSource {
        List<OccupancyMemory.Hit> hits();
        List<OccupancyMemory.Ray> rays();
    }

    private final Config cfg = new Config();
    private final OccupancyMemory memory = new OccupancyMemory(cfg);
    private final OrientationPlanner planner = new OrientationPlanner(cfg);
    private final SafetySupervisor supervisor = new SafetySupervisor(cfg);

    private final Supplier<Pose2d> pose;                // swerve odometry
    private final Supplier<ChassisSpeeds> velocity;     // field-relative
    private final Consumer<ChassisSpeeds> drive;        // field-relative command
    private final ScanSource scan;
    private final PowerIO power;

    private List<Waypoint> path = new ArrayList<>();
    private double lastPlan = -1;
    private boolean summoned = false;
    private double goalX, goalY;

    public SummonSubsystem(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> velocity, Consumer<ChassisSpeeds> drive,
                           ScanSource scan, PowerIO power) {
        this.pose = pose; this.velocity = velocity; this.drive = drive; this.scan = scan; this.power = power;
    }

    /** Call from a command or the phone interface. Refuses while the charging cord may still be connected. */
    public boolean summon(double userX, double userY) {
        if (power.cordPluggedIn()) return false;        // interlock layer 1: do not drive with a cord attached
        goalX = userX; goalY = userY; summoned = true; lastPlan = -1;
        return true;
    }
    public void cancel() { summoned = false; drive.accept(new ChassisSpeeds()); }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        Pose2d p = pose.get();
        memory.update(now, scan.hits(), scan.rays());

        // Charging-cord lock, checked EVERY loop (not just at summon): while any signal says the cord may be plugged in,
        // the wheels get no command and any trip is cancelled, even one already under way. This is the software layer;
        // the hardware layer is an inlet-empty switch in series with the drive contactor coil (see the proposal).
        if (power.cordPluggedIn()) {
            summoned = false;
            drive.accept(new ChassisSpeeds());
            return;
        }
        if (!summoned) return;

        if (lastPlan < 0 || now - lastPlan > 0.2) {      // replan about 5 times a second
            boolean[] occ = memory.buildOccupancy(now, new ArrayList<>(), false, p.getX(), p.getY());   // TODO: feed tracked people
            path = planner.plan(p.getX(), p.getY(), p.getRotation().getRadians(),
                    OrientationPlanner.GoalType.USER, goalX, goalY, occ, memory.risk());
            lastPlan = now;
        }

        // TODO (not ported yet): follow `path` with pure pursuit, steering yaw toward path[i].th, slowing when yaw
        // lags the plan, and run the stuck-recovery sequence. Placeholder command below is zero.
        double cx = 0, cy = 0, w = 0;

        ChassisSpeeds v = velocity.get();
        List<double[]> pts = new ArrayList<>();
        for (OccupancyMemory.Hit h : scan.hits()) pts.add(new double[] {h.x, h.y});
        SafetySupervisor.Result r = supervisor.supervise(cx, cy, w, p.getX(), p.getY(), p.getRotation().getRadians(),
                v.vxMetersPerSecond, v.vyMetersPerSecond, pts);
        // r.held => brake at cfg.emergencyDecel (slew limiter belongs in the drive subsystem)
        drive.accept(new ChassisSpeeds(r.vx, r.vy, r.w));
    }
}

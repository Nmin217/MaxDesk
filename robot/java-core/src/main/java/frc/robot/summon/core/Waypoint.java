package frc.robot.summon.core;

/** One planned state: where, and which way the desk's long axis points (k * 22.5 deg). */
public final class Waypoint {
    public final double x, y, th;
    public final int k;
    public Waypoint(double x, double y, int k) { this.x = x; this.y = y; this.k = k; this.th = k * (Math.PI / Config.NKF); }
    @Override public String toString() { return String.format("(%.2f,%.2f,k%d)", x, y, k); }
}

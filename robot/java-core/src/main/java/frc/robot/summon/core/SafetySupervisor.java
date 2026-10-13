package frc.robot.summon.core;

import java.util.List;

/**
 * Last stage before the wheels. Uses only what the lidar can see.
 * Translation: sweep the desk's real rectangle along the commanded AND the actual velocity and cap speed so the
 * desk can always stop (at safeDecel, which is below its real braking) before contact.
 * Rotation: refuse yaw that would swing a corner into a lidar point.
 * When it intervenes, `held` is true and the caller should brake at emergencyDecel.
 */
public final class SafetySupervisor {
    public static final class Result {
        public double vx, vy, w; public boolean held;
        Result(double vx, double vy, double w, boolean held) { this.vx = vx; this.vy = vy; this.w = w; this.held = held; }
    }
    private final Config cfg;
    public SafetySupervisor(Config cfg) { this.cfg = cfg; }

    /** hits: lidar hit points in world coordinates (meters). */
    public Result supervise(double cx, double cy, double w, double x, double y, double th, double vx, double vy, List<double[]> hits) {
        double co = Math.cos(th), si = Math.sin(th);
        double r2 = cfg.supervisorRange * cfg.supervisorRange;
        java.util.ArrayList<double[]> pts = new java.util.ArrayList<>();
        for (double[] h : hits) {
            double dx = h[0] - x, dy = h[1] - y;
            if (dx * dx + dy * dy > r2) continue;
            pts.add(new double[] {dx, dy, co * dx + si * dy, -si * dx + co * dy});
        }
        boolean held = false;
        double v = Math.hypot(cx, cy), va = Math.hypot(vx, vy);
        if (v > 0.02 || va > 0.06) {
            double m = cfg.safeMargin + Math.min(0.16, Math.abs(w) * 0.2);
            if (v > 0.02) {
                double f = sweepFree(pts, cx / v, cy / v, m, th);
                double vAllow = Math.sqrt(2 * cfg.safeDecel * Math.max(0, f - cfg.stopGap));
                if (vAllow < v) { double k = vAllow / v; cx *= k; cy *= k; if (k < 0.05) held = true; }
            }
            if (va > 0.06) {   // still coasting: brake if the path along the REAL velocity is short
                double f = sweepFree(pts, vx / va, vy / va, m, th);
                double vAllow = Math.sqrt(2 * cfg.safeDecel * Math.max(0, f - cfg.stopGap));
                if (va > vAllow) { cx = 0; cy = 0; held = true; }
            }
        }
        if (Math.abs(w) > 0.02) {
            double hx = cfg.hw + cfg.yawMargin, hy = cfg.hh + cfg.yawMargin, dir = Math.signum(w), look = 0.06 + Math.abs(w) * 0.3;
            double p0 = pen(pts, th, 0, hx, hy), worst = p0;
            for (int k = 1; k <= 4; k++) worst = Math.max(worst, pen(pts, th, dir * look * k / 4, hx, hy));
            if (worst > 0 && worst > p0 + 0.01) { w = 0; held = true; }
        }
        return new Result(cx, cy, w, held);
    }

    private double pen(List<double[]> pts, double th, double a, double hx, double hy) {
        double c = Math.cos(th + a), s = Math.sin(th + a), b = 0;
        for (double[] p : pts) {
            double qx = c * p[0] + s * p[1], qy = -s * p[0] + c * p[1];
            double d = Math.min(hx - Math.abs(qx), hy - Math.abs(qy));
            if (d > b) b = d;
        }
        return b;
    }

    /** Distance the footprint can travel along world direction (ux, uy) before touching a point. */
    private double sweepFree(List<double[]> pts, double ux, double uy, double m, double th) {
        double co = Math.cos(th), si = Math.sin(th);
        double udx = co * ux + si * uy, udy = -si * ux + co * uy, hx = cfg.hw + m, hy = cfg.hh + m, free = 1e18;
        for (double[] p : pts) {
            double qx = p[2], qy = p[3];
            if (p[0] * ux + p[1] * uy <= 0 && Math.abs(qx) <= hx && Math.abs(qy) <= hy) continue;  // touching but moving away: allowed
            double t0 = 0, t1 = 1e18; boolean ok = true;
            double[][] axes = {{qx, udx, hx}, {qy, udy, hy}};
            for (double[] ax : axes) {
                double q = ax[0], u = ax[1], hh = ax[2];
                if (Math.abs(u) < 1e-9) { if (Math.abs(q) > hh) { ok = false; break; } }
                else {
                    double a = (q - hh) / u, b = (q + hh) / u;
                    if (a > b) { double t = a; a = b; b = t; }
                    t0 = Math.max(t0, a); t1 = Math.min(t1, b);
                }
            }
            if (ok && t0 <= t1 && t1 >= 0) free = Math.min(free, Math.max(0, t0));
        }
        return free;
    }
}

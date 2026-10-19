package frc.robot.summon.core;

/** Small geometry helpers. */
public final class Geometry {
    private Geometry() {}

    /** Wrap to (-pi, pi]. */
    public static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    /** a - b, wrapped. */
    public static double ang(double a, double b) { return wrap(a - b); }

    /** Corners of the robot rectangle at a pose. */
    public static double[][] footprint(double x, double y, double th, double hw, double hh) {
        double c = Math.cos(th), s = Math.sin(th);
        double[][] loc = {{-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh}};
        double[][] out = new double[4][2];
        for (int i = 0; i < 4; i++) {
            out[i][0] = x + c * loc[i][0] - s * loc[i][1];
            out[i][1] = y + s * loc[i][0] + c * loc[i][1];
        }
        return out;
    }

    /** Separating-axis overlap test for two convex polygons. */
    public static boolean overlap(double[][] a, double[][] b) {
        for (double[][] p : new double[][][] {a, b}) {
            for (int i = 0; i < p.length; i++) {
                double[] u = p[i], v = p[(i + 1) % p.length];
                double nx = -(v[1] - u[1]), ny = v[0] - u[0];
                double a0 = 1e18, a1 = -1e18, b0 = 1e18, b1 = -1e18;
                for (double[] q : a) { double d = q[0] * nx + q[1] * ny; a0 = Math.min(a0, d); a1 = Math.max(a1, d); }
                for (double[] q : b) { double d = q[0] * nx + q[1] * ny; b0 = Math.min(b0, d); b1 = Math.max(b1, d); }
                if (a1 < b0 || b1 < a0) return false;
            }
        }
        return true;
    }

    public static double[][] rect(double x, double y, double w, double h) {
        return new double[][] {{x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}};
    }
}

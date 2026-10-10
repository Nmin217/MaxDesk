package frc.robot.summon.core;

import java.util.List;

/**
 * Long-term map memory.
 *  - staticMem: obstacles seen steadily for persistSeconds; kept until a later lidar ray passes through the cell.
 *  - seen:      cells the lidar has ever covered (unknown space is slightly penalised by the planner).
 *  - traffic:   where people tend to be (a small planning cost, never stored as furniture).
 *  - last/since: short-term hit memory used to decide what counts as "steady".
 * Persists between trips; serialize()/deserialize() let it survive a restart.
 */
public final class OccupancyMemory {
    public enum HitType { STATIC, PERSON, USER }

    public static final class Hit {
        public final double x, y; public final HitType type;
        public Hit(double x, double y, HitType type) { this.x = x; this.y = y; this.type = type; }
    }
    /** A lidar ray: free space from d0 to d1 along (cos, sin) from (ox, oy). */
    public static final class Ray {
        public final double ox, oy, cos, sin, d0, d1;
        public Ray(double ox, double oy, double cos, double sin, double d0, double d1) {
            this.ox = ox; this.oy = oy; this.cos = cos; this.sin = sin; this.d0 = d0; this.d1 = d1;
        }
    }
    public static final class Person {
        public final double x, y, r; public final boolean settled;
        public Person(double x, double y, double r, boolean settled) { this.x = x; this.y = y; this.r = r; this.settled = settled; }
    }

    private final Config cfg;
    public final int gw, gh, n;
    public final double[] last, since;
    public final boolean[] staticMem, seen;
    public final float[] traffic;
    private boolean changed = false;
    private int calls = 0;

    public OccupancyMemory(Config cfg) {
        this.cfg = cfg; gw = cfg.gw; gh = cfg.gh; n = gw * gh;
        last = new double[n]; since = new double[n];
        java.util.Arrays.fill(last, -99); java.util.Arrays.fill(since, -99);
        staticMem = new boolean[n]; seen = new boolean[n]; traffic = new float[n];
    }

    private int cellX(double x) { return (int) (x / cfg.cell); }
    private int cellY(double y) { return (int) (y / cfg.cell); }
    private boolean in(int x, int y) { return x >= 0 && x < gw && y >= 0 && y < gh; }

    public void clear() {
        java.util.Arrays.fill(staticMem, false); java.util.Arrays.fill(seen, false); java.util.Arrays.fill(traffic, 0f);
        changed = false;
    }
    /** True once since the last call if anything worth saving changed. */
    public boolean consumeChanged() { boolean c = changed; changed = false; return c; }

    /** Feed one lidar scan. */
    public void update(double now, List<Hit> hits, List<Ray> rays) {
        for (Hit h : hits) {                       // short-term memory of every hit
            int gx = cellX(h.x), gy = cellY(h.y);
            if (!in(gx, gy)) continue;
            int k = gy * gw + gx;
            if (now - last[k] > 0.4) since[k] = now;
            last[k] = now;
        }
        for (Ray r : rays)                         // free space: clear stale statics, mark coverage
            for (double d = r.d0; d < r.d1; d += cfg.rayStep) {
                int gx = cellX(r.ox + r.cos * d), gy = cellY(r.oy + r.sin * d);
                if (!in(gx, gy)) continue;
                int k = gy * gw + gx;
                if (staticMem[k]) { staticMem[k] = false; changed = true; }
                if (!seen[k]) { seen[k] = true; changed = true; }
            }
        for (Hit h : hits) {
            int gx = cellX(h.x), gy = cellY(h.y);
            if (!in(gx, gy)) continue;
            int k = gy * gw + gx;
            if (!seen[k]) { seen[k] = true; changed = true; }
            if (h.type == HitType.PERSON) { traffic[k] = Math.min(40f, traffic[k] + 0.01f); continue; }  // never furniture
            if (h.type == HitType.USER) continue;
            if (now - since[k] >= cfg.persistSeconds) { if (!staticMem[k]) changed = true; staticMem[k] = true; }
        }
        if ((++calls) % 60 == 0) for (int i = 0; i < n; i++) if (traffic[i] > 0) traffic[i] *= 0.9982f;  // fade, ~9 min
    }

    /**
     * What the planner treats as occupied right now.
     * Moving people are waited for (their own cells are removed); settled people are obstacles with a comfort margin.
     * patient=true treats nearby moving people as settled (used when someone has held the desk up too long).
     */
    public boolean[] buildOccupancy(double now, List<Person> people, boolean patient, double robotX, double robotY) {
        boolean[] occ = new boolean[n];
        for (int i = 0; i < n; i++)
            if ((now - last[i] < cfg.shortMemory && last[i] - since[i] >= cfg.persistSeconds) || staticMem[i]) occ[i] = true;
        double patientRange = 3.0;
        for (Person p : people) {
            if (p.settled || (patient && Math.hypot(p.x - robotX, p.y - robotY) < patientRange)) continue;
            stamp(occ, p.x, p.y, p.r + 0.06, false);
        }
        for (Person p : people) {
            if (!(p.settled || (patient && Math.hypot(p.x - robotX, p.y - robotY) < patientRange))) continue;
            stamp(occ, p.x, p.y, p.r + 0.16, true);
        }
        return occ;
    }

    private void stamp(boolean[] occ, double px, double py, double rr, boolean value) {
        int cx = cellX(px), cy = cellY(py), r = (int) Math.ceil(rr / cfg.cell);
        for (int dy = -r; dy <= r; dy++)
            for (int dx = -r; dx <= r; dx++) {
                if (Math.hypot(dx * cfg.cell, dy * cfg.cell) > rr) continue;
                int x = cx + dx, y = cy + dy;
                if (in(x, y)) occ[y * gw + x] = value;
            }
    }

    /** Per-cell extra cost: unseen space and busy places. */
    public float[] risk() {
        float[] r = new float[n];
        for (int i = 0; i < n; i++)
            r[i] = (float) ((seen[i] ? 0 : cfg.unknownPen) + Math.min(cfg.trafficMax, traffic[i] * cfg.trafficK));
        return r;
    }

    public byte[] serialize() {
        byte[] a = new byte[3 * n];
        for (int i = 0; i < n; i++) {
            a[i] = (byte) (staticMem[i] ? 1 : 0);
            a[n + i] = (byte) (seen[i] ? 1 : 0);
            a[2 * n + i] = (byte) Math.min(255, Math.round(traffic[i] * 6));
        }
        return a;
    }
    public boolean deserialize(byte[] a) {
        if (a == null || a.length != 3 * n) return false;
        for (int i = 0; i < n; i++) {
            staticMem[i] = a[i] != 0; seen[i] = a[n + i] != 0; traffic[i] = (a[2 * n + i] & 0xFF) / 6f;
        }
        return true;
    }
    public int countStatic() { int c = 0; for (boolean b : staticMem) if (b) c++; return c; }
    public int countSeen() { int c = 0; for (boolean b : seen) if (b) c++; return c; }
}

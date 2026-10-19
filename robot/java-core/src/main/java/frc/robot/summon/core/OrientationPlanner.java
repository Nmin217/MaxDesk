package frc.robot.summon.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Orientation-aware A*. State = (cell, orientation). The desk is a rectangle, so what fits through a gap depends on
 * which way it faces. Orientation k means the long axis points at k * 22.5 deg (full 360 deg: which side faces you
 * matters at the end). Each of 8 distinct footprints gets its own blocked grid (every obstacle cell grown by the
 * rotated rectangle plus a margin). Moves go in 8 directions and may turn the desk one step while moving, but only
 * in "roomy" cells; pure in-place turns only where the desk could turn to ANY angle with margin.
 */
public final class OrientationPlanner {
    public enum GoalType { USER, HOME, POINT }

    private static final int[][] DIRS = {{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
    private static final double DK = Math.PI / Config.NKF;

    private final Config cfg;
    private final int gw, gh, nc, ns;
    private final double[][] stepc = new double[Config.NK][8];
    private final int[][][] kernStrict, kernTight;
    private final int[][] roomOffs, rotOffs;
    private final double roomyR, rotR;
    private final float[] G; private final int[] P; private final boolean[] closed;
    private final float[] hf; private final int[] hi; private int hn;
    /** True if the last plan only fit with the tight margin. */
    public boolean squeeze = false;

    public OrientationPlanner(Config cfg) {
        this.cfg = cfg; gw = cfg.gw; gh = cfg.gh; nc = gw * gh; ns = nc * Config.NK;
        for (int k = 0; k < Config.NK; k++)
            for (int d = 0; d < 8; d++) {
                double dx = DIRS[d][0], dy = DIRS[d][1];
                stepc[k][d] = Math.hypot(dx, dy) * (1 + cfg.alignPen * Math.abs(Math.sin(Math.atan2(dy, dx) - k * DK)));
            }
        kernStrict = makeKernels(cfg.mgStrict); kernTight = makeKernels(cfg.mgTight);
        double circ = Math.hypot(cfg.hw, cfg.hh);
        roomyR = circ + cfg.roomyMargin; rotR = circ + cfg.rotMargin;
        roomOffs = offsets(roomyR); rotOffs = offsets(rotR);
        G = new float[ns]; P = new int[ns]; closed = new boolean[ns]; hf = new float[ns * 10]; hi = new int[ns * 10];
    }

    // ---------------- grids ----------------
    private int[][][] makeKernels(double mg) {
        int[][][] out = new int[Config.NKF][][];
        double cs = cfg.cell, hc = cs / 2;
        for (int k = 0; k < Config.NKF; k++) {
            double c = Math.cos(k * DK), s = Math.sin(k * DK), ac = Math.abs(c), as = Math.abs(s);
            double ex = (cfg.hw + mg) * ac + (cfg.hh + mg) * as + hc, ey = (cfg.hw + mg) * as + (cfg.hh + mg) * ac + hc;
            double hu = cfg.hw + mg + hc * (ac + as), hv = cfg.hh + mg + hc * (ac + as);
            int R = (int) Math.ceil(Math.max(ex, ey) / cs) + 1;
            ArrayList<int[]> list = new ArrayList<>();
            for (int b = -R; b <= R; b++)
                for (int a = -R; a <= R; a++) {
                    double dx = a * cs, dy = b * cs;   // obstacle cell (a square) vs rotated rectangle: exact 4-axis SAT
                    if (Math.abs(dx) > ex + 1e-9 || Math.abs(dy) > ey + 1e-9) continue;   // ties count as inside (conservative)
                    if (Math.abs(dx * c + dy * s) > hu + 1e-9 || Math.abs(-dx * s + dy * c) > hv + 1e-9) continue;
                    list.add(new int[] {a, b});
                }
            out[k] = list.toArray(new int[0][]);
        }
        return out;
    }

    private int[][] offsets(double rad) {
        ArrayList<int[]> l = new ArrayList<>();
        for (int a = -8; a <= 8; a++) for (int b = -8; b <= 8; b++)
            if (Math.hypot(a * cfg.cell, b * cfg.cell) <= rad + cfg.cell / 2 + 1e-9) l.add(new int[] {a, b});
        return l.toArray(new int[0][]);
    }

    private boolean[][] buildBlocked(boolean[] occ, int[][][] kern, double mg, double rx, double ry) {
        boolean[][] B = new boolean[Config.NKF][nc];
        for (int i = 0; i < nc; i++) {
            if (!occ[i]) continue;
            int cx = i % gw, cy = i / gw;
            for (int k = 0; k < Config.NKF; k++) {
                boolean[] bk = B[k];
                for (int[] o : kern[k]) { int x = cx + o[0], y = cy + o[1]; if (x >= 0 && x < gw && y >= 0 && y < gh) bk[y * gw + x] = true; }
            }
        }
        for (int k = 0; k < Config.NKF; k++) {   // map edges, using the rotated half-extents
            double ac = Math.abs(Math.cos(k * DK)), as = Math.abs(Math.sin(k * DK));
            double ex = cfg.hw * ac + cfg.hh * as + mg, ey = cfg.hw * as + cfg.hh * ac + mg;
            for (int y = 0; y < gh; y++) for (int x = 0; x < gw; x++) {
                double px = (x + .5) * cfg.cell, py = (y + .5) * cfg.cell;
                if (px < ex - 1e-9 || px > cfg.mapW - ex + 1e-9 || py < ey - 1e-9 || py > cfg.mapH - ey + 1e-9) B[k][y * gw + x] = true;
            }
        }
        int sx = (int) (rx / cfg.cell), sy = (int) (ry / cfg.cell);   // never block the robot's own surroundings
        for (int k = 0; k < Config.NKF; k++)
            for (int a = -1; a <= 1; a++) for (int b = -1; b <= 1; b++) {
                int x = sx + a, y = sy + b;
                if (x >= 0 && x < gw && y >= 0 && y < gh) B[k][y * gw + x] = false;
            }
        return B;
    }

    private boolean[] buildRoomy(boolean[] occ, int[][] offs, double rad) {
        boolean[] R = new boolean[nc];
        for (int i = 0; i < nc; i++) {
            if (!occ[i]) continue;
            int cx = i % gw, cy = i / gw;
            for (int[] o : offs) { int x = cx + o[0], y = cy + o[1]; if (x >= 0 && x < gw && y >= 0 && y < gh) R[y * gw + x] = true; }
        }
        for (int y = 0; y < gh; y++) for (int x = 0; x < gw; x++) {
            double px = (x + .5) * cfg.cell, py = (y + .5) * cfg.cell;
            if (px < rad - 1e-9 || px > cfg.mapW - rad + 1e-9 || py < rad - 1e-9 || py > cfg.mapH - rad + 1e-9) R[y * gw + x] = true;
        }
        return R;
    }

    // ---------------- heap ----------------
    private void push(float f, int i) {
        int k = hn++;
        while (k > 0) { int p = (k - 1) >> 1; if (hf[p] <= f) break; hf[k] = hf[p]; hi[k] = hi[p]; k = p; }
        hf[k] = f; hi[k] = i;
    }
    private int pop() {
        int ri = hi[0]; float lf = hf[--hn]; int li = hi[hn];
        if (hn > 0) {
            int k = 0;
            for (;;) {
                int a = 2 * k + 1; if (a >= hn) break;
                int m = a; if (a + 1 < hn && hf[a + 1] < hf[a]) m = a + 1;
                if (hf[m] >= lf) break;
                hf[k] = hf[m]; hi[k] = hi[m]; k = m;
            }
            hf[k] = lf; hi[k] = li;
        }
        return ri;
    }

    private static int roundHalfUp(double v) { return (int) Math.floor(v + 0.5); }

    // ---------------- the search ----------------
    private List<Waypoint> search(boolean[][] B, boolean[][] Bt, boolean[] roomy, boolean[] rotOk, float[] risk,
                                  double x0, double y0, double th0, GoalType type, double gx, double gy) {
        Arrays.fill(G, 1e9f); Arrays.fill(P, -1); Arrays.fill(closed, false); hn = 0;
        boolean[] ring = new boolean[nc]; int[] kPk = new int[nc]; Arrays.fill(kPk, -1);
        float[] gpen = new float[nc], hfa = new float[nc];
        boolean point = type != GoalType.USER;
        for (int c = 0; c < nc; c++)
            hfa[c] = (float) (cfg.heurWeight * Math.max(0, (Math.hypot((c % gw + .5) * cfg.cell - gx, (c / gw + .5) * cfg.cell - gy)
                    - (point ? 0 : cfg.heurRingRadius)) / cfg.cell));
        if (point) {
            int hc = Math.max(0, Math.min(gh - 1, (int) (gy / cfg.cell))) * gw + Math.max(0, Math.min(gw - 1, (int) (gx / cfg.cell)));
            ring[hc] = true; kPk[hc] = type == GoalType.POINT ? -2 : 0;
        } else {
            for (int c = 0; c < nc; c++) {
                double cx = (c % gw + .5) * cfg.cell, cy = (c / gw + .5) * cfg.cell, d = Math.hypot(cx - gx, cy - gy);
                if (Math.abs(d - cfg.ringRadius) < cfg.ringTol - 1e-9) {
                    ring[c] = true;
                    gpen[c] = (float) (Math.abs(cx - gx) < .7 * Math.abs(cy - gy) - 1e-9 ? 0 : cfg.sectorPen);
                    double u = Math.atan2(gy - cy, gx - cx), thP = u - Math.PI / 2, thH = Math.sin(u) > 0 ? 0 : Math.PI;
                    double t = Math.abs(Geometry.ang(thP, thH)) <= .65 ? thH : thP;
                    kPk[c] = ((roundHalfUp(t / DK) % Config.NK) + Config.NK) % Config.NK;   // face the user with the keyboard side
                }
            }
        }
        int sx = Math.max(0, Math.min(gw - 1, (int) (x0 / cfg.cell))), sy = Math.max(0, Math.min(gh - 1, (int) (y0 / cfg.cell)));
        int k0 = ((roundHalfUp(th0 / DK) % Config.NK) + Config.NK) % Config.NK;
        int s0 = k0 * nc + sy * gw + sx; G[s0] = 0; push(hfa[s0 % nc], s0);
        int goalS = -1, pops = 0, bestS = -1; double bestC = 1e18;
        while (hn > 0) {
            int e = pop();
            if (e < 0) { goalS = -e - 1; break; }
            int s = e; if (closed[s]) continue; closed[s] = true;
            if (++pops > cfg.maxPops) break;
            int c = s % nc, k = s / nc;
            if (ring[c] && (kPk[c] == -2 || k == kPk[c])) {
                double tc = G[s] + gpen[c]; if (tc < bestC) { bestC = tc; bestS = s; }
                push((float) tc, -(s + 1)); continue;
            }
            int x = c % gw, y = c / gw; boolean[] bk = B[k & 7];
            for (int d = 0; d < 8; d++) {
                int dx = DIRS[d][0], dy = DIRS[d][1], nx = x + dx, ny = y + dy;
                if (nx < 0 || ny < 0 || nx >= gw || ny >= gh) continue;
                int ncell = ny * gw + nx;
                for (int dk = -1; dk <= 1; dk++) {
                    int nk = (k + dk + Config.NK) % Config.NK;
                    boolean gn = ring[ncell] && (kPk[ncell] == -2 || nk == kPk[ncell]) && !Bt[nk & 7][ncell];  // parking spot may be tighter
                    boolean[] bn = gn ? Bt[nk & 7] : B[nk & 7], bo = gn ? Bt[k & 7] : bk;
                    if (bn[ncell]) continue;
                    if (dk != 0 && (roomy[c] || roomy[ncell])) continue;     // moving turns only where there is lots of room
                    if (dk != 0 && (bo[ncell] || bn[c])) continue;           // mixed old/new footprints must be free too
                    if (dx != 0 && dy != 0 && (bn[y * gw + nx] || bn[ny * gw + x] || bo[y * gw + nx] || bo[ny * gw + x])) continue;
                    int nsx = nk * nc + ncell; if (closed[nsx]) continue;
                    double ng = G[s] + stepc[nk][d] + (dk != 0 ? cfg.rotMove : 0) + risk[ncell];
                    if (ng < G[nsx]) { G[nsx] = (float) ng; P[nsx] = s; push((float) (ng + hfa[ncell]), nsx); }
                }
            }
            for (int dk : new int[] {1, Config.NK - 1}) {   // pure in-place turn
                int nk = (k + dk) % Config.NK;
                if (B[nk & 7][c] || rotOk[c]) continue;
                int nsx = nk * nc + c; if (closed[nsx]) continue;
                double ng = G[s] + cfg.rotCost;
                if (ng < G[nsx]) { G[nsx] = (float) ng; P[nsx] = s; push((float) (ng + hfa[c]), nsx); }
            }
        }
        if (goalS < 0) goalS = bestS;   // search cut short: use the best goal found so far
        if (goalS < 0) return Collections.emptyList();
        ArrayList<Waypoint> out = new ArrayList<>();
        for (int s = goalS; s != -1; s = P[s]) { int c = s % nc, k = s / nc; out.add(new Waypoint((c % gw + .5) * cfg.cell, (c / gw + .5) * cfg.cell, k)); }
        Collections.reverse(out);
        return out;
    }

    /**
     * Plan from (x, y, th) to the goal.
     * USER: park on a ring around (gx, gy) facing it (table horizontal when the spots above/below are reachable).
     * HOME: reach the cell at (gx, gy) with orientation 0. POINT: reach the cell in any orientation.
     * risk: per-cell extra cost from OccupancyMemory.risk(). occ: from OccupancyMemory.buildOccupancy().
     */
    public List<Waypoint> plan(double x, double y, double th, GoalType type, double gx, double gy, boolean[] occ, float[] risk) {
        boolean[] roomy = buildRoomy(occ, roomOffs, roomyR), rotOk = buildRoomy(occ, rotOffs, rotR);
        boolean[][] Bt = buildBlocked(occ, kernTight, cfg.mgTight, x, y);
        squeeze = false;
        List<Waypoint> r = search(buildBlocked(occ, kernStrict, cfg.mgStrict, x, y), Bt, roomy, rotOk, risk, x, y, th, type, gx, gy);
        if (r.isEmpty()) {   // nothing fits with the full margin: retry tighter (the supervisor still guards contact)
            List<Waypoint> r2 = search(Bt, Bt, roomy, rotOk, risk, x, y, th, type, gx, gy);
            if (!r2.isEmpty()) { r = r2; squeeze = true; }
        }
        return r;
    }
}

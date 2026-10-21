package frc.robot.summon.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Plain-main tests (no JUnit needed, so they run anywhere with a JDK):
 *   java tools/Build.java && java -cp out frc.robot.summon.core.CoreTests
 * Scenes use pixel numbers from the browser simulator (1 px = 0.02 m) so results can be compared line by line.
 */
public final class CoreTests {
    static final double PX = 0.02;
    static int fails = 0, passes = 0;
    static void check(String name, boolean ok, String detail) {
        if (ok) passes++; else fails++;
        System.out.println((ok ? "PASS " : "FAIL ") + name + (detail.isEmpty() ? "" : "  [" + detail + "]"));
    }

    // ground truth for a scene (pixels)
    static List<double[]> rects = new ArrayList<>();   // x,y,w,h
    static List<double[]> circs = new ArrayList<>();   // x,y,r

    static void scene(String name) {
        rects.clear(); circs.clear();
        if (name.equals("slot")) { rects.add(new double[] {0, 240, 365, 20}); rects.add(new double[] {435, 240, 365, 20}); }
        else {
            rects.add(new double[] {220, 170, 40, 150}); rects.add(new double[] {450, 60, 150, 40}); rects.add(new double[] {520, 300, 120, 70});
            circs.add(new double[] {400, 260, 18});
        }
    }
    static OccupancyMemory memoryFor(Config cfg) {
        OccupancyMemory m = new OccupancyMemory(cfg);
        for (int cy = 0; cy < cfg.gh; cy++) for (int cx = 0; cx < cfg.gw; cx++) {
            double x0 = cx * 10, x1 = x0 + 10, y0 = cy * 10, y1 = y0 + 10; boolean hit = false;
            for (double[] r : rects) if (x0 < r[0] + r[2] && x1 > r[0] && y0 < r[1] + r[3] && y1 > r[1]) hit = true;
            for (double[] c : circs) {
                double qx = Math.max(x0, Math.min(c[0], x1)), qy = Math.max(y0, Math.min(c[1], y1));
                if (Math.hypot(qx - c[0], qy - c[1]) < c[2]) hit = true;
            }
            if (hit) m.staticMem[cy * cfg.gw + cx] = true;
            m.seen[cy * cfg.gw + cx] = true;
        }
        return m;
    }
    static long fnv(String s) {
        long h = 2166136261L;
        for (int i = 0; i < s.length(); i++) { h ^= s.charAt(i); h = (h * 16777619L) & 0xFFFFFFFFL; }
        return h;
    }
    /** true if any waypoint's real footprint overlaps real obstacles */
    static int truthHits(Config cfg, List<Waypoint> path) {
        int n = 0;
        for (Waypoint w : path) {
            double[][] fp = Geometry.footprint(w.x, w.y, w.th, cfg.hw, cfg.hh);
            boolean hit = false;
            for (double[] r : rects) if (Geometry.overlap(fp, Geometry.rect(r[0] * PX, r[1] * PX, r[2] * PX, r[3] * PX))) hit = true;
            for (double[] c : circs) {
                double cc = Math.cos(-w.th), ss = Math.sin(-w.th), dx = c[0] * PX - w.x, dy = c[1] * PX - w.y;
                double lx = cc * dx - ss * dy, ly = ss * dx + cc * dy;
                if (Math.hypot(lx - Math.max(-cfg.hw, Math.min(cfg.hw, lx)), ly - Math.max(-cfg.hh, Math.min(cfg.hh, ly))) < c[2] * PX) hit = true;
            }
            if (hit) n++;
        }
        return n;
    }

    static List<Waypoint> report(String label, Config cfg, OrientationPlanner pl, OccupancyMemory m, double sx, double sy,
                                 OrientationPlanner.GoalType gt, double gx, double gy) {
        long t0 = System.nanoTime();
        List<Waypoint> p = pl.plan(sx * PX, sy * PX, 0, gt, gx * PX, gy * PX, m.buildOccupancy(0, new ArrayList<>(), false, sx * PX, sy * PX), m.risk());
        double ms = (System.nanoTime() - t0) / 1e6;
        StringBuilder sb = new StringBuilder(); int turns = 0;
        for (int i = 0; i < p.size(); i++) {
            Waypoint w = p.get(i);
            sb.append(Math.round(w.x / PX)).append(',').append(Math.round(w.y / PX)).append(',').append(w.k).append(';');
            if (i > 0 && w.k != p.get(i - 1).k) turns++;
        }
        String s = sb.toString(); if (s.endsWith(";")) s = s.substring(0, s.length() - 1);
        Waypoint l = p.isEmpty() ? null : p.get(p.size() - 1);
        System.out.println("XC " + label + " len " + p.size() + " turns " + turns + " squeeze " + pl.squeeze + " last "
                + (l == null ? "-" : Math.round(l.x / PX) + "," + Math.round(l.y / PX) + ",k" + l.k) + " hash " + fnv(s) + " ms " + String.format("%.1f", ms));
        return p;
    }

    public static void main(String[] args) {
        Config cfg = new Config();
        OrientationPlanner pl = new OrientationPlanner(cfg);

        System.out.println("--- cross-check scenes (compare with the simulator's XC lines)");
        scene("slot"); OccupancyMemory m = memoryFor(cfg);
        List<Waypoint> pSlot = report("slot->user", cfg, pl, m, 85, 95, OrientationPlanner.GoalType.USER, 700, 430);
        report("slot->home", cfg, pl, m, 700, 360, OrientationPlanner.GoalType.HOME, 85, 95);
        scene("default"); m = memoryFor(cfg);
        List<Waypoint> pDef = report("default->user", cfg, pl, m, 85, 95, OrientationPlanner.GoalType.USER, 700, 430);
        List<Waypoint> p450 = report("default->user(400,450)", cfg, pl, m, 85, 95, OrientationPlanner.GoalType.USER, 400, 450);
        report("default->home", cfg, pl, m, 700, 352, OrientationPlanner.GoalType.HOME, 85, 95);

        System.out.println("--- planner properties");
        scene("slot");
        check("slot path never overlaps real walls", truthHits(cfg, pSlot) == 0, "overlapping waypoints: " + truthHits(cfg, pSlot));
        int through = -1;
        for (Waypoint w : pSlot) if (Math.abs(w.y / PX - 250) < 6 && w.x / PX > 365 && w.x / PX < 435) { through = w.k; break; }
        check("desk faces along the slot (long axis vertical) as it passes through", through == 4 || through == 12, "k=" + through);
        scene("default"); 
        check("default path never overlaps real obstacles", truthHits(cfg, pDef) == 0, "overlapping waypoints: " + truthHits(cfg, pDef));
        Waypoint last = pDef.get(pDef.size() - 1);
        check("parks with the table horizontal (above/below the user)", last.k % 8 == 0, "k=" + last.k);
        double d = Math.hypot(last.x - 700 * PX, last.y - 430 * PX);
        check("parks on the 1.4 m ring around the user", Math.abs(d - 1.40) < 0.12 + 0.2, String.format("%.2f m", d));

        check("second target: path never overlaps real obstacles", truthHits(cfg, p450) == 0, "overlapping waypoints: " + truthHits(cfg, p450));
        check("second target: parks table-horizontal", p450.get(p450.size() - 1).k % 8 == 0, "k=" + p450.get(p450.size() - 1).k);

        OccupancyMemory boxed = new OccupancyMemory(cfg);   // goal walled in on all sides: no route exists
        int bx = 60, by = 25;
        for (int dy = -6; dy <= 6; dy++) for (int dx = -6; dx <= 6; dx++) if (Math.max(Math.abs(dx), Math.abs(dy)) == 6) boxed.staticMem[(by + dy) * cfg.gw + bx + dx] = true;
        Arrays.fill(boxed.seen, true);
        long tb = System.nanoTime();
        List<Waypoint> pBox = pl.plan(2.0, 2.0, 0, OrientationPlanner.GoalType.POINT, (bx + .5) * cfg.cell, (by + .5) * cfg.cell,
                boxed.buildOccupancy(0, new ArrayList<>(), false, 2.0, 2.0), boxed.risk());
        double msBox = (System.nanoTime() - tb) / 1e6;
        check("unreachable goal: no path, search stops at maxPops instead of exhausting the state space",
                pBox.isEmpty() && pl.capped && pl.lastPops <= 2 * cfg.maxPops,
                "len " + pBox.size() + " pops " + pl.lastPops + " (cap " + cfg.maxPops + " per pass) capped " + pl.capped + String.format(" %.1f ms", msBox));

        System.out.println("--- safety supervisor");
        SafetySupervisor sup = new SafetySupervisor(cfg);
        // wall of lidar points at x = 7.0 m from y = 2 to 4 m; desk at (5.0, 3.0) heading +x, commanded 0.8 m/s
        List<double[]> wall = new ArrayList<>();
        for (double y = 2.0; y <= 4.0; y += 0.05) wall.add(new double[] {7.0, y});
        double x = 5.0, vx = 0.0, dt = 1.0 / 60; double minGap = 1e9; boolean touched = false;
        for (int i = 0; i < 60 * 12; i++) {
            SafetySupervisor.Result r = sup.supervise(0.8, 0, 0, x, 3.0, 0, vx, 0, wall);
            double a = r.held ? cfg.emergencyDecel : 0.5;                    // controller brakes harder when it intervenes
            double want = r.vx;
            if (want > vx) vx = Math.min(want, vx + 0.5 * dt); else vx = Math.max(want, vx - (r.held ? cfg.emergencyDecel : 1.2) * dt);
            x += vx * dt;
            double gap = 7.0 - (x + cfg.hw); minGap = Math.min(minGap, gap);
            if (gap < 0) touched = true;
        }
        check("full-speed approach to a wall stops without touching", !touched && vx < 0.01, String.format("final gap %.2f m, min gap %.2f m", 7.0 - (x + cfg.hw), minGap));
        // sudden obstacle appears 0.45 m ahead of the nose while at full speed
        x = 5.0; vx = 0.8; touched = false; List<double[]> sudden = new ArrayList<>();
        for (double y = 2.7; y <= 3.3; y += 0.05) sudden.add(new double[] {x + cfg.hw + 1.1, y});
        for (int i = 0; i < 60 * 8; i++) {
            SafetySupervisor.Result r = sup.supervise(0.8, 0, 0, x, 3.0, 0, vx, 0, sudden);
            if (r.vx < vx) vx = Math.max(r.vx, vx - (r.held ? cfg.emergencyDecel : 1.2) * dt);
            x += vx * dt; if (sudden.get(0)[0] - (x + cfg.hw) < 0) touched = true;
        }
        check("box appearing 1.1 m ahead of the nose at full speed is not hit", !touched, String.format("stopped at gap %.2f m", sudden.get(0)[0] - (x + cfg.hw)));
        // yaw refused when a corner would swing into a wall
        List<double[]> near = new ArrayList<>(); for (double xx = 3.0; xx <= 7.0; xx += 0.05) near.add(new double[] {xx, 3.0 + cfg.hh + 0.07});
        SafetySupervisor.Result yr = sup.supervise(0, 0, 1.0, 5.0, 3.0, 0, 0, 0, near);
        check("turning toward a nearby wall is refused", yr.w == 0 && yr.held, "w=" + yr.w);
        SafetySupervisor.Result free = sup.supervise(0, 0, 1.0, 5.0, 3.0, 0, 0, 0, new ArrayList<>());
        check("turning in free space is allowed", free.w == 1.0 && !free.held, "w=" + free.w);

        System.out.println("--- map memory");
        OccupancyMemory mem = new OccupancyMemory(cfg);
        List<OccupancyMemory.Hit> hits = new ArrayList<>(); hits.add(new OccupancyMemory.Hit(4.1, 4.1, OccupancyMemory.HitType.STATIC));
        hits.add(new OccupancyMemory.Hit(8.1, 8.1, OccupancyMemory.HitType.PERSON));
        double t = 0; List<OccupancyMemory.Ray> none = new ArrayList<>();
        for (int i = 0; i < 90; i++) { mem.update(t, hits, none); t += 1.0 / 60; }
        int sc = (int) (4.1 / cfg.cell) + (int) (4.1 / cfg.cell) * cfg.gw, pc = (int) (8.1 / cfg.cell) + (int) (8.1 / cfg.cell) * cfg.gw;
        check("a steadily-seen obstacle is remembered", mem.staticMem[sc], "");
        check("a person is never stored as furniture", !mem.staticMem[pc], "");
        check("people leave a traffic trace", mem.traffic[pc] > 0, String.format("%.2f", mem.traffic[pc]));
        byte[] blob = mem.serialize(); OccupancyMemory mem2 = new OccupancyMemory(cfg);
        check("memory survives save and load", mem2.deserialize(blob) && mem2.staticMem[sc] && Math.abs(mem2.traffic[pc] - mem.traffic[pc]) < 0.2, "bytes=" + blob.length);
        List<OccupancyMemory.Ray> rays = new ArrayList<>(); rays.add(new OccupancyMemory.Ray(2.0, 4.1, 1, 0, 0.1, 3.0));   // a ray passes through the cell
        mem.update(t, new ArrayList<>(), rays);
        check("a lidar ray passing through clears it (moved furniture leaves no ghost)", !mem.staticMem[sc], "");
        OccupancyMemory edge = new OccupancyMemory(cfg); List<OccupancyMemory.Hit> off = new ArrayList<>();   // just off the map
        off.add(new OccupancyMemory.Hit(-0.1, 1.1, OccupancyMemory.HitType.STATIC)); off.add(new OccupancyMemory.Hit(1.1, -0.1, OccupancyMemory.HitType.STATIC));
        t = 0; for (int i = 0; i < 90; i++) { edge.update(t, off, none); t += 1.0 / 60; }
        int ex = (int) (1.1 / cfg.cell) * cfg.gw, ey = (int) (1.1 / cfg.cell);
        check("hits just off the map (x or y = -0.1 m) are not stored on the edge cells", edge.countStatic() == 0 && edge.countSeen() == 0,
                "static " + edge.countStatic() + " seen " + edge.countSeen() + " (cell x0 " + edge.staticMem[ex] + ", cell y0 " + edge.staticMem[ey] + ")");

        System.out.println("--- timing");
        scene("default"); m = memoryFor(cfg); double[] ts = new double[25];
        for (int i = 0; i < ts.length; i++) {
            long t0 = System.nanoTime();
            pl.plan(1.7, 1.9, 0, OrientationPlanner.GoalType.USER, 14.0, 8.6, m.buildOccupancy(0, new ArrayList<>(), false, 1.7, 1.9), m.risk());
            ts[i] = (System.nanoTime() - t0) / 1e6;
        }
        Arrays.sort(ts);
        System.out.println(String.format("plan time over %d runs: median %.1f ms, max %.1f ms (JIT-warm after the first few)", ts.length, ts[ts.length / 2], ts[ts.length - 1]));
        System.out.println(passes + " passed, " + fails + " failed");
        System.exit(fails == 0 ? 0 : 1);
    }
}

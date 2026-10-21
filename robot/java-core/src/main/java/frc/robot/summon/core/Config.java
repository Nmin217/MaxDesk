package frc.robot.summon.core;

/**
 * Every tunable in one place. SI units (meters, seconds, radians).
 * Values mirror the browser simulator, where 1 px = 0.02 m (50 px per meter).
 * Replace hw/hh with measured values from the real robot: they must describe the LARGEST outline
 * at any height (desktop overhang, monitor), not just the frame.
 */
public final class Config {
    // ---- map / grid ----
    public double mapW = 16.0, mapH = 10.0;      // m
    public double cell = 0.20;                   // m
    public int gw = 80, gh = 50;

    // ---- robot footprint (half extents) ----
    public double hw = 0.56, hh = 0.36;          // m  (sim: 28 x 18 px)

    // ---- orientation-aware planner ----
    public static final int NKF = 8;             // distinct footprints (the rectangle repeats every 180 deg)
    public static final int NK = 16;             // orientations, 22.5 deg apart, full 360 deg
    public double mgStrict = 0.14, mgTight = 0.04;   // clearance margins (m)
    public double rotMargin = 0.16;              // extra radius where an in-place turn is allowed (m)
    public double roomyMargin = 0.44;            // extra radius where a turn-while-moving is allowed (m)
    public double alignPen = 0.25;               // cost of travelling sideways to the long axis
    public double rotCost = 1.0;                 // in-place 22.5 deg turn (cells)
    public double rotMove = 0.5;                 // 22.5 deg turn while moving (cells)
    public double heurWeight = 1.5;              // weighted A*
    public double sectorPen = 45;                // cost of NOT parking directly above/below the user (cells)
    public double unknownPen = 0.15;             // per unseen cell
    public double trafficK = 0.015, trafficMax = 0.4;
    public double ringRadius = 1.40, ringTol = 0.12, heurRingRadius = 1.52;  // parking ring around the user (m)
    public int maxPops = 20000;                  // heap pops per search pass (stale ones too); the test scenes need <= ~6000

    // ---- map memory ----
    public double persistSeconds = 1.0;          // a hit must be seen this long to count as static
    public double shortMemory = 1.2;             // s: recent hits count as obstacles
    public double rayStep = 0.10;                // m

    // ---- safety supervisor ----
    public double safeDecel = 0.9;               // m/s^2: the stop the supervisor GUARANTEES (below real braking)
    public double stopGap = 0.10;                // m
    public double safeMargin = 0.04;             // m
    public double yawMargin = 0.06;              // m
    public double emergencyDecel = 2.4;          // m/s^2: used when the supervisor intervenes (below item-slip)
    public double supervisorRange = 3.6;         // m
}

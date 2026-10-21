#!/usr/bin/env python3
"""Pi safety monitor: LiDAR -> NetworkTables. Never drives motors.

Structure:
  * LidarReader runs the (blocking) driver in a background thread and keeps
    the latest scan + its monotonic timestamp.
  * A fixed-rate main loop (safety.loop_hz, default 20 Hz) publishes an
    incrementing Pi/Heartbeat every iteration and evaluates the latest scan:
      no scan within max_stale_scan_sec        -> Stop, STALE (or ERROR)
      obstacle closer than stop_distance_m     -> Stop, OBSTACLE
      < min_sector_points returns in the sector -> Stop, BLIND
      otherwise                                 -> no Stop, OK
Distances from rplidar 0.9.x are millimetres; everything published is metres.
"""
import argparse
import json
import time
from collections import namedtuple

MM_PER_M = 1000.0
DEFAULT_MIN_SECTOR_POINTS = 5
DEFAULT_LOOP_HZ = 20.0

Decision = namedtuple("Decision", "stop status obstacle distance_m points")


def load_cfg(p):
    with open(p, "r", encoding="utf-8") as f:
        return json.load(f)


def in_sector(a, lo, hi):
    a = a % 360.0
    if lo < 0 and a > 180.0:
        a -= 360.0
    return lo <= a <= hi


def evaluate_scan(scan, lo_deg, hi_deg, stop_m,
                  min_points=DEFAULT_MIN_SECTOR_POINTS):
    """scan: iterable of (quality, angle_deg, distance_mm). Returns Decision."""
    nearest = float("inf")
    n = 0
    for item in scan or ():
        if len(item) < 3:
            continue
        a, d = item[1], item[2]
        if d is None or d <= 0 or not in_sector(a, lo_deg, hi_deg):
            continue
        n += 1
        nearest = min(nearest, d / MM_PER_M)
    dist = nearest if n else -1.0
    obstacle = n > 0 and nearest < stop_m
    if obstacle:
        return Decision(True, "OBSTACLE", True, dist, n)
    if n < min_points:
        # Too few returns: sensor blocked / object inside minimum range.
        return Decision(True, "BLIND", False, dist, n)
    return Decision(False, "OK", False, dist, n)


class SafetyMonitor:
    def __init__(self, nt, reader, safety_cfg, clock=time.monotonic):
        self.nt, self.reader, self.clock = nt, reader, clock
        self.stop_m = float(safety_cfg["stop_distance_m"])
        self.lo_deg = float(safety_cfg["forward_angle_min_deg"])
        self.hi_deg = float(safety_cfg["forward_angle_max_deg"])
        self.max_stale = float(safety_cfg["max_stale_scan_sec"])
        self.min_points = int(safety_cfg.get("min_sector_points",
                                             DEFAULT_MIN_SECTOR_POINTS))
        self.loop_hz = float(safety_cfg.get("loop_hz", DEFAULT_LOOP_HZ))
        self.heartbeat = 0

    def decide(self):
        scan, t, _count, err = self.reader.snapshot()
        if t is None or (self.clock() - t) > self.max_stale:
            return Decision(True, "ERROR" if err else "STALE", False, -1.0, 0)
        return evaluate_scan(scan, self.lo_deg, self.hi_deg, self.stop_m,
                             self.min_points)

    def step(self):
        self.heartbeat += 1
        self.nt.set_heartbeat(self.heartbeat)
        d = self.decide()
        self.nt.set_obstacle_forward(d.obstacle)
        self.nt.set_obstacle_distance(d.distance_m)
        self.nt.set_stop(d.stop)
        self.nt.set_lidar_status(d.status)
        return d

    def run(self, stop_event=None, sleep=time.sleep):
        period = 1.0 / self.loop_hz
        next_t = self.clock()
        while stop_event is None or not stop_event.is_set():
            self.step()
            next_t += period
            delay = next_t - self.clock()
            if delay > 0:
                sleep(delay)
            else:
                next_t = self.clock()   # overran; don't try to catch up


def build(cfg, nt_factory=None, reader_factory=None):
    from lidar_reader import LidarReader
    from nt_client import NTClient
    ntc = cfg["networktables"]
    nt = (nt_factory or NTClient)(server=ntc.get("server"),
                                  port=int(ntc.get("port", 5810)),
                                  team=ntc.get("team"))
    lc = cfg["lidar"]
    reader = (reader_factory or LidarReader)(
        lc["port"], int(lc.get("baudrate", 115200)), float(lc.get("timeout", 3.0)))
    return nt, reader


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.json")
    args = ap.parse_args(argv)
    cfg = load_cfg(args.config)

    nt, reader = build(cfg)
    nt.set_stop(True)
    nt.set_lidar_status("STALE")
    monitor = SafetyMonitor(nt, reader, cfg["safety"])
    print("Starting LiDAR reader...")
    reader.start()
    try:
        monitor.run()
    except KeyboardInterrupt:
        print("Stopping.")
    finally:
        try:
            nt.set_stop(True)
            nt.set_lidar_status("SHUTDOWN")
        finally:
            reader.close()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Pi safety monitor: LiDAR -> NetworkTables. Never drives motors."""
import argparse, json, time
from lidar_reader import LidarReader
from nt_client import NTClient

def load_cfg(p):
    with open(p, "r", encoding="utf-8") as f: return json.load(f)

def in_sector(a, lo, hi):
    a = a % 360.0
    if lo < 0 and a > 180.0: a -= 360.0
    return lo <= a <= hi

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.json")
    args = ap.parse_args()
    cfg = load_cfg(args.config)

    nt = NTClient(cfg["networktables"]["server"], int(cfg["networktables"].get("port", 5810)))
    reader = LidarReader(cfg["lidar"]["port"],
                         int(cfg["lidar"].get("baudrate", 115200)),
                         float(cfg["lidar"].get("timeout", 3.0)))
    print("Connecting LiDAR...")
    reader.connect()

    stop_m   = float(cfg["safety"]["stop_distance_m"])
    lo_deg   = float(cfg["safety"]["forward_angle_min_deg"])
    hi_deg   = float(cfg["safety"]["forward_angle_max_deg"])
    stale    = float(cfg["safety"]["max_stale_scan_sec"])

    try:
        while True:
            nt.set_heartbeat(time.time())
            try:
                for scan in reader.read_scans():
                    nearest = float("inf")
                    for _, a, d in scan:
                        if d and d > 0 and in_sector(a, lo_deg, hi_deg) and d < nearest:
                            nearest = d
                    obstacle = nearest < stop_m
                    nt.set_obstacle_forward(obstacle)
                    nt.set_obstacle_distance(nearest if nearest != float("inf") else -1.0)
                    if obstacle:
                        nt.set_stop(True); nt.set_lidar_status("OBSTACLE")
                    elif reader.is_stale(stale):
                        nt.set_stop(True); nt.set_lidar_status("STALE")
                    else:
                        nt.set_stop(False); nt.set_lidar_status("OK")
                    time.sleep(0.05)
            except Exception as e:
                print("LiDAR error:", e)
                nt.set_stop(True); nt.set_lidar_status("ERROR")
                time.sleep(1.0)
    except KeyboardInterrupt:
        print("Stopping.")
    finally:
        nt.set_stop(True); nt.set_lidar_status("SHUTDOWN"); reader.close()

if __name__ == "__main__":
    main()

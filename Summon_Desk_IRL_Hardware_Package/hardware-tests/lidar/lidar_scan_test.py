#!/usr/bin/env python3
"""Safe LiDAR scan test. Never commands robot movement."""
import argparse, json, sys, time
try:
    from rplidar import RPLidar, RPLidarException
except ImportError:
    raise SystemExit("Install rplidar: pip install -r requirements.txt")

def load_config(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def summarize(scan):
    valid, invalid = [], 0
    for item in scan:
        if len(item) < 3:
            invalid += 1; continue
        q, a, d = item[0], item[1], item[2]
        if d is None or d <= 0:
            invalid += 1; continue
        valid.append((q, a, d))
    return valid, invalid

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="../config.json")
    args = ap.parse_args()
    cfg = load_config(args.config)["lidar"]

    print("Model:", cfg.get("model"))
    print("Port:", cfg["port"], "Baud:", cfg["baudrate"])
    print("Verify against your exact sensor datasheet. Ctrl+C to stop.\n")

    lidar = None
    try:
        lidar = RPLidar(cfg["port"], baudrate=int(cfg["baudrate"]),
                        timeout=float(cfg.get("timeout", 3.0)))
        try: print("Info:", lidar.get_info())
        except Exception as e: print("Info error:", e)
        try: print("Health:", lidar.get_health())
        except Exception as e: print("Health error:", e)

        n = 0; last = time.time()
        for scan in lidar.iter_scans():
            n += 1
            valid, invalid = summarize(scan)
            if valid and time.time() - last >= 1.0:
                nearest = sorted(valid, key=lambda x: x[2])[:5]
                print(f"scan={n} valid={len(valid)} invalid={invalid} "
                      f"nearest={[(round(a,1), round(d,3)) for _,a,d in nearest]}")
                last = time.time()
    except KeyboardInterrupt:
        print("\nStopped by user.")
    except RPLidarException as e:
        print("RPLIDAR error:", e); return 2
    except Exception as e:
        print("Error:", e); return 3
    finally:
        if lidar:
            try: lidar.stop()
            except Exception: pass
            try: lidar.disconnect()
            except Exception: pass
        print("LiDAR closed.")
    return 0

if __name__ == "__main__":
    sys.exit(main())

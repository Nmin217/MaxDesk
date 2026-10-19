#!/usr/bin/env python3
"""LiDAR wrapper with health and stale-scan tracking."""
import time
try:
    from rplidar import RPLidar, RPLidarException
except ImportError:
    raise SystemExit("Install rplidar first.")

class LidarReader:
    def __init__(self, port, baudrate=115200, timeout=3.0):
        self.port, self.baudrate, self.timeout = port, baudrate, timeout
        self.lidar = None; self.last_scan_time = 0.0
        self.last_error = None; self.connected = False

    def connect(self):
        self.lidar = RPLidar(self.port, baudrate=self.baudrate, timeout=self.timeout)
        self.connected = True
        self.last_scan_time = time.time()

    def get_info(self):
        return self.lidar.get_info() if self.lidar else None

    def get_health(self):
        return self.lidar.get_health() if self.lidar else None

    def read_scans(self):
        if not self.lidar: raise RuntimeError("LiDAR not connected")
        try:
            for scan in self.lidar.iter_scans():
                self.last_scan_time = time.time()
                yield scan
        except RPLidarException as e:
            self.last_error = str(e); self.connected = False; raise

    def is_stale(self, max_age_sec):
        return (time.time() - self.last_scan_time) > max_age_sec

    def close(self):
        self.connected = False
        if self.lidar:
            try: self.lidar.stop()
            except Exception: pass
            try: self.lidar.disconnect()
            except Exception: pass
        self.lidar = None

#!/usr/bin/env python3
"""LiDAR wrapper: a background thread reads scans and keeps the latest one.

The main safety loop never blocks on the serial port. It polls snapshot()
at a fixed rate and judges staleness from the scan timestamp, so a stalled
sensor (iter_scans blocking forever) is detected instead of silently
freezing the last published Safety/Stop value.

Scan points are whatever the driver yields: (quality, angle_deg, distance_mm).
rplidar / rplidar-roboticia 0.9.x report distance in MILLIMETRES.

The `rplidar` package is imported lazily (only when connecting with the
default factory) so this module can be imported and tested without it.
"""
import threading
import time


def default_lidar_factory(port, baudrate, timeout):
    """Open a real RPLIDAR. Imported lazily so tests don't need the package."""
    try:
        from rplidar import RPLidar
    except ImportError as e:  # pragma: no cover - depends on the Pi install
        raise RuntimeError(
            "rplidar is not installed: pip install -r requirements.txt") from e
    return RPLidar(port, baudrate=baudrate, timeout=timeout)


class LidarReader:
    def __init__(self, port, baudrate=115200, timeout=3.0,
                 lidar_factory=None, clock=time.monotonic, retry_delay_sec=1.0):
        self.port, self.baudrate, self.timeout = port, baudrate, timeout
        self._factory = lidar_factory or default_lidar_factory
        self._clock = clock
        self.retry_delay_sec = float(retry_delay_sec)
        self.lidar = None
        self.connected = False
        self.last_error = None
        self._lock = threading.Lock()
        self._latest_scan = None
        self._latest_time = None      # clock() of the latest scan, None = never
        self._scan_count = 0
        self._stop = threading.Event()
        self._thread = None

    # -- device -----------------------------------------------------------
    def connect(self):
        self.lidar = self._factory(self.port, self.baudrate, self.timeout)
        self.connected = True

    def get_info(self):
        return self.lidar.get_info() if self.lidar else None

    def get_health(self):
        return self.lidar.get_health() if self.lidar else None

    def read_scans(self):
        """Blocking generator over driver scans; records each as latest."""
        if not self.lidar:
            raise RuntimeError("LiDAR not connected")
        for scan in self.lidar.iter_scans():
            self._record(scan)
            yield scan

    def _record(self, scan):
        with self._lock:
            self._latest_scan = list(scan)
            self._latest_time = self._clock()
            self._scan_count += 1
            self.last_error = None

    def _release_device(self):
        lidar, self.lidar = self.lidar, None
        self.connected = False
        if lidar is not None:
            for fn in ("stop", "disconnect"):
                try:
                    getattr(lidar, fn)()
                except Exception:
                    pass

    # -- background thread -----------------------------------------------
    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="lidar-reader",
                                        daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                if self.lidar is None:
                    self.connect()
                for _ in self.read_scans():
                    if self._stop.is_set():
                        break
                else:
                    if not self._stop.is_set():
                        raise RuntimeError("scan iterator ended")
            except Exception as e:
                if self._stop.is_set():
                    break
                with self._lock:
                    self.last_error = "%s: %s" % (type(e).__name__, e)
                print("LiDAR error:", self.last_error)
                self._release_device()
                self._stop.wait(self.retry_delay_sec)

    # -- queries used by the safety loop -----------------------------------
    def snapshot(self):
        """Return (scan, scan_time, scan_count, last_error) atomically."""
        with self._lock:
            return (self._latest_scan, self._latest_time,
                    self._scan_count, self.last_error)

    def scan_age(self):
        with self._lock:
            t = self._latest_time
        return float("inf") if t is None else self._clock() - t

    def is_stale(self, max_age_sec):
        return self.scan_age() > max_age_sec

    def close(self, join_timeout=2.0):
        self._stop.set()
        self._release_device()   # closing the port unblocks iter_scans
        t = self._thread
        if t and t is not threading.current_thread():
            t.join(join_timeout)
        self._thread = None

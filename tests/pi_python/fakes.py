"""Fakes for the Pi Python code: fake `rplidar`, `ntcore` and `networktables`
modules plus a shared WORLD that records every NetworkTables write.

Importing this module installs the fakes into sys.modules and puts the Pi
source directory on sys.path. SUMMON_PKG_ROOT may point at an alternative
copy of Summon_Desk_IRL_Hardware_Package (used to run the suite against the
pre-fix code).
"""
import json
import os
import sys
import tempfile
import threading
import time
import types
from collections import defaultdict

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
PKG_ROOT = os.environ.get("SUMMON_PKG_ROOT") or os.path.join(
    REPO, "Summon_Desk_IRL_Hardware_Package")
PI_SRC = os.path.join(PKG_ROOT, "hardware", "raspberry_pi")
LIDAR_TOOL_SRC = os.path.join(PKG_ROOT, "hardware-tests", "lidar")


class World:
    def __init__(self):
        self.reset()

    def reset(self):
        self.lock = threading.Lock()
        self.history = defaultdict(list)     # "Table/Key" -> [values]
        self.nt_calls = []
        self.strict_pynetworktables = False
        self.interrupt = False
        self._interrupt_fired = False
        self.release = threading.Event()
        self.scan_source = None              # generator function(world)
        self.lidars = []

    def record(self, path, value):
        with self.lock:
            self.history[path].append(value)
        if (path.endswith("Pi/Heartbeat") and self.interrupt
                and not self._interrupt_fired):
            self._interrupt_fired = True
            raise KeyboardInterrupt  # stops safety_monitor.main()

    def latest(self, key, table="SummonDesk"):
        with self.lock:
            h = self.history.get(table + "/" + key)
            return h[-1] if h else None

    def values(self, key, table="SummonDesk"):
        with self.lock:
            return list(self.history.get(table + "/" + key, []))


WORLD = World()


# ---- NetworkTables ---------------------------------------------------------
class FakeEntry:
    def __init__(self, path):
        self.path = path

    def _set(self, v):
        WORLD.record(self.path, v)
        return True

    setString = setBoolean = setDouble = setInteger = _set


class FakeTable:
    def __init__(self, name):
        self.name = name

    def getEntry(self, key):
        return FakeEntry(self.name + "/" + key)


class FakeNTInstance:
    def startClient4(self, identity):
        WORLD.nt_calls.append(("startClient4", identity))

    def setServer(self, server, port=0):
        WORLD.nt_calls.append(("setServer", server, port))

    def setServerTeam(self, team, port=0):
        WORLD.nt_calls.append(("setServerTeam", team, port))

    def getTable(self, name):
        return FakeTable(name)

    def isConnected(self):
        return True

    def stopClient(self):
        WORLD.nt_calls.append(("stopClient",))


ntcore = types.ModuleType("ntcore")
ntcore.NetworkTableInstance = type(
    "NetworkTableInstance", (), {"getDefault": staticmethod(FakeNTInstance)})


class _PyNetworkTables:
    @staticmethod
    def initialize(server=None, **kwargs):
        # Real pynetworktables: `def initialize(cls, server=None)` - no port.
        if WORLD.strict_pynetworktables and kwargs:
            raise TypeError("initialize() got an unexpected keyword argument "
                            "'%s'" % next(iter(kwargs)))
        WORLD.nt_calls.append(("pynetworktables.initialize", server, kwargs))
        return True

    @staticmethod
    def getTable(name):
        return FakeTable(name)


networktables = types.ModuleType("networktables")
networktables.NetworkTables = _PyNetworkTables


# ---- rplidar -----------------------------------------------------------------
rplidar = types.ModuleType("rplidar")


class RPLidarException(Exception):
    pass


class FakeRPLidar:
    def __init__(self, port, baudrate=115200, timeout=1, logger=None):
        self.port, self.baudrate, self.timeout = port, baudrate, timeout
        WORLD.lidars.append(self)

    def iter_scans(self, *a, **k):
        return WORLD.scan_source(WORLD)

    def get_info(self):
        return {"model": 24}

    def get_health(self):
        return ("Good", 0)

    def stop(self):
        WORLD.release.set()

    def disconnect(self):
        WORLD.release.set()


rplidar.RPLidar = FakeRPLidar
rplidar.RPLidarException = RPLidarException

sys.modules["ntcore"] = ntcore
sys.modules["networktables"] = networktables
sys.modules["rplidar"] = rplidar
for _p in (PI_SRC, LIDAR_TOOL_SRC):
    if _p not in sys.path:
        sys.path.insert(0, _p)


# ---- scans (quality, angle_deg, distance_mm) ----------------------------------
def forward_points(dist_mm, n=10):
    """n points spread over -25..+25 deg, expressed as 0..360 like rplidar."""
    step = 50.0 / max(n - 1, 1)
    return [(15, (-25.0 + i * step) % 360.0, float(dist_mm)) for i in range(n)]


def rear_points(dist_mm=2000.0, n=20):
    return [(15, 150.0 + i * 3.0, float(dist_mm)) for i in range(n)]


def streaming(scan, period=0.02):
    """Scan source that keeps producing `scan` until the test ends."""
    def source(world):
        while not world.interrupt:
            yield list(scan)
            time.sleep(period)
        raise RPLidarException("test finished")
    return source


def stalling(scan, count=3, period=0.02):
    """Scan source that yields `count` scans then blocks (sensor stall)."""
    def source(world):
        for _ in range(count):
            yield list(scan)
            time.sleep(period)
        world.release.wait()
        raise RPLidarException("port closed")
    return source


def wait_for(pred, timeout=2.0, interval=0.01):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(interval)
    return pred()


TEST_SAFETY = {
    "forward_angle_min_deg": -30, "forward_angle_max_deg": 30,
    "stop_distance_m": 0.8, "slow_distance_m": 1.5,
    "max_stale_scan_sec": 0.3, "min_sector_points": 5, "loop_hz": 50,
}


class MonitorHarness:
    """Runs safety_monitor.main() (old or new) in a thread against the fakes."""

    def __init__(self, scan_source, safety=None):
        WORLD.reset()
        WORLD.scan_source = scan_source
        cfg = {"lidar": {"port": "/dev/fake", "baudrate": 115200, "timeout": 1.0},
               "safety": dict(TEST_SAFETY, **(safety or {})),
               "networktables": {"server": "10.0.0.2", "port": 5810}}
        self.tmp = tempfile.TemporaryDirectory()
        self.cfg_path = os.path.join(self.tmp.name, "config.json")
        with open(self.cfg_path, "w") as f:
            json.dump(cfg, f)
        self.exc = None
        self._argv = sys.argv
        sys.argv = ["safety_monitor.py", "--config", self.cfg_path]
        import safety_monitor
        self.thread = threading.Thread(target=self._target,
                                       args=(safety_monitor.main,), daemon=True)
        self.thread.start()

    def _target(self, main):
        try:
            main()
        except BaseException as e:  # noqa: BLE001 - surface to the test
            self.exc = e

    def stop(self):
        WORLD.interrupt = True
        WORLD.release.set()
        self.thread.join(5.0)
        sys.argv = self._argv
        self.tmp.cleanup()

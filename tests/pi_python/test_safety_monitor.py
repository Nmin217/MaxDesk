"""Behavioural tests: run safety_monitor.main() against fake LiDAR + fake NT."""
import threading
import time
import unittest

import fakes
from fakes import WORLD, MonitorHarness, forward_points, rear_points, wait_for


def stop():
    return WORLD.latest("Safety/Stop")


def status():
    return WORLD.latest("LiDAR/Status")


def distance():
    return WORLD.latest("Obstacle/Distance")


class HarnessCase(unittest.TestCase):
    def run_monitor(self, source, **safety):
        h = MonitorHarness(source, safety)
        self.addCleanup(h.stop)
        return h

    def assertEventually(self, pred, msg, timeout=2.0):
        if not wait_for(pred, timeout):
            self.fail("%s (Stop=%r Status=%r Distance=%r)"
                      % (msg, stop(), status(), distance()))


class TestUnitsMillimetres(HarnessCase):
    """Bug 1: rplidar yields mm; thresholds and Obstacle/Distance are metres."""

    def test_half_metre_forward_hit_triggers_stop(self):
        self.run_monitor(fakes.streaming(forward_points(500) + rear_points()))
        self.assertEventually(
            lambda: stop() is True and status() == "OBSTACLE"
            and distance() is not None and abs(distance() - 0.5) < 1e-9,
            "500 mm forward return should publish Stop/OBSTACLE at 0.5 m")
        self.assertIs(WORLD.latest("Obstacle/Forward"), True)

    def test_two_metres_forward_does_not_stop(self):
        self.run_monitor(fakes.streaming(forward_points(2000) + rear_points()))
        self.assertEventually(
            lambda: stop() is False and status() == "OK"
            and distance() is not None and abs(distance() - 2.0) < 1e-9,
            "2000 mm forward return should be clear with Distance=2.0 m")
        self.assertIs(WORLD.latest("Obstacle/Forward"), False)


class TestStaleness(HarnessCase):
    """Bug 2: a stalled LiDAR (iter_scans blocks) must publish Stop/STALE."""

    def test_stalled_lidar_publishes_stale_stop(self):
        self.run_monitor(fakes.stalling(forward_points(2000) + rear_points()))
        self.assertEventually(lambda: status() == "OK" and stop() is False,
                              "should be OK while scans arrive")
        t0 = time.monotonic()
        self.assertEventually(lambda: status() == "STALE" and stop() is True,
                              "no scans for max_stale_scan_sec -> Stop/STALE")
        self.assertLess(time.monotonic() - t0, 1.0)

    def test_heartbeat_keeps_running_while_lidar_stalled(self):
        self.run_monitor(fakes.stalling(forward_points(2000) + rear_points()))
        self.assertEventually(lambda: status() == "STALE", "should go STALE")
        n = len(WORLD.values("Pi/Heartbeat"))
        time.sleep(0.2)
        self.assertGreater(len(WORLD.values("Pi/Heartbeat")), n + 3)


class TestBlindSector(HarnessCase):
    """Bug 3: too few valid forward returns must not read as 'clear'."""

    def test_empty_forward_sector_is_blind_stop(self):
        zeros = [(0, a, 0.0) for _, a, _ in forward_points(0)]
        self.run_monitor(fakes.streaming(zeros + rear_points()))
        self.assertEventually(lambda: status() == "BLIND" and stop() is True,
                              "empty forward sector -> Stop/BLIND")
        self.assertEqual(distance(), -1.0)

    def test_too_few_forward_points_is_blind_stop(self):
        self.run_monitor(fakes.streaming(forward_points(2000, n=3) + rear_points()))
        self.assertEventually(lambda: status() == "BLIND" and stop() is True,
                              "3 < min_sector_points(5) -> Stop/BLIND")

    def test_enough_forward_points_is_ok(self):
        self.run_monitor(fakes.streaming(forward_points(2000, n=5) + rear_points()))
        self.assertEventually(lambda: status() == "OK" and stop() is False,
                              "5 >= min_sector_points -> OK")


class TestHeartbeat(HarnessCase):
    """Bug 4: heartbeat is an incrementing counter published every iteration."""

    def test_heartbeat_increments_every_iteration(self):
        self.run_monitor(fakes.streaming(forward_points(2000) + rear_points()))
        self.assertEventually(lambda: status() == "OK", "should reach OK")
        time.sleep(0.5)
        hb = WORLD.values("Pi/Heartbeat")
        self.assertGreaterEqual(len(hb), 10, "loop_hz=50 for >0.5 s: %r" % hb[:5])
        self.assertEqual(hb, [float(i) for i in range(1, len(hb) + 1)])


class FakeReader:
    """Deterministic reader: test sets latest scan/time directly."""

    def __init__(self):
        self.scan, self.t, self.count, self.err = None, None, 0, None

    def feed(self, scan, t):
        self.scan, self.t, self.count = scan, t, self.count + 1

    def snapshot(self):
        return self.scan, self.t, self.count, self.err


class FakeNT:
    def __init__(self):
        self.v = {}

    def __getattr__(self, name):
        if name.startswith("set_"):
            return lambda val: self.v.__setitem__(name[4:], val)
        raise AttributeError(name)


class TestSafetyMonitorStep(unittest.TestCase):
    """Deterministic (fake clock) checks of the fixed-rate loop body."""

    def setUp(self):
        import safety_monitor
        self.sm = safety_monitor
        self.now = [100.0]
        self.reader, self.nt = FakeReader(), FakeNT()
        self.mon = safety_monitor.SafetyMonitor(
            self.nt, self.reader, dict(fakes.TEST_SAFETY, max_stale_scan_sec=1.0),
            clock=lambda: self.now[0])

    def test_no_scan_yet_is_stale(self):
        d = self.mon.step()
        self.assertEqual((d.stop, d.status), (True, "STALE"))

    def test_reader_stops_producing_goes_stale(self):
        self.reader.feed(forward_points(2000), 100.0)
        self.now[0] = 100.5
        self.assertEqual(self.mon.step().status, "OK")
        self.assertIs(self.nt.v["stop"], False)
        self.now[0] = 101.01          # no new scan for > 1.0 s
        self.assertEqual(self.mon.step().status, "STALE")
        self.assertIs(self.nt.v["stop"], True)
        self.assertEqual(self.nt.v["heartbeat"], 2)

    def test_reader_error_reports_error(self):
        self.reader.err = "RPLidarException: boom"
        self.assertEqual(self.mon.step().status, "ERROR")

    def test_evaluate_scan_units_and_blind(self):
        ev = self.sm.evaluate_scan
        self.assertEqual(ev(forward_points(500), -30, 30, 0.8).status, "OBSTACLE")
        self.assertAlmostEqual(ev(forward_points(500), -30, 30, 0.8).distance_m, 0.5)
        self.assertEqual(ev(forward_points(2000), -30, 30, 0.8).status, "OK")
        self.assertEqual(ev(rear_points(), -30, 30, 0.8).status, "BLIND")
        self.assertEqual(ev(forward_points(2000, n=4), -30, 30, 0.8, 5).status, "BLIND")
        # an obstacle seen by only a couple of points is still an OBSTACLE
        self.assertEqual(ev(forward_points(300, n=2), -30, 30, 0.8, 5).status, "OBSTACLE")

    def test_run_stops_on_event(self):
        ev = threading.Event()
        calls = []

        def fake_sleep(dt):
            calls.append(dt)
            if len(calls) >= 5:
                ev.set()
        self.mon.run(stop_event=ev, sleep=fake_sleep)
        self.assertEqual(self.mon.heartbeat, 5)


class TestLidarReaderThread(unittest.TestCase):
    """LidarReader background thread with a fake device that stalls."""

    def test_background_reader_records_then_goes_stale_and_closes(self):
        import lidar_reader
        release = threading.Event()

        class Dev:
            def iter_scans(self):
                yield forward_points(2000)
                yield forward_points(1500)
                release.wait()
                raise RuntimeError("closed")

            def stop(self):
                release.set()

            def disconnect(self):
                release.set()

        r = lidar_reader.LidarReader("/dev/fake", lidar_factory=lambda *a: Dev())
        r.start()
        self.assertTrue(wait_for(lambda: r.snapshot()[2] == 2))
        self.assertEqual(r.snapshot()[0][0][2], 1500.0)
        self.assertFalse(r.is_stale(0.5))
        self.assertTrue(wait_for(lambda: r.is_stale(0.1), 1.0))
        thread = r._thread
        r.close()
        self.assertFalse(thread.is_alive())


if __name__ == "__main__":
    unittest.main()

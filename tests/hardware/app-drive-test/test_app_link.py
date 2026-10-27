"""Checks for app_link.py with a fake roboRIO (no pyntcore needed): python3 test_app_link.py"""
import json
import os
import sys
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import app_link  # noqa: E402


class FakeNt:
    def __init__(self):
        self.sent = []
        self.values = {"Running Mode": "APP", "FL/AngleDeg": 91.24, "FL/ConfigOK": True,
                       "FL/DriveFault": False, "Result/FL": "PASS"}

    def connected(self):
        return True

    def send(self, cmd):
        self.sent.append(cmd)

    def module_names(self):
        return ["FL"]

    def number(self, key, default=None):
        return self.values.get(key, default)

    boolean = string = number


class LinkTests(unittest.TestCase):
    def setUp(self):
        self.nt = FakeNt()
        self.link = app_link.Link(self.nt)

    def test_drive_publishes_new_seq_each_time(self):
        self.link.drive({"x": 0.5, "y": 0, "sid": "a", "seq": 1})
        self.link.drive({"x": 0.5, "y": 0, "sid": "a", "seq": 2})
        self.assertEqual([c[0] for c in self.nt.sent], [1.0, 2.0])
        self.assertEqual(self.nt.sent[-1][1:], [0.5, 0.0, -1.0])

    def test_stick_clamped_and_junk_zeroed(self):
        self.link.drive({"x": 3, "y": 4})
        self.assertAlmostEqual(self.nt.sent[-1][1], 0.6)
        self.assertAlmostEqual(self.nt.sent[-1][2], 0.8)
        self.link.drive({"x": float("nan"), "y": "fast", "point": float("inf")})
        self.assertEqual(self.nt.sent[-1][1:], [0.0, 0.0, -1.0])

    def test_point_wraps(self):
        self.link.drive({"x": 0, "y": 0, "point": 450})
        self.assertEqual(self.nt.sent[-1][3], 90.0)

    def test_older_drive_refused(self):
        self.link.drive({"x": 0.2, "sid": "a", "seq": 5})
        code, _ = self.link.drive({"x": 0.9, "sid": "a", "seq": 4})
        self.assertEqual(code, 409)
        self.assertEqual(len(self.nt.sent), 1)

    def test_late_drive_after_stop_refused(self):
        self.link.drive({"x": 0.5, "sid": "a", "seq": 1})
        self.link.stop({"sid": "a", "seq": 3})
        code, _ = self.link.drive({"x": 0.5, "sid": "a", "seq": 2})  # was in flight when Stop was pressed
        self.assertEqual(code, 409)
        self.assertEqual(self.nt.sent[-1][1:], [0.0, 0.0, -1.0])

    def test_state(self):
        _, s = self.link.state()
        self.assertEqual(s["mode"], "APP")
        self.assertEqual(s["modules"][0]["angle_deg"], 91.24)
        self.assertEqual(s["modules"][0]["result"], "PASS")


class HttpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nt = FakeNt()
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), app_link.make_handler(app_link.Link(cls.nt), "secret"))
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()
        cls.base = "http://127.0.0.1:%d" % cls.server.server_port

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()

    def req(self, path, body=None, key="secret"):
        data = None if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode())
        r = urllib.request.Request(self.base + path, data=data, headers={"X-Key": key, "Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(r) as resp:
                return resp.status, resp.read()
        except urllib.error.HTTPError as e:
            return e.code, e.read()

    def test_page_served_without_key(self):
        code, body = self.req("/?k=whatever", key="")
        self.assertEqual(code, 200)
        self.assertIn(b"Module drive test", body)

    def test_key_required(self):
        self.assertEqual(self.req("/state", key="nope")[0], 401)
        n = len(self.nt.sent)
        self.assertEqual(self.req("/drive", {"x": 1}, key="nope")[0], 401)
        self.assertEqual(len(self.nt.sent), n)

    def test_drive_and_stop(self):
        self.assertEqual(self.req("/drive", {"x": 0.3, "y": 0})[0], 200)
        self.assertEqual(self.nt.sent[-1][1], 0.3)
        self.assertEqual(self.req("/stop", {})[0], 200)
        self.assertEqual(self.nt.sent[-1][1:], [0.0, 0.0, -1.0])

    def test_bad_body_stops(self):
        self.req("/drive", {"x": 0.3})
        self.assertEqual(self.req("/drive", b"not json")[0], 400)
        self.assertEqual(self.nt.sent[-1][1:], [0.0, 0.0, -1.0])


if __name__ == "__main__":
    unittest.main(verbosity=1)

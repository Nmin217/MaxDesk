#!/usr/bin/env python3
"""Phone -> laptop -> roboRIO link for driving the MAXSwerve modules on the bench.

Test tool only, separate from the main app and bridge. Run it on the laptop that has the Driver
Station, open the printed address on your phone, pick "Phone app (test link)" as the Test Mode and
enable TELEOP. See README.md in this folder.

The phone posts drive commands while a finger is on the pad. Each one is published to the roboRIO
as /AppDrive/cmd = [seq, x, y, pointDeg] with a new seq; the robot program stops everything when
seq stops changing (finger lifted, phone asleep, Wi-Fi dropped, this script killed).
"""
import argparse
import json
import math
import os
import secrets
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
MODULE_FIELDS = {
    "angle_deg": "AngleDeg", "drive_mps": "DriveVelMps", "drive_amps": "DriveAmps",
    "turn_amps": "TurnAmps", "drive_temp_c": "DriveTempC", "turn_temp_c": "TurnTempC",
    "bus_volts": "BusVolts",
}


class NtLink:
    """The roboRIO side, over NetworkTables 4 (pyntcore)."""

    def __init__(self, server, team=None):
        import ntcore  # pip install pyntcore
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startClient4("app-drive-test")
        if team:
            self.inst.setServerTeam(team)
        else:
            self.inst.setServer(server)
        self._sub = ntcore.MultiSubscriber(self.inst, ["/SmartDashboard/"])  # keeps readouts flowing
        self._pub = self.inst.getDoubleArrayTopic("/AppDrive/cmd").publish()
        self.sd = self.inst.getTable("SmartDashboard")

    def connected(self):
        return self.inst.isConnected()

    def send(self, cmd):
        self._pub.set(cmd)
        self.inst.flush()

    def module_names(self):
        return sorted(n for n in self.sd.getSubTables() if self.sd.getSubTable(n).containsKey("AngleDeg"))

    def number(self, key, default=None):
        return self.sd.getNumber(key, default)

    def boolean(self, key, default=None):
        return self.sd.getBoolean(key, default)

    def string(self, key, default=""):
        return self.sd.getString(key, default)


def clamp_stick(x, y):
    """Clamp to the unit circle; anything that isn't a finite number becomes 0."""
    x = x if isinstance(x, (int, float)) and math.isfinite(x) else 0.0
    y = y if isinstance(y, (int, float)) and math.isfinite(y) else 0.0
    m = math.hypot(x, y)
    return (x / m, y / m) if m > 1 else (float(x), float(y))


class Link:
    """Everything except HTTP, so it can be tested with a fake NtLink."""

    def __init__(self, nt):
        self.nt = nt
        self.seq = 0
        self.lock = threading.Lock()
        self.phone_seq = {}  # phone session id -> last seq seen, to drop out-of-order posts

    def drive(self, body):
        sid, pseq = str(body.get("sid", "")), body.get("seq")
        x, y = clamp_stick(body.get("x"), body.get("y"))
        point = body.get("point")
        point = float(point) % 360 if isinstance(point, (int, float)) and math.isfinite(point) else -1.0
        with self.lock:
            if isinstance(pseq, int) and pseq <= self.phone_seq.get(sid, -1):
                return 409, {"ok": False, "status": "older command ignored"}
            if isinstance(pseq, int):
                self.phone_seq[sid] = pseq
            self.seq += 1
            self.nt.send([float(self.seq), x, y, point])
        return 200, {"ok": True, "x": x, "y": y, "point": point}

    def stop(self, body=None):
        body = body or {}
        sid, pseq = str(body.get("sid", "")), body.get("seq")
        with self.lock:
            if isinstance(pseq, int):  # a drive from before this stop that arrives late is refused
                self.phone_seq[sid] = max(pseq, self.phone_seq.get(sid, -1))
            self.seq += 1
            self.nt.send([float(self.seq), 0.0, 0.0, -1.0])
        return 200, {"ok": True, "status": "stopped"}

    def state(self):
        nt = self.nt
        modules = []
        for name in nt.module_names():
            m = {"name": name, "config_ok": nt.boolean(name + "/ConfigOK", False),
                 "drive_fault": nt.boolean(name + "/DriveFault", False),
                 "turn_fault": nt.boolean(name + "/TurnFault", False),
                 "result": nt.string("Result/" + name, "")}
            for k, key in MODULE_FIELDS.items():
                v = nt.number(name + "/" + key, None)
                m[k] = round(v, 3) if isinstance(v, (int, float)) else None
            modules.append(m)
        return 200, {"connected": nt.connected(), "mode": nt.string("Running Mode", ""), "modules": modules}


def make_handler(link, key):
    page = open(os.path.join(HERE, "index.html"), "rb").read()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def reply(self, code, obj):
            data = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def authed(self):
            if secrets.compare_digest(self.headers.get("X-Key", ""), key):
                return True
            self.reply(401, {"ok": False, "status": "wrong or missing key: open the address this script printed"})
            return False

        def do_GET(self):
            if self.path == "/" or self.path.startswith("/?"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)
            elif self.path == "/state":
                if self.authed():
                    self.reply(*link.state())
            else:
                self.reply(404, {"ok": False})

        def do_POST(self):
            if not self.authed():
                return
            try:
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}") if n <= 4096 else None
            except (ValueError, json.JSONDecodeError):
                body = None
            if not isinstance(body, dict):
                link.stop()
                return self.reply(400, {"ok": False, "status": "bad request"})
            if self.path == "/drive":
                self.reply(*link.drive(body))
            elif self.path == "/stop":
                self.reply(*link.stop(body))
            else:
                self.reply(404, {"ok": False})

    return Handler


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--rio", default="172.22.11.2", help="roboRIO address (default: USB, 172.22.11.2)")
    ap.add_argument("--team", type=int, help="find the roboRIO by team number instead of --rio")
    ap.add_argument("--port", type=int, default=8090)
    args = ap.parse_args()
    try:
        nt = NtLink(args.rio, args.team)
    except ImportError:
        sys.exit("pyntcore is missing: pip install -r requirements.txt")
    link = Link(nt)
    key = secrets.token_urlsafe(6)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(link, key))
    print(f"Open on your phone (same Wi-Fi as this laptop):\n  http://{lan_ip()}:{args.port}/?k={key}")
    print("Waiting for the roboRIO..." if not nt.connected() else "roboRIO connected.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        link.stop()
        time.sleep(0.1)


if __name__ == "__main__":
    main()

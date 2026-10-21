#!/usr/bin/env python3
"""NetworkTables (NT4) helper for Summon Desk.

Uses pyntcore (`import ntcore`), the WPILib NT4 client. The legacy
pynetworktables package is NT3-only and its initialize() takes no port.
ntcore is imported lazily so tests can inject a fake module.
"""

TABLE = "SummonDesk"
NT4_DEFAULT_PORT = 5810


class NTClient:
    def __init__(self, server=None, port=NT4_DEFAULT_PORT, team=None,
                 identity="summon-pi", ntcore_module=None):
        if ntcore_module is None:
            try:
                import ntcore as ntcore_module
            except ImportError as e:  # pragma: no cover - depends on install
                raise RuntimeError(
                    "pyntcore is not installed: pip install -r requirements.txt") from e
        if server is None and team is None:
            raise ValueError("networktables needs 'server' or 'team'")
        self.inst = ntcore_module.NetworkTableInstance.getDefault()
        self.inst.startClient4(identity)
        if team is not None:
            self.inst.setServerTeam(int(team), int(port))
        else:
            self.inst.setServer(str(server), int(port))
        self.t = self.inst.getTable(TABLE)

    def is_connected(self):
        return bool(self.inst.isConnected())

    def close(self):
        try:
            self.inst.stopClient()
        except Exception:
            pass

    def set_lidar_status(self, s): self.t.getEntry("LiDAR/Status").setString(str(s))
    def set_obstacle_forward(self, v): self.t.getEntry("Obstacle/Forward").setBoolean(bool(v))
    def set_obstacle_distance(self, m): self.t.getEntry("Obstacle/Distance").setDouble(float(m))
    def set_stop(self, v): self.t.getEntry("Safety/Stop").setBoolean(bool(v))
    def set_movement_request(self, f, s=0.0, r=0.0):
        self.t.getEntry("Movement/Forward").setDouble(float(f))
        self.t.getEntry("Movement/Strafe").setDouble(float(s))
        self.t.getEntry("Movement/Rotation").setDouble(float(r))
    # roboRIO subscribes Pi/Heartbeat as a double and only checks that it changes.
    def set_heartbeat(self, v): self.t.getEntry("Pi/Heartbeat").setDouble(float(v))

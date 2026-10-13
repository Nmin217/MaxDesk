#!/usr/bin/env python3
"""NetworkTables helper for Summon Desk."""
from networktables import NetworkTables

class NTClient:
    def __init__(self, server, port=5810):
        NetworkTables.initialize(server=server, port=port)
        self.t = NetworkTables.getTable("SummonDesk")

    def set_lidar_status(self, s): self.t.getEntry("LiDAR/Status").setString(str(s))
    def set_obstacle_forward(self, v): self.t.getEntry("Obstacle/Forward").setBoolean(bool(v))
    def set_obstacle_distance(self, m): self.t.getEntry("Obstacle/Distance").setDouble(float(m))
    def set_stop(self, v): self.t.getEntry("Safety/Stop").setBoolean(bool(v))
    def set_movement_request(self, f, s=0.0, r=0.0):
        self.t.getEntry("Movement/Forward").setDouble(float(f))
        self.t.getEntry("Movement/Strafe").setDouble(float(s))
        self.t.getEntry("Movement/Rotation").setDouble(float(r))
    def set_heartbeat(self, v): self.t.getEntry("Pi/Heartbeat").setDouble(float(v))

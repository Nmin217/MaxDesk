"""Bug 5: NT4 client via pyntcore; pynetworktables initialize() has no port."""
import unittest

import fakes
from fakes import WORLD


class TestNTClient(unittest.TestCase):
    def setUp(self):
        WORLD.reset()
        WORLD.strict_pynetworktables = True   # behave like real pynetworktables
        import nt_client
        self.nt_client = nt_client

    def test_constructs_nt4_client_with_server_and_port(self):
        self.nt_client.NTClient("10.0.0.2", 5810)
        self.assertIn(("startClient4", "summon-pi"), WORLD.nt_calls)
        self.assertIn(("setServer", "10.0.0.2", 5810), WORLD.nt_calls)

    def test_team_number_uses_set_server_team(self):
        self.nt_client.NTClient(team=1234)
        self.assertIn(("setServerTeam", 1234, 5810), WORLD.nt_calls)

    def test_publish_methods_write_summondesk_topics(self):
        c = self.nt_client.NTClient("10.0.0.2", 5810)
        c.set_stop(True); c.set_lidar_status("OK"); c.set_obstacle_forward(0)
        c.set_obstacle_distance(1); c.set_heartbeat(7)
        c.set_movement_request(0.5, 0.25, -0.1)
        self.assertIs(WORLD.latest("Safety/Stop"), True)
        self.assertEqual(WORLD.latest("LiDAR/Status"), "OK")
        self.assertIs(WORLD.latest("Obstacle/Forward"), False)
        self.assertEqual(WORLD.latest("Obstacle/Distance"), 1.0)
        self.assertEqual(WORLD.latest("Pi/Heartbeat"), 7.0)
        self.assertEqual(WORLD.latest("Movement/Rotation"), -0.1)

    def test_injected_ntcore_module(self):
        c = self.nt_client.NTClient("host", 5810, ntcore_module=fakes.ntcore)
        c.set_stop(False)
        self.assertIs(WORLD.latest("Safety/Stop"), False)


if __name__ == "__main__":
    unittest.main()

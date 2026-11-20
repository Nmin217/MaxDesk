"""robot/pi/spark_can.py: the Pi's SPARK MAX CAN helper (firmware 24), against its pretend SPARKs.

Checks the message format against sparkcan / REV's well-known IDs, the safety rules (no heartbeat unless Node
keeps saying "enabled", zeros on stop, stdin gone = stop), the steering and drive loops, and the guards.
"""
import math
import os
import struct
import subprocess
import sys
import json
import unittest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "robot", "pi"))
import spark_can as sc  # noqa: E402


class Rig:
    """Helper + pretend SPARKs on a fake clock. Node's side is driven by hand."""

    def __init__(self, ids=(1, 2), **sim_kw):
        self.sim = sc.SimBus(ids=ids, **sim_kw)
        self.lines = []
        self.h = sc.Helper(self.sim.send, self.lines.append, lambda: self.sim.t)

    def cfg(self, dev, kind, **kw):
        self.h.command({'op': 'config', 'id': dev, 'kind': kind, **kw})

    def run(self, seconds, every=None):
        for _ in range(int(round(seconds / sc.TICK_S))):
            if every:
                every()
            self.sim.step(sc.TICK_S)
            for can_id, data in self.sim.read_all():
                self.h.frame(can_id, data)
            self.h.tick()

    def sent_since(self, t0, pred):
        return [f for f in self.sim.sent if f[0] >= t0 and pred(f)]


def angle_deg(rig, dev):
    return math.degrees(rig.sim.devs[dev]['angle'])


class TestFormat(unittest.TestCase):
    def test_well_known_ids(self):
        # REV's documented API IDs: duty cycle setpoint 0x2050080, status 0 0x2051800, heartbeat 0x2052C80.
        self.assertEqual(sc.arb_id(sc.API_DUTY_CYCLE, 3), 0x02050083)
        self.assertEqual(sc.arb_id(sc.API_STATUS[0], 7), 0x02051807)
        self.assertEqual(sc.arb_id(sc.API_STATUS[5], 2), 0x02051942)
        self.assertEqual(sc.HEARTBEAT_ID, 0x02052C80)

    def test_param_frame_like_sparkcan(self):
        can_id, data = sc.enc_param(4, sc.P_SMART_STALL_LIMIT, sc.PARAM_TYPE_UINT, 20)
        self.assertEqual(can_id, (2 << 24) | (5 << 16) | (48 << 10) | (59 << 6) | 4)
        self.assertEqual(data, struct.pack('<I', 20) + bytes([1]))
        _, data = sc.enc_param(4, sc.P_DUTY_INVERTED, sc.PARAM_TYPE_BOOL, True)
        self.assertEqual(data, bytes([1, 0, 0, 0, 3]))
        _, data = sc.enc_duty(1, 0.25)
        self.assertEqual(data, struct.pack('<f', 0.25) + b'\x00' * 4)

    def test_status_decoding(self):
        dev, n, f = sc.decode_status(sc.arb_id(sc.API_STATUS[5], 6), struct.pack('<fH2x', 0.25, 0))
        self.assertEqual((dev, n), (6, 5))
        self.assertAlmostEqual(f['abs'], 0.25)
        v = int(12.5 * 128)
        amps = int(10 * 32) << 4
        body = struct.pack('<fB', 1234.5, 41) + bytes([v & 0xFF, (v >> 8) | (amps & 0xF0), amps >> 8])
        _, n, f = sc.decode_status(sc.arb_id(sc.API_STATUS[1], 1), body)
        self.assertEqual(n, 1)
        self.assertAlmostEqual(f['vel'], 1234.5, places=3)
        self.assertEqual(f['temp'], 41)
        self.assertAlmostEqual(f['volts'], 12.5, places=2)
        self.assertAlmostEqual(f['amps'], 10.0, places=2)
        self.assertIsNone(sc.decode_status(0x01234567, bytes(8)))   # someone else's device

    def test_configure_sets_units_limits_and_fast_encoder_frames(self):
        r = Rig(ids=(2,))
        r.cfg(2, 'turn', currentLimitA=20)
        params = r.sim.devs[2]['params']
        self.assertEqual(params[sc.P_SMART_STALL_LIMIT], struct.pack('<I', 20) + b'\x01')
        self.assertEqual(params[sc.P_DUTY_INVERTED], bytes([1, 0, 0, 0, 3]))
        self.assertEqual(params[sc.P_DUTY_POSITION_FACTOR], struct.pack('<f', 1.0) + b'\x02')
        self.assertAlmostEqual(r.sim.devs[2]['p5'], 0.010)
        self.assertNotIn(154, params)   # the jig's zero offset is never touched


class TestSafety(unittest.TestCase):
    def test_no_heartbeat_unless_enabled_and_fresh(self):
        r = Rig()
        r.cfg(1, 'drive')
        r.run(0.2)
        self.assertEqual(r.sent_since(0, lambda f: f[1] == sc.HEARTBEAT_ID), [])
        r.h.command({'op': 'hb', 'on': True})
        r.run(0.05)
        self.assertTrue(r.sent_since(0, lambda f: f[1] == sc.HEARTBEAT_ID))
        # Node stops refreshing: heartbeats stop within ENABLE_FRESH_S, and zeros go out once.
        t_stop = r.sim.t + sc.ENABLE_FRESH_S + 0.02
        r.run(0.3)
        self.assertEqual(r.sent_since(t_stop, lambda f: f[1] == sc.HEARTBEAT_ID), [])
        last_duty = [f for f in r.sim.sent if f[1] == sc.arb_id(sc.API_DUTY_CYCLE, 1)][-1]
        self.assertEqual(struct.unpack_from('<f', last_duty[2])[0], 0.0)

    def test_commands_go_out_before_the_heartbeat(self):
        r = Rig()
        r.cfg(1, 'drive')
        r.h.command({'op': 'hb', 'on': True})
        r.h.command({'op': 'vel', 'id': 1, 'v': 1000})
        r.run(0.01)
        ids = [f[1] for f in r.sim.sent if f[1] in (sc.HEARTBEAT_ID, sc.arb_id(sc.API_DUTY_CYCLE, 1))]
        self.assertEqual(ids[:2], [sc.arb_id(sc.API_DUTY_CYCLE, 1), sc.HEARTBEAT_ID])

    def test_stale_target_means_stop(self):
        r = Rig()
        r.cfg(1, 'drive')
        r.h.command({'op': 'vel', 'id': 1, 'v': 3000})
        r.run(1.0, every=lambda: r.h.command({'op': 'hb', 'on': True}))   # enabled, but the target is never refreshed
        self.assertLess(abs(r.sim.devs[1]['rpm']), 100)

    def test_close_stops_everything(self):
        r = Rig()
        r.cfg(1, 'drive')
        r.run(0.5, every=lambda: (r.h.command({'op': 'hb', 'on': True}), r.h.command({'op': 'vel', 'id': 1, 'v': 3000})))
        self.assertGreater(r.sim.devs[1]['rpm'], 2000)
        r.h.command({'op': 'close'})
        self.assertTrue(r.h.closed)
        last = [f for f in r.sim.sent if f[1] == sc.arb_id(sc.API_DUTY_CYCLE, 1)][-1]
        self.assertEqual(struct.unpack_from('<f', last[2])[0], 0.0)

    def test_rebooted_spark_is_configured_again(self):
        r = Rig(ids=(2,))
        r.cfg(2, 'turn')
        r.run(0.1)
        r.sim.devs[2]['params'].clear()
        r.sim.devs[2]['p5'] = 0.2               # a reboot forgets the fast status 5
        saved = r.sim.devs.pop(2)
        r.run(0.6)                               # silent
        r.sim.devs[2] = saved
        r.run(0.3)
        self.assertIn(sc.P_SMART_STALL_LIMIT, r.sim.devs[2]['params'])
        self.assertAlmostEqual(r.sim.devs[2]['p5'], 0.010)


class TestLoops(unittest.TestCase):
    def test_steering_goes_the_short_way_and_settles(self):
        r = Rig(ids=(2,), angles={2: math.radians(350)})
        r.cfg(2, 'turn')
        target = 20 / 360
        r.run(1.0, every=lambda: (r.h.command({'op': 'hb', 'on': True}), r.h.command({'op': 'pos', 'id': 2, 'v': target})))
        self.assertLess(abs((angle_deg(r, 2) - 20 + 180) % 360 - 180), 2.0)
        self.assertEqual(r.h.devs[2].fault, None)
        rep = r.lines[-1]['dev']['2']
        self.assertAlmostEqual(rep['abs'], target, delta=0.01)

    def test_big_correct_move_never_trips_the_guards(self):
        for target_deg in (170, -170, 90):
            r = Rig(ids=(2,))
            r.cfg(2, 'turn')
            t = (target_deg % 360) / 360
            r.run(1.5, every=lambda: (r.h.command({'op': 'hb', 'on': True}), r.h.command({'op': 'pos', 'id': 2, 'v': t})))
            self.assertIsNone(r.h.devs[2].fault, target_deg)
            self.assertLess(abs((angle_deg(r, 2) - target_deg + 180) % 360 - 180), 2.0, target_deg)

    def test_drive_tracks_rpm(self):
        r = Rig(ids=(1,))
        r.cfg(1, 'drive')
        r.run(1.0, every=lambda: (r.h.command({'op': 'hb', 'on': True}), r.h.command({'op': 'vel', 'id': 1, 'v': 2000})))
        self.assertAlmostEqual(r.sim.devs[1]['rpm'], 2000, delta=150)

    def test_max_duty_caps_the_bench(self):
        r = Rig(ids=(1,))
        r.cfg(1, 'drive', maxDuty=0.15)
        r.run(1.0, every=lambda: (r.h.command({'op': 'hb', 'on': True}), r.h.command({'op': 'vel', 'id': 1, 'v': 5000})))
        self.assertLess(r.sim.devs[1]['rpm'], 0.16 * sc.NEO_FREE_RPM)

    def test_backwards_encoder_trips_the_guard_instead_of_spinning(self):
        for start in (60, 150):
            r = Rig(ids=(2,), angles={2: math.radians(start)}, backwards=(2,))
            r.cfg(2, 'turn')
            r.run(2.0, every=lambda: (r.h.command({'op': 'hb', 'on': True}), r.h.command({'op': 'pos', 'id': 2, 'v': 0.0})))
            self.assertIsNotNone(r.h.devs[2].fault, f'start {start}')
            self.assertIn('backwards', r.lines[-1]['dev']['2']['faults'][-1])
            self.assertEqual(r.sim.devs[2]['duty'], 0.0)

    def test_duty_mode_for_the_direction_check(self):
        r = Rig(ids=(2,))
        r.cfg(2, 'turn')
        r.run(0.3, every=lambda: (r.h.command({'op': 'hb', 'on': True}), r.h.command({'op': 'duty', 'id': 2, 'v': 0.1})))
        self.assertGreater(angle_deg(r, 2), 20)


class TestProcess(unittest.TestCase):
    def test_sim_process_runs_and_exits_when_stdin_closes(self):
        p = subprocess.Popen([sys.executable, '-B', os.path.join(REPO, 'robot', 'pi', 'spark_can.py'), '--sim', '--sim-ids', '1,2'],
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        first = json.loads(p.stdout.readline())
        self.assertTrue(first['ready'])
        p.stdin.write(json.dumps({'op': 'config', 'id': 2, 'kind': 'turn'}) + '\n')
        p.stdin.flush()
        line = json.loads(p.stdout.readline())
        self.assertIn('dev', line)
        p.stdin.close()
        self.assertEqual(p.wait(timeout=5), 0)
        p.stdout.close()


if __name__ == '__main__':
    unittest.main()

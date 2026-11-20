#!/usr/bin/env python3
"""
SPARK MAX CAN helper for the Pi: talks to SPARK MAX controllers on firmware 24 directly over SocketCAN
(a CANable / CANivore showing up as can0), with no roboRIO and no Driver Station.

Only the Python standard library is used (Linux sockets speak CAN natively), so nothing to install.

How it fits:
  node (spark_bus.js)  --JSON lines on stdin-->   this helper  --CAN-->  SPARK MAXes
                       <--JSON lines on stdout--
Node sends targets ("drive this wheel at N rpm", "point this module at R rotations") and an enable heartbeat.
This helper runs the control loops at 100 Hz, close to the bus, so a busy Node process can't make the steering
wobble:
  - steering: a P loop on the absolute (Through Bore) encoder, which the SPARK reports in status frame 5;
  - drive: feed-forward (rpm / free speed, corrected for battery voltage) plus a small P on measured speed;
  - both send plain duty-cycle commands, the one control message sparkcan proves on firmware 24.

Safety, in order of importance:
  1. The SPARKs only drive while they keep receiving the CAN enable heartbeat; on firmware 24 they stop by
     themselves about 100 ms after it stops. This helper sends it ONLY while Node keeps saying "enabled"
     (every tick, at most 100 ms old). Node quits, crashes or freezes -> heartbeat stops -> motors stop.
     stdin closing (Node gone) also makes this helper send zeros and exit.
  2. Runaway guard: a module whose steering moves AWAY from its target while being driven towards it
     (encoder direction wrong, wrong encoder on the wrong SPARK) has its steering shut off until restart.
  3. Current limits are written to every SPARK at start, and again whenever one reboots (brown-out).

The CAN message format comes from sparkcan (MIT, github.com/grayson-arendt/sparkcan), plus status frame 5
(absolute encoder position, float32 in bytes 0-3, REV's documented layout), which sparkcan does not read.

Usage:  python3 spark_can.py --iface can0        (real bus)
        python3 spark_can.py --sim               (built-in pretend SPARKs: try everything with no hardware)
"""

import argparse
import json
import math
import os
import select
import socket
import struct
import sys
import time

# ---------------------------------------------------------------------------------------------------------
# Message format (firmware 24). 29-bit ID: deviceType(5) | manufacturer(8) | apiClass(6) | apiIndex(4) | id(6)
# ---------------------------------------------------------------------------------------------------------
DEVICE_TYPE = 0x02          # motor controller
MANUFACTURER = 0x05         # REV
EFF_FLAG = 0x80000000       # extended (29-bit) frame
EFF_MASK = 0x1FFFFFFF
FRAME = struct.Struct('=IB3x8s')   # Linux struct can_frame: id, length, padding, 8 data bytes

API_DUTY_CYCLE = (0 << 4) | 2
API_HEARTBEAT = (11 << 4) | 2
API_STATUS = {n: (6 << 4) | n for n in range(7)}   # periodic status 0..6 (same ID sets the period)
PARAM_CLASS = 48

PARAM_TYPE_UINT, PARAM_TYPE_FLOAT, PARAM_TYPE_BOOL = 1, 2, 3
P_MOTOR_TYPE = 2
P_IDLE_MODE = 6
P_SMART_STALL_LIMIT = 59
P_SMART_FREE_LIMIT = 60
P_POSITION_FACTOR = 112      # built-in (hall) encoder position: 1 = motor rotations
P_VELOCITY_FACTOR = 113      # built-in encoder velocity: 1 = motor rpm
P_DUTY_POSITION_FACTOR = 139  # absolute encoder position: 1 = rotations
P_DUTY_INVERTED = 141        # MAXSwerve: the output shaft turns opposite to the steering motor -> inverted
# The absolute encoder's zero offset (154) is NOT touched: it is the one set with the jig in REV Hardware Client.

FAULT_BITS = ['brownout', 'overcurrent', 'watchdog reset', 'motor fault', 'sensor fault', 'stall',
              'EEPROM CRC', 'CAN TX', 'CAN RX', 'has reset', 'gate driver fault', 'other fault']
IGNORED_FAULTS = {'has reset'}   # informational: set after every power-up


def arb_id(api, dev):
    """Same arithmetic as sparkcan's CreateArbId (class and index packed as (class << 4) | index)."""
    return (DEVICE_TYPE << 24) | (MANUFACTURER << 16) | ((api >> 4) << 10) | ((api & 0x0F) << 6) | dev


def param_arb_id(param, dev):
    """sparkcan's CreateParamArbId: class 48, parameter number shifted into the index field."""
    return (DEVICE_TYPE << 24) | (MANUFACTURER << 16) | (PARAM_CLASS << 10) | (param << 6) | dev


HEARTBEAT_ID = (DEVICE_TYPE << 24) | (MANUFACTURER << 16) | ((API_HEARTBEAT >> 4) << 10) | ((API_HEARTBEAT & 0x0F) << 6)
HEARTBEAT_DATA = b'\xff' * 8


def enc_duty(dev, duty):
    return arb_id(API_DUTY_CYCLE, dev), struct.pack('<f', duty) + b'\x00' * 4


def enc_param(dev, param, ptype, value):
    if ptype == PARAM_TYPE_FLOAT:
        body = struct.pack('<f', float(value))
    elif ptype == PARAM_TYPE_BOOL:
        body = bytes([1 if value else 0, 0, 0, 0])
    else:
        body = struct.pack('<I', int(value))
    return param_arb_id(param, dev), body + bytes([ptype])


def enc_status_period(dev, n, period_ms):
    return arb_id(API_STATUS[n], dev), struct.pack('<H', int(period_ms))


def decode_status(can_id, data):
    """(device id, status number, fields) for a SPARK periodic status frame, else None."""
    if (can_id >> 16) != ((DEVICE_TYPE << 8) | MANUFACTURER):
        return None
    api = (can_id >> 6) & 0x3FF
    api_class, api_index, dev = api >> 4, api & 0x0F, can_id & 0x3F
    if api_class != 6 or api_index > 6 or len(data) < 8:
        return None
    if api_index == 0:
        faults = struct.unpack_from('<H', data, 2)[0]
        return dev, 0, {'duty': struct.unpack_from('<h', data, 0)[0] / 32768.0, 'faults': faults}
    if api_index == 1:
        return dev, 1, {'vel': struct.unpack_from('<f', data, 0)[0], 'temp': data[4],
                        'volts': (struct.unpack_from('<H', data, 5)[0] & 0x0FFF) / 128.0,
                        'amps': (struct.unpack_from('<H', data, 6)[0] >> 4) / 32.0}
    if api_index == 2:
        return dev, 2, {'pos': struct.unpack_from('<f', data, 0)[0]}
    if api_index == 5:
        return dev, 5, {'abs': struct.unpack_from('<f', data, 0)[0]}
    return dev, api_index, {}


def fault_names(bits):
    return [n for i, n in enumerate(FAULT_BITS) if bits & (1 << i) and n not in IGNORED_FAULTS]


# ---------------------------------------------------------------------------------------------------------
# Tuning (bench values; the Node side can override per device in its config message)
# ---------------------------------------------------------------------------------------------------------
TICK_S = 0.01               # control loop: 100 Hz
ENABLE_FRESH_S = 0.1        # Node's "enabled" must be newer than this, or the heartbeat stops
TARGET_FRESH_S = 0.25       # a target older than this counts as "stop"
STATUS_OUT_S = 0.02         # status lines to Node: 50 Hz
REASSERT_S = 1.0            # re-send status periods this often (they don't survive a SPARK reboot)
SILENT_REBOOT_S = 0.5       # a SPARK silent this long then back = rebooted: configure it again
NEO_FREE_RPM = 5676.0
NOMINAL_V = 12.0
TURN_KP = 0.6               # duty per radian of steering error
TURN_MAX = 0.5              # steering duty cap (the roboRIO bench program used the same)
TURN_DEADBAND_RAD = math.radians(0.5)
DRIVE_KP = 0.3              # extra duty per (rpm error / free rpm)
RUNAWAY_ERR_RAD = math.radians(20)
RUNAWAY_S = 0.12            # error growing (fresh readings) while pushing towards the target this long -> fault
STUCK_S = 1.0               # error still over RUNAWAY_ERR at full steering power this long -> fault


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Device:
    def __init__(self, dev, cfg):
        self.dev = dev
        self.cfg = cfg
        self.kind = cfg.get('kind', 'drive')
        self.seen = None            # last time any status arrived
        self.fields = {}
        self.target = None          # rpm (drive) or rotations (turn) or ('duty', v)
        self.target_at = -1e9
        self.fault = None           # helper-side fault (runaway): output held at 0 until restart
        self.runaway_since = None
        self.stuck_since = None
        self.last_err = None
        self.abs_seq = 0            # counts fresh absolute-encoder readings
        self.used_seq = 0
        self.configured_at = None
        self.duty_out = 0.0


class Helper:
    """All the logic, with the socket, the clock and the Node pipe passed in, so tests can drive it."""

    def __init__(self, send_frame, emit, now):
        self.send_frame = send_frame   # (can_id_without_flag, data_bytes) -> None
        self.emit = emit               # (dict) -> None, one JSON line to Node
        self.now = now
        self.devs = {}
        self.enabled_at = -1e9
        self.last_status_out = -1e9
        self.last_reassert = -1e9
        self.closed = False
        self.was_enabled = False

    # ---- from Node ----
    def command(self, msg):
        op = msg.get('op')
        t = self.now()
        if op == 'config':
            d = Device(int(msg['id']), msg)
            self.devs[d.dev] = d
            self.configure(d)
        elif op == 'hb':
            self.enabled_at = t if msg.get('on') else -1e9
            if not msg.get('on'):
                self.stop_outputs()
        elif op in ('vel', 'pos', 'duty'):
            d = self.devs.get(int(msg['id']))
            if d is None:
                return
            v = float(msg.get('v', 0))
            if not math.isfinite(v):
                v = 0.0
            d.target = ('duty', max(-1.0, min(1.0, v))) if op == 'duty' else v
            d.target_at = t
        elif op == 'reset':   # clear helper-side faults (runaway) on all devices
            for d in self.devs.values():
                d.fault, d.runaway_since, d.stuck_since, d.last_err = None, None, None, None
        elif op == 'close':
            self.close()

    def configure(self, d):
        c, dev = d.cfg, d.dev
        limit = int(c.get('currentLimitA', 40 if d.kind == 'drive' else 20))
        for frame in (
            enc_param(dev, P_MOTOR_TYPE, PARAM_TYPE_UINT, 1),              # brushless (NEO / NEO 550)
            enc_param(dev, P_IDLE_MODE, PARAM_TYPE_UINT, 1 if c.get('idleMode', 'brake') == 'brake' else 0),
            enc_param(dev, P_SMART_STALL_LIMIT, PARAM_TYPE_UINT, limit),
            enc_param(dev, P_SMART_FREE_LIMIT, PARAM_TYPE_UINT, limit),
            enc_param(dev, P_POSITION_FACTOR, PARAM_TYPE_FLOAT, 1.0),       # undo any roboRIO-burned units
            enc_param(dev, P_VELOCITY_FACTOR, PARAM_TYPE_FLOAT, 1.0),
        ):
            self.send_frame(*frame)
        if d.kind == 'turn':
            self.send_frame(*enc_param(dev, P_DUTY_POSITION_FACTOR, PARAM_TYPE_FLOAT, 1.0))
            self.send_frame(*enc_param(dev, P_DUTY_INVERTED, PARAM_TYPE_BOOL, c.get('encoderInverted', True)))
        self.set_periods(d)
        d.configured_at = self.now()

    def set_periods(self, d):
        self.send_frame(*enc_status_period(d.dev, 1, 20))
        self.send_frame(*enc_status_period(d.dev, 2, 20))
        if d.kind == 'turn':
            self.send_frame(*enc_status_period(d.dev, 5, 10))   # default is 200 ms: far too slow to steer on

    # ---- from CAN ----
    def frame(self, can_id, data):
        r = decode_status(can_id & EFF_MASK, data)
        if not r:
            return
        dev, n, fields = r
        d = self.devs.get(dev)
        if d is None:
            return
        t = self.now()
        if d.seen is not None and t - d.seen > SILENT_REBOOT_S:
            self.configure(d)   # it went quiet and came back: probably rebooted, settings lost
        d.seen = t
        d.fields.update(fields)
        if n == 5:
            d.abs_seq += 1

    # ---- every tick ----
    def tick(self):
        t = self.now()
        enabled = t - self.enabled_at <= ENABLE_FRESH_S
        if enabled:
            for d in self.devs.values():
                self.control(d, t)
            # Commands first, then the heartbeat: a SPARK never gets enabled with a stale command still loaded.
            self.send_frame(HEARTBEAT_ID, HEARTBEAT_DATA)
        elif self.was_enabled:
            self.stop_outputs()   # "enabled" went stale (Node froze?): zero everything, no more heartbeats
        self.was_enabled = enabled
        if t - self.last_reassert >= REASSERT_S:
            self.last_reassert = t
            for d in self.devs.values():
                self.set_periods(d)
        if t - self.last_status_out >= STATUS_OUT_S:
            self.last_status_out = t
            self.emit({'t': round(t, 4), 'enabled': enabled, 'dev': {str(d.dev): self.report(d, t) for d in self.devs.values()}})
        return enabled

    def control(self, d, t):
        out = 0.0
        fresh = t - d.target_at <= TARGET_FRESH_S
        if d.fault is None and fresh and d.target is not None:
            if isinstance(d.target, tuple):
                out = d.target[1]
            elif d.kind == 'turn':
                out = self.steer(d, t)
            else:
                out = self.drive(d)
        d.duty_out = out
        self.send_frame(*enc_duty(d.dev, out))

    def steer(self, d, t):
        a = d.fields.get('abs')
        if a is None or not math.isfinite(a) or d.seen is None or t - d.seen > 0.1:
            return 0.0
        err = wrap_pi((d.target - a) * 2 * math.pi)
        out = 0.0 if abs(err) < TURN_DEADBAND_RAD else max(-TURN_MAX, min(TURN_MAX, TURN_KP * err))
        # Two guards, judged only on fresh readings (status 5 comes every 10 ms, the loop runs every 10 ms):
        #  - runaway: pushing towards the target, yet the error keeps growing -> the encoder counts backwards;
        #  - stuck: still far off at full steering power for STUCK_S -> jammed (or backwards and parked 180 off).
        if d.abs_seq != d.used_seq:
            d.used_seq = d.abs_seq
            far = abs(err) > RUNAWAY_ERR_RAD and abs(out) > 0.1
            if far and d.last_err is not None and abs(err) > abs(d.last_err) + 1e-4:
                d.runaway_since = d.runaway_since if d.runaway_since is not None else t
            elif d.last_err is None or abs(err) < abs(d.last_err) - 1e-4 or not far:
                d.runaway_since = None
            if abs(err) > RUNAWAY_ERR_RAD and abs(out) >= TURN_MAX * 0.9:
                d.stuck_since = d.stuck_since if d.stuck_since is not None else t
            else:
                d.stuck_since = None
            d.last_err = err
        if d.runaway_since is not None and t - d.runaway_since > RUNAWAY_S:
            d.fault = 'steering runs away from its target: the absolute encoder reads backwards'
        elif d.stuck_since is not None and t - d.stuck_since > STUCK_S:
            d.fault = 'steering cannot reach its target: jammed, or the absolute encoder reads backwards'
        if d.fault:
            return 0.0
        return out

    def drive(self, d):
        want = max(-NEO_FREE_RPM, min(NEO_FREE_RPM, d.target))
        volts = d.fields.get('volts') or NOMINAL_V
        comp = NOMINAL_V / max(8.0, volts)
        out = want / NEO_FREE_RPM * comp
        vel = d.fields.get('vel')
        if vel is not None and math.isfinite(vel) and want != 0:
            out += DRIVE_KP * (want - vel) / NEO_FREE_RPM
        cap = float(d.cfg.get('maxDuty', 1.0))
        return max(-cap, min(cap, out))

    def report(self, d, t):
        f = d.fields
        faults = fault_names(f.get('faults', 0))
        if d.fault:
            faults.append(d.fault)
        r = {'age': None if d.seen is None else round((t - d.seen) * 1000, 1),
             'vel': f.get('vel'), 'pos': f.get('pos'), 'amps': f.get('amps'), 'temp': f.get('temp'),
             'volts': f.get('volts'), 'duty': round(d.duty_out, 4), 'faults': faults}
        if d.kind == 'turn':
            r['abs'] = f.get('abs')
        return r

    def stop_outputs(self):
        for d in self.devs.values():
            d.duty_out = 0.0
            self.send_frame(*enc_duty(d.dev, 0.0))

    def close(self):
        if not self.closed:
            self.closed = True
            self.enabled_at = -1e9
            self.stop_outputs()   # and no more heartbeats: the SPARKs disable within ~100 ms


# ---------------------------------------------------------------------------------------------------------
# Pretend SPARKs (--sim, and the tests): enough physics to show the loops work and the guards trip
# ---------------------------------------------------------------------------------------------------------
class SimBus:
    """Fake SPARK MAXes answering on a pretend CAN bus. Firmware-like rules: no heartbeat for 100 ms -> output 0."""

    def __init__(self, ids=(), angles=None, backwards=()):
        self.t = 0.0
        self.hb_at = -1e9
        self.out = []      # frames from the SPARKs, waiting to be read
        self.devs = {i: {'duty': 0.0, 'rpm': 0.0, 'rot': 0.0, 'angle': (angles or {}).get(i, 0.0),
                         'p5': 0.2, 'last5': -1e9, 'last1': -1e9, 'last0': -1e9, 'last2': -1e9,
                         'backwards': i in backwards, 'params': {}} for i in ids}
        self.sent = []     # every frame the helper sent, for tests

    def send(self, can_id, data):
        self.sent.append((self.t, can_id, bytes(data)))
        if can_id == HEARTBEAT_ID:
            self.hb_at = self.t
            return
        dev = can_id & 0x3F
        d = self.devs.get(dev)
        if d is None:
            return
        api = (can_id >> 6) & 0x3FF
        if api == API_DUTY_CYCLE:
            d['duty'] = struct.unpack_from('<f', data, 0)[0]
        elif api >> 4 == 6 and len(data) == 2 and (api & 0x0F) == 5:
            d['p5'] = struct.unpack('<H', data)[0] / 1000.0
        elif ((can_id >> 10) & 0x3F) >= PARAM_CLASS and len(data) == 5:
            d['params'][(api - (PARAM_CLASS << 4))] = data

    def step(self, dt):
        self.t += dt
        enabled = self.t - self.hb_at <= 0.1
        for i, d in self.devs.items():
            duty = d['duty'] if enabled else 0.0
            # drive: speed follows duty with a 0.08 s lag; steering: angle rate proportional to duty
            d['rpm'] += (duty * NEO_FREE_RPM - d['rpm']) * min(1.0, dt / 0.08)
            d['rot'] += d['rpm'] / 60.0 * dt
            rate = duty * 25.0 * (-1 if d['backwards'] else 1)        # rad/s at full duty
            d['angle'] = (d['angle'] + rate * dt) % (2 * math.pi)
            if self.t - d['last0'] >= 0.01:
                d['last0'] = self.t
                self.out.append((arb_id(API_STATUS[0], i), struct.pack('<hHHBB', int(duty * 32767), 0, 0, 0, 2)))
            if self.t - d['last1'] >= 0.02:
                d['last1'] = self.t
                v = int(12.4 * 128) & 0x0FFF
                amps = int(min(80, 1.5 + abs(duty) * 10) * 32) << 4
                body = struct.pack('<fB', d['rpm'], 30) + bytes([v & 0xFF, ((v >> 8) & 0x0F) | (amps & 0xF0), (amps >> 8) & 0xFF])
                self.out.append((arb_id(API_STATUS[1], i), body))
            if self.t - d['last2'] >= 0.02:
                d['last2'] = self.t
                self.out.append((arb_id(API_STATUS[2], i), struct.pack('<fI', d['rot'], 0)))
            if self.t - d['last5'] >= d['p5']:
                d['last5'] = self.t
                self.out.append((arb_id(API_STATUS[5], i), struct.pack('<fH2x', d['angle'] / (2 * math.pi), 0)))

    def read_all(self):
        frames, self.out = self.out, []
        return frames


# ---------------------------------------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--iface', default='can0')
    ap.add_argument('--sim', action='store_true', help='pretend SPARKs, no CAN hardware')
    ap.add_argument('--sim-ids', default='1,2,3,4,5,6,7,8')
    args = ap.parse_args(argv)

    out = sys.stdout

    def emit(obj):
        out.write(json.dumps(obj, separators=(',', ':')) + '\n')
        out.flush()

    sim = None
    sock = None
    if args.sim:
        sim = SimBus(ids=[int(x) for x in args.sim_ids.split(',') if x])
        send = sim.send
    else:
        try:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.bind((args.iface,))
        except OSError as e:
            emit({'error': f'cannot open CAN interface {args.iface}: {e}. Is the adapter plugged in and up '
                           f'(sudo ip link set {args.iface} up type can bitrate 1000000)?'})
            return 2
        sock.setblocking(False)

        def send(can_id, data):
            try:
                sock.send(FRAME.pack(can_id | EFF_FLAG, len(data), data.ljust(8, b'\x00')))
            except OSError:
                pass   # TX buffer full / bus off: the next tick tries again; silence also stops the motors

    h = Helper(send, emit, time.monotonic)
    stdin = sys.stdin.buffer.raw if hasattr(sys.stdin.buffer, 'raw') else sys.stdin.buffer
    os.set_blocking(stdin.fileno(), False)
    pending = b''
    next_tick = time.monotonic()
    emit({'ready': True, 'sim': bool(sim)})
    while not h.closed:
        wait = max(0.0, next_tick - time.monotonic())
        watch = [stdin] + ([sock] if sock else [])
        ready, _, _ = select.select(watch, [], [], wait)
        if stdin in ready:
            chunk = os.read(stdin.fileno(), 65536)
            if not chunk:   # Node is gone: stop everything
                h.close()
                break
            pending += chunk
            *lines, pending = pending.split(b'\n')
            for line in lines:
                if line.strip():
                    try:
                        h.command(json.loads(line))
                    except (ValueError, KeyError, TypeError) as e:
                        emit({'error': f'bad command: {e}'})
        if sock is not None and sock in ready:
            while True:
                try:
                    raw = sock.recv(16)
                except BlockingIOError:
                    break
                can_id, n, data = FRAME.unpack(raw)
                if can_id & EFF_FLAG:
                    h.frame(can_id & EFF_MASK, data[:n])
        now = time.monotonic()
        if now >= next_tick:
            if sim:
                sim.step(TICK_S)
                for can_id, data in sim.read_all():
                    h.frame(can_id, data)
            h.tick()
            next_tick += TICK_S
            if now - next_tick > 0.2:   # fell far behind (machine paused): don't try to catch up
                next_tick = now + TICK_S
    return 0


if __name__ == '__main__':
    import signal
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)   # Ctrl+C: stop quietly; no heartbeat from here on, so the SPARKs disable within ~100 ms

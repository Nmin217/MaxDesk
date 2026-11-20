'use strict';

/**
 * Xbox controller on the Pi, read straight from Linux (/dev/input/js0), so no Driver Station or laptop is needed.
 * Plug the controller into the Pi by USB (Linux's built-in xpad driver). Bluetooth pads usually show up the same
 * way; if the buttons come out shuffled, pass another mapping.
 *
 *   const pad = createXbox();
 *   pad.connected          false when unplugged: treat as "let go of everything"
 *   pad.axis('lx')          -1..1, stick right / down = + (Linux convention, same as WPILib)
 *   pad.button('rb')        true while held
 *   pad.pressed('start')    true once per press (edge)
 *
 * Unplugging the controller ends the read with an error: every button reads released and every axis 0 from then
 * on, until it is plugged back in (checked once a second). So the RB deadman lets go by itself.
 */

const fs = require('fs');

// Linux xpad driver (wired Xbox One / Series / 360 pads).
const XPAD = {
  axes: { lx: 0, ly: 1, lt: 2, rx: 3, ry: 4, rt: 5, dx: 6, dy: 7 },
  buttons: { a: 0, b: 1, x: 2, y: 3, lb: 4, rb: 5, back: 6, start: 7, guide: 8, ls: 9, rs: 10 }
};

const JS_EVENT_BUTTON = 0x01, JS_EVENT_AXIS = 0x02, JS_EVENT_INIT = 0x80;

/** Linux joystick events: 8 bytes each (u32 time ms, s16 value, u8 type, u8 number). Leftover bytes are returned. */
function parseEvents(buf) {
  const events = [];
  let off = 0;
  for (; off + 8 <= buf.length; off += 8) {
    events.push({ time: buf.readUInt32LE(off), value: buf.readInt16LE(off + 4), type: buf[off + 6] & ~JS_EVENT_INIT, number: buf[off + 7] });
  }
  return { events, rest: buf.subarray(off) };
}

function createXbox({ device = '/dev/input/js0', mapping = XPAD, fsImpl = fs, retryMs = 1000, log = () => {} } = {}) {
  const axisByNum = new Map(Object.entries(mapping.axes).map(([k, v]) => [v, k]));
  const buttonByNum = new Map(Object.entries(mapping.buttons).map(([k, v]) => [v, k]));
  let axes = {}, buttons = {}, edges = new Set();
  let connected = false, stream = null, rest = Buffer.alloc(0), timer = null, closed = false;

  function clear() { axes = {}; buttons = {}; edges = new Set(); }

  function apply(e) {
    if (e.type === JS_EVENT_AXIS) {
      const name = axisByNum.get(e.number);
      if (name) axes[name] = Math.max(-1, e.value / 32767);
    } else if (e.type === JS_EVENT_BUTTON) {
      const name = buttonByNum.get(e.number);
      if (!name) return;
      const down = e.value !== 0;
      if (down && !buttons[name]) edges.add(name);
      buttons[name] = down;
    }
  }

  function feed(chunk) {
    const r = parseEvents(Buffer.concat([rest, chunk]));
    rest = Buffer.from(r.rest);
    for (const e of r.events) apply(e);
  }

  function lost(why) {
    if (connected) log(`controller disconnected (${why})`);
    connected = false; stream = null; clear();
    if (!closed && !timer) timer = setTimeout(() => { timer = null; open(); }, retryMs);
  }

  function open() {
    if (closed) return;
    let s;
    try { s = fsImpl.createReadStream(device); } catch (e) { lost(e.message); return; }
    stream = s;
    s.on('open', () => { connected = true; rest = Buffer.alloc(0); clear(); log(`controller connected (${device})`); });
    s.on('data', feed);
    s.on('error', e => lost(e.code || e.message));
    s.on('close', () => { if (stream === s) lost('closed'); });
  }

  open();

  return {
    get connected() { return connected; },
    axis(name) { return connected && Number.isFinite(axes[name]) ? axes[name] : 0; },
    button(name) { return connected && !!buttons[name]; },
    pressed(name) { const was = connected && edges.has(name); edges.delete(name); return was; },
    close() { closed = true; if (timer) clearTimeout(timer); if (stream && stream.destroy) stream.destroy(); },
    _feed: feed   // tests
  };
}

module.exports = { createXbox, parseEvents, XPAD };

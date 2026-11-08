'use strict';

/**
 * Black box recorder: keeps the last `seconds` of what the desk did in memory and saves it to a file when
 * something goes wrong, so a fault on the real desk can be looked at afterwards instead of guessed at.
 * View a saved file with tools/blackbox_viewer.html.
 *
 * - record(sample) is called every control tick; it keeps one sample per 1/hz s (20 Hz by default).
 * - keyframe(points) stores what the LiDAR saw (map points, metres) at most once per keyframeEvery s.
 * - event(text) notes something that happened (a status message change, a trigger) with its time.
 * - trigger(reason) asks for a save. Recording carries on for `after` s so the file shows what happened
 *   next (did it stop cleanly?), then the file is written. Triggers within `quiet` s of a save are the same
 *   incident: they are added as events, not saved again.
 * - Files: <dir>/<date>-<time>-<reason>.json, written atomically (temp file, fsync, rename), newest `keep` kept.
 *
 * Time comes from nowMs(), so tests run on a fake clock. Nothing here touches the motors: a failure to
 * write a file is logged in the recorder's own state and never reaches the control loop.
 */

const fs = require('fs');
const path = require('path');

function createBlackBox({
  dir, seconds = 60, hz = 20, after = 5, quiet = 10, keep = 20, keyframeEvery = 1,
  nowMs = () => Date.now(), wallClock = () => new Date()
} = {}) {
  const maxSamples = Math.ceil(seconds * hz), period = 1000 / hz;
  let samples = [], keyframes = [], events = [];
  let lastSampleAt = -Infinity, lastKeyAt = -Infinity, pending = null, lastSaveAt = -Infinity;
  const saved = [];          // files written this run (newest last)
  let lastError = null;

  const trimOld = (now) => {
    const cutoff = now - (seconds + after) * 1000;
    if (samples.length > maxSamples + Math.ceil(after * hz)) samples = samples.slice(-(maxSamples + Math.ceil(after * hz)));
    keyframes = keyframes.filter(k => k.ms >= cutoff);
    events = events.filter(e => e.ms >= cutoff);
  };

  function write(reason, now) {
    if (!dir) return null;
    const start = now - (seconds + after) * 1000;
    const t0 = samples.length ? samples[0].ms : now;
    const rel = ms => +((ms - t0) / 1000).toFixed(3);
    const doc = {
      kind: 'maxdesk-blackbox', version: 1, reason, saved_at: wallClock().toISOString(),
      trigger_t: rel(pending ? pending.at : now), seconds, hz,
      events: events.filter(e => e.ms >= start).map(e => ({ t: rel(e.ms), text: e.text })),
      keyframes: keyframes.filter(k => k.ms >= start).map(k => ({ t: rel(k.ms), points: k.points })),
      samples: samples.map(s => ({ t: rel(s.ms), ...s.data }))
    };
    const stamp = wallClock().toISOString().replace(/[-:]/g, '').replace('T', '-').slice(0, 15);
    const name = `${stamp}-${String(reason).toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '').slice(0, 40) || 'event'}.json`;
    try {
      fs.mkdirSync(dir, { recursive: true });
      const f = path.join(dir, name), tmp = f + '.tmp';
      const fd = fs.openSync(tmp, 'w');
      try { fs.writeSync(fd, JSON.stringify(doc)); fs.fsyncSync(fd); } finally { fs.closeSync(fd); }
      fs.renameSync(tmp, f);
      saved.push(f);
      const all = fs.readdirSync(dir).filter(n => n.endsWith('.json')).sort();   // names sort by time
      for (const old of all.slice(0, Math.max(0, all.length - keep))) { try { fs.unlinkSync(path.join(dir, old)); } catch (_) {} }
      return f;
    } catch (e) { lastError = String(e && e.message || e); return null; }
  }

  return {
    record(data) {
      const now = nowMs();
      if (now - lastSampleAt >= period - 1e-6) { samples.push({ ms: now, data }); lastSampleAt = now; }
      if (pending && now >= pending.due) {
        const f = write(pending.reason, now);
        lastSaveAt = now; pending = null;
        if (f) events.push({ ms: now, text: 'Saved ' + path.basename(f) });
      }
      trimOld(now);
    },
    keyframe(points) {
      const now = nowMs();
      if (now - lastKeyAt < keyframeEvery * 1000 - 1e-6) return;
      lastKeyAt = now; keyframes.push({ ms: now, points });
    },
    event(text) { events.push({ ms: nowMs(), text: String(text) }); },
    trigger(reason) {
      const now = nowMs();
      events.push({ ms: now, text: 'Trigger: ' + reason });
      if (pending || now - lastSaveAt < quiet * 1000) return false;   // same incident
      pending = { reason, at: now, due: now + after * 1000 };
      return true;
    },
    // Save right now (shutting down): whatever is pending, or the given reason.
    flush(reason = 'shutdown') { const now = nowMs(); const r = pending ? pending.reason : reason; const f = write(r, now); pending = null; lastSaveAt = now; return f; },
    get pending() { return pending ? { ...pending } : null; },
    get saved() { return saved.slice(); },
    get lastError() { return lastError; },
    get size() { return { samples: samples.length, keyframes: keyframes.length, events: events.length }; }
  };
}

module.exports = { createBlackBox };

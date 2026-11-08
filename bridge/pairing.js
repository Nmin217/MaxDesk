'use strict';
/**
 * Pairing: which phones may control the desk.
 *
 * The bridge has one secret key. Phones get it by pairing: typing a 6-digit code that the desk software shows
 * (in its window/log, or on a phone that is already paired). Every API call then carries the key; anything else
 * gets 401. The key is made at random the first time and kept in <dataDir>/bridge-token.txt (readable only by
 * this user); delete that file and restart to unpair every phone. SUMMON_TOKEN overrides it (tests, dev).
 *
 * Guessing protection: 5 wrong codes in a minute blocks that address for the rest of the minute; 20 wrong codes in
 * total switch pairing off until the bridge restarts. A code lasts 10 minutes. Comparisons are constant-time.
 */
const crypto = require('crypto');
const fs = require('fs');
const path = require('path');

function loadOrCreateToken(dataDir) {
  const file = path.join(dataDir, 'bridge-token.txt');
  try { const t = fs.readFileSync(file, 'utf8').trim(); if (t.length >= 16) return { token: t, file, created: false }; } catch (_) {}
  const token = crypto.randomBytes(24).toString('base64url');   // 192 bits
  fs.mkdirSync(dataDir, { recursive: true });
  const tmp = file + '.tmp';
  fs.writeFileSync(tmp, token + '\n', { mode: 0o600 });
  fs.renameSync(tmp, file);
  return { token, file, created: true };
}

const same = (a, b) => { const x = Buffer.from(String(a)), y = Buffer.from(String(b)); return x.length === y.length && crypto.timingSafeEqual(x, y); };

function createPairing({ dataDir, envToken = '', now = () => Date.now(), codeTtlMs = 10 * 60 * 1000,
  perAddressLimit = 5, totalLimit = 20, randomCode = () => String(crypto.randomInt(0, 1000000)).padStart(6, '0') } = {}) {
  const fromEnv = !!envToken;
  const t = fromEnv ? { token: envToken, file: null, created: false } : loadOrCreateToken(dataDir);
  let code = null, codeUntil = 0, totalFails = 0;
  const fails = new Map();   // address -> times of recent wrong codes

  return {
    get token() { return t.token; },
    get tokenFile() { return t.file; },
    get created() { return t.created; },
    get fromEnv() { return fromEnv; },
    /** True for "Bearer <key>". */
    checkAuth(header) { return same(header || '', 'Bearer ' + t.token); },
    /** A fresh code, valid for codeTtlMs. Replaces any older one. */
    newCode() { code = randomCode(); codeUntil = now() + codeTtlMs; return { code, expires_in_s: Math.round(codeTtlMs / 1000) }; },
    get code() { return code && now() < codeUntil ? code : null; },
    /** { status, body }: 200 { token } for the right code; 401 wrong; 410 no code; 429 slow down; 403 switched off. */
    pair(given, address = '') {
      const ms = now();
      if (totalFails >= totalLimit) return { status: 403, body: { paired: false, status: 'Pairing is switched off after too many wrong codes. Restart the desk software to try again.' } };
      const recent = (fails.get(address) || []).filter(x => ms - x < 60000);
      fails.set(address, recent);
      if (recent.length >= perAddressLimit) return { status: 429, body: { paired: false, status: 'Too many wrong codes. Wait a minute and try again.' } };
      if (!code || ms >= codeUntil) return { status: 410, body: { paired: false, status: 'No pairing code is active. Get a new one from the desk software or a paired phone.' } };
      if (typeof given === 'string' && /^\d{6}$/.test(given) && same(given, code)) return { status: 200, body: { paired: true, token: t.token } };
      recent.push(ms); totalFails++;
      return { status: 401, body: { paired: false, status: 'Wrong code. Check the 6 digits and try again.' } };
    }
  };
}

module.exports = { createPairing, loadOrCreateToken };

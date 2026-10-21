#!/usr/bin/env bash
set -u

PORT=8798
BASE="http://127.0.0.1:${PORT}"
AUTH="Authorization: Bearer verify"
LOG=/tmp/wp5_bridge.log
TMP=/tmp/wp5_bridge_test
mkdir -p "$TMP"

BP=
cleanup() {
  if [ -n "${BP:-}" ]; then
    kill "$BP" 2>/dev/null || true
    wait "$BP" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

start_bridge() {
  PORT=$PORT SUMMON_TOKEN=verify node bridge/bridge_sim.js >"$LOG" 2>&1 &
  BP=$!
  for _ in $(seq 1 30); do
    if curl -fsS "$BASE/v1/health" >"$TMP/health.json" 2>/dev/null; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

post() {
  local url=$1 body=$2 output=$3
  curl -sS -o "$output" -w '%{http_code}' \
    -H "$AUTH" -H 'Content-Type: application/json' \
    -X POST -d "$body" "$BASE$url"
}

json_check() {
  python3 - "$1" "$2" <<'PY'
import json, sys
data = json.load(open(sys.argv[1]))
kind = sys.argv[2]
if kind == "poll_once":
    first = data
    second = json.load(open(sys.argv[3]))
    ok = (
        isinstance(first.get("commands"), list)
        and len(first["commands"]) == 1
        and first["commands"][0].get("type") == "summon"
        and isinstance(second.get("commands"), list)
        and len(second["commands"]) == 0
    )
elif kind == "stale":
    ok = data.get("accepted") is False and data.get("stale_epoch") is True
elif kind == "accepted":
    ok = data.get("accepted") is True
else:
    ok = False
sys.exit(0 if ok else 1)
PY
}

bridge_dup_poll=false
bridge_restart_stale_epoch=false
bridge_restart_current_epoch=false

if start_bridge; then
  # Register a browser simulator so the headless loop does not drain the queue.
  curl -fsS -H "$AUTH" -H 'Content-Type: application/json' \
    -X POST -d '{}' "$BASE/v1/sim/state" >"$TMP/initial-state.json" || :

  # Queue a Summon and verify the poll endpoint drains it only once.
  code=$(post /v1/summon '{"x":14,"y":8.6}' "$TMP/summon.json") || code=000
  if [ "$code" = 200 ] &&
     python3 - "$TMP/summon.json" <<'PY'
import json, sys
sys.exit(0 if json.load(open(sys.argv[1])).get("accepted") is True else 1)
PY
  then
    curl -fsS -H "$AUTH" "$BASE/v1/sim/poll" >"$TMP/poll1.json" || :
    curl -fsS -H "$AUTH" "$BASE/v1/sim/poll" >"$TMP/poll2.json" || :
    if python3 - "$TMP/poll1.json" "$TMP/poll2.json" <<'PY'
import json, sys
a, b = (json.load(open(p)) for p in sys.argv[1:])
ok = (len(a.get("commands", [])) == 1
      and a["commands"][0].get("type") == "summon"
      and len(b.get("commands", [])) == 0)
sys.exit(0 if ok else 1)
PY
    then bridge_dup_poll=true; fi
  fi

  # Advance the old process epoch so it differs from the restarted process.
  curl -fsS -H "$AUTH" -H 'Content-Type: application/json' \
    -X POST -d '{}' "$BASE/v1/sim/state" >"$TMP/state.json" || :
  curl -fsS "$BASE/v1/health" >"$TMP/old-health.json" || :
  old_epoch=$(python3 -c 'import json;print(json.load(open("'"$TMP/old-health.json"'"))["epoch"])' 2>/dev/null || echo 0)

  cleanup
  BP=
  if start_bridge; then
    curl -fsS "$BASE/v1/health" >"$TMP/new-health.json" || :
    current_epoch=$(python3 -c 'import json;print(json.load(open("'"$TMP/new-health.json"'"))["epoch"])' 2>/dev/null || echo 0)

    code=$(post /v1/summon \
      "{\"x\":14,\"y\":8.6,\"epoch\":$old_epoch}" "$TMP/stale.json") || code=000
    if [ "$code" = 409 ] &&
       python3 - "$TMP/stale.json" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
sys.exit(0 if d.get("stale_epoch") is True else 1)
PY
    then bridge_restart_stale_epoch=true; fi

    code=$(post /v1/summon \
      "{\"x\":14,\"y\":8.6,\"epoch\":$current_epoch}" "$TMP/current.json") || code=000
    if [ "$code" = 200 ] &&
       python3 - "$TMP/current.json" <<'PY'
import json, sys
sys.exit(0 if json.load(open(sys.argv[1])).get("accepted") is True else 1)
PY
    then bridge_restart_current_epoch=true; fi
  fi
fi

# Regression checks on a fresh bridge: stop coalescing, nav guard reset, token exposure, headless summon checks.
bridge_stop_coalesce_purges=false
bridge_stop_resets_nav_guard=false
bridge_sim_token_loopback=false
bridge_sim_token_lan_hidden=false
cleanup
BP=
if start_bridge; then
  curl -fsS -H "$AUTH" -H 'Content-Type: application/json' -X POST -d '{}' "$BASE/v1/sim/state" >/dev/null || :
  # 1. Stop, Drive, Stop within 250 ms: the second Stop must still purge the Drive and answer 200.
  post /v1/stop '{}' "$TMP/stop1.json" >/dev/null
  post /v1/drive '{"x":1,"y":0,"turn":0,"speed":0.5}' "$TMP/drive.json" >/dev/null
  code=$(post /v1/stop '{}' "$TMP/stop2.json") || code=000
  curl -fsS -H "$AUTH" "$BASE/v1/sim/poll" >"$TMP/poll_stop.json" || :
  if [ "$code" = 200 ] && python3 - "$TMP/stop2.json" "$TMP/poll_stop.json" <<'PY'
import json, sys
r, p = (json.load(open(f)) for f in sys.argv[1:])
types = [c.get("type") for c in p.get("commands", [])]
sys.exit(0 if r.get("accepted") is True and "drive" not in types and "stop" in types else 1)
PY
  then bridge_stop_coalesce_purges=true; fi
  sleep 0.3
  # 2. Return, Stop, Return within 2 s: Stop resets the duplicate-navigation guard.
  c1=$(post /v1/return '{}' "$TMP/ret1.json") || c1=000
  post /v1/stop '{}' "$TMP/stop3.json" >/dev/null
  c2=$(post /v1/return '{}' "$TMP/ret2.json") || c2=000
  [ "$c1" = 200 ] && [ "$c2" = 200 ] && bridge_stop_resets_nav_guard=true
  # 3. The simulator page carries the token only for loopback requests.
  curl -fsS "$BASE/simulator" 2>/dev/null | grep -q 'SUMMON_BRIDGE_TOKEN="verify"' && bridge_sim_token_loopback=true
  LAN_IP=$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -v '^127\.' | grep -E '^[0-9.]+$' | head -1)
  if [ -n "$LAN_IP" ] && curl -fsS -o "$TMP/sim_lan.html" "http://$LAN_IP:$PORT/simulator" 2>/dev/null; then
    grep -q 'verify' "$TMP/sim_lan.html" || bridge_sim_token_lan_hidden=true
  else
    echo "note: no reachable non-loopback address; LAN token check skipped" >&2; bridge_sim_token_lan_hidden=true
  fi
fi
cleanup
BP=
# 3b. Keep-out zones and the new state fields (headless bridge, no simulator page registered).
bridge_zones=false
PORT=$((PORT-4)); BASE="http://127.0.0.1:${PORT}"
if start_bridge; then
  zc=$(curl -s -o "$TMP/zones.json" -w '%{http_code}' -H "$AUTH" -H 'Content-Type: application/json' -X POST -d '{"zones":[{"name":"Desk","x":7,"y":3.8,"w":2,"h":1.6}]}' "$BASE/v1/zones")
  bad=$(curl -s -o /dev/null -w '%{http_code}' -H "$AUTH" -H 'Content-Type: application/json' -X POST -d '{"zones":[{"name":"Off map","x":15,"y":9,"w":5,"h":5}]}' "$BASE/v1/zones")
  sleep 0.3
  curl -fsS -H "$AUTH" "$BASE/v1/state" >"$TMP/state_zones.json"
  if [ "$zc" = 200 ] && [ "$bad" = 422 ] && python3 -c "
import json,sys; s=json.load(open('$TMP/state_zones.json'))
z=s.get('zones') or []; bp=s.get('battery_plan') or {}
ok=len(z)==1 and z[0]['name']=='Desk' and abs(z[0]['x']-7)<1e-6 and isinstance(bp.get('need_home_pct'),(int,float)) and 'can_summon' in bp and isinstance(s.get('trips'),list) and 'alert_text' in s
sys.exit(0 if ok else 1)"; then bridge_zones=true; else echo "zones: post=$zc bad=$bad state=$(head -c 400 "$TMP/state_zones.json")" >&2; fi
fi
cleanup
BP=
PORT=$((PORT+4)); BASE="http://127.0.0.1:${PORT}"
# 4. In-process checks (loopback predicate, headless summon low-battery redirect and E-stop).
PORT=$((PORT-2)) SUMMON_TOKEN=verify node tests/bridge/headless_checks.js >"$TMP/headless.jsonl" 2>"$TMP/headless.err"
headless_ok=$?
cat "$TMP/headless.jsonl"

printf '{"check":"bridge_dup_poll","ok":%s}\n' "$bridge_dup_poll"
printf '{"check":"bridge_restart_stale_epoch","ok":%s}\n' "$bridge_restart_stale_epoch"
printf '{"check":"bridge_restart_current_epoch","ok":%s}\n' "$bridge_restart_current_epoch"

printf '{"check":"bridge_stop_coalesce_purges","ok":%s}\n' "$bridge_stop_coalesce_purges"
printf '{"check":"bridge_stop_resets_nav_guard","ok":%s}\n' "$bridge_stop_resets_nav_guard"
printf '{"check":"bridge_sim_token_loopback","ok":%s}\n' "$bridge_sim_token_loopback"
printf '{"check":"bridge_sim_token_lan_hidden","ok":%s}\n' "$bridge_sim_token_lan_hidden"
printf '{"check":"bridge_zones_and_battery_plan","ok":%s}\n' "$bridge_zones"

[ "$bridge_dup_poll" = true ] &&
[ "$bridge_restart_stale_epoch" = true ] &&
[ "$bridge_restart_current_epoch" = true ] &&
[ "$bridge_stop_coalesce_purges" = true ] &&
[ "$bridge_stop_resets_nav_guard" = true ] &&
[ "$bridge_sim_token_loopback" = true ] &&
[ "$bridge_sim_token_lan_hidden" = true ] &&
[ "$bridge_zones" = true ] &&
[ "$headless_ok" = 0 ]

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

printf '{"check":"bridge_dup_poll","ok":%s}\n' "$bridge_dup_poll"
printf '{"check":"bridge_restart_stale_epoch","ok":%s}\n' "$bridge_restart_stale_epoch"
printf '{"check":"bridge_restart_current_epoch","ok":%s}\n' "$bridge_restart_current_epoch"

[ "$bridge_dup_poll" = true ] &&
[ "$bridge_restart_stale_epoch" = true ] &&
[ "$bridge_restart_current_epoch" = true ]

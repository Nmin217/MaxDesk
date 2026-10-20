#!/usr/bin/env bash
# One command to check what can be checked here.   ./scripts/verify.sh          quick (about 1-2 minutes)
#                                                  ./scripts/verify.sh --full   adds slots, parking, round trips, batteries
cd "$(dirname "$0")/.." || exit 1
FULL=0; [ "${1:-}" = "--full" ] && FULL=1
pass=0; fail=0
ok(){ echo "  PASS  $1"; pass=$((pass+1)); }
no(){ echo "  FAIL  $1"; fail=$((fail+1)); }

echo "== Java core (planner, safety supervisor, map memory)"
if (cd robot/java-core && rm -rf out && java tools/Build.java >/tmp/jb.log 2>&1 && java -cp out frc.robot.summon.core.CoreTests >/tmp/jt.log 2>&1); then
  ok "builds with source/target 17, $(tail -1 /tmp/jt.log)"; else no "Java build or tests (see /tmp/jb.log /tmp/jt.log)"; fi
rm -rf robot/java-core/out

echo "== Simulator is the single source of truth"
python3 scripts/sync_sim_core.py >/dev/null && ok "bridge/sim_core.js regenerated from the simulator" || no "sync_sim_core"
node tools/check3d.js robot/simulator/summon_table_sim.html > /tmp/3d.log 2>&1 && ok "3D model: every part connected to the floor at 4 desk heights" || { no "3D model has floating parts"; tail -3 /tmp/3d.log; }

echo "== Bridge (serves the real simulator over the API)"
PORT=8799 SUMMON_TOKEN=verify node bridge/bridge_sim.js >/tmp/bridge.log 2>&1 & BP=$!
sleep 1.5
A="Authorization: Bearer verify"; B=http://127.0.0.1:8799
[ "$(curl -s -o /dev/null -w '%{http_code}' $B/v1/state)" = "401" ] && ok "rejects a request with no token (401)" || no "token check"
curl -s -H "$A" $B/v1/state | python3 -c "import sys,json;d=json.load(sys.stdin);assert d['pose'] and 'battery' in d" 2>/dev/null && ok "GET /v1/state" || no "GET /v1/state"
[ "$(curl -s -o /dev/null -w '%{http_code}' -H "$A" -X POST -d '{"x":99,"y":3}' $B/v1/summon)" = "422" ] && ok "off-map target refused (422)" || no "422 check"
curl -s -H "$A" -X POST -d '{"plugged":true}' $B/v1/sim/cord >/dev/null; sleep 0.3
[ "$(curl -s -o /dev/null -w '%{http_code}' -H "$A" -X POST -d '{"x":14,"y":8.6}' $B/v1/summon)" = "409" ] && ok "summon refused while the cord is plugged in (409)" || no "cord interlock via API"
curl -s -H "$A" -X POST -d '{"plugged":false}' $B/v1/sim/cord >/dev/null; sleep 0.3
curl -s -H "$A" -X POST -d '{"x":14,"y":8.6}' $B/v1/summon | grep -q '"accepted":true' && ok "summon accepted after unplugging" || no "summon after unplug"
sleep 3; curl -s -H "$A" $B/v1/state | grep -q '"active":true' && ok "the desk is driving" || no "desk did not start driving"
curl -s -H "$A" $B/v1/map | python3 -c "import sys,json;m=json.load(sys.stdin);assert len(m['rows'])==50 and len(m['rows'][0])==80" 2>/dev/null && ok "GET /v1/map (50 x 80)" || no "map"
kill $BP 2>/dev/null; wait $BP 2>/dev/null

echo "== iOS app source"
python3 scripts/check_swift.py && ok "Swift delimiters balanced (NOT compiled: build it in Xcode)" || no "Swift typo check"

echo "== Simulator scenarios (headless)"
if node tests/sim/run.js cord > /tmp/cord.log 2>&1 && grep -q "refused" /tmp/cord.log && grep -q "breakaways 1" /tmp/cord.log && ! grep -q "MOVED (bad)" /tmp/cord.log; then
  ok "cord lock: refuses, stops mid-trip, breakaway works, auto mode unaffected"; else no "cord scenario"; tail -5 /tmp/cord.log; fi
if node tests/sim/run.js stoppriority > /tmp/sp.log 2>&1 && grep -q "STOP-PRIORITY PASS" /tmp/sp.log; then
  ok "stop priority: cord, summon, stop, dup"
else
  no "stop priority"; tail -6 /tmp/sp.log
fi
if node tests/sim/run.js recovery > /tmp/rec.log 2>&1 && grep -q "RECOVERY PASS" /tmp/rec.log; then
  ok "recovery: turning is not stuck, wall stall recovers, never gives up, Stop/E-stop win"
else
  no "recovery"; grep '"ok":false' /tmp/rec.log | head -6
fi

node tests/pi/test_runtime.js > /tmp/pi.log 2>&1 && ok "pi runtime" || { no "pi runtime"; tail -8 /tmp/pi.log; }
if bash tests/bridge/run.sh > /tmp/br.log 2>&1; then
  ok "bridge: dup poll, restart epoch handling"
else
  no "bridge restart"; tail -8 /tmp/br.log
fi
if [ $FULL -eq 1 ]; then
  for s in slots parking roundtrip battery; do
    node tests/sim/run.js $s > /tmp/$s.log 2>&1 && ! grep -q '"collisions":[1-9]' /tmp/$s.log && ! grep -Eq 'collisions[^0-9]*[1-9]' /tmp/$s.log && ! grep -q 'did not' /tmp/$s.log && ok "$s ran with no collisions" || { no "$s"; tail -4 /tmp/$s.log; }
  done
  node tests/sim/run.js navdiag > /tmp/nav.log 2>&1 && grep -q "NAVDIAG PASS" /tmp/nav.log && ok "navigation: every trip arrives, no route flip-flops, no twitching, no recoveries in open room" || { no "navigation diagnostics"; grep -E "^NAVDIAG|problem" /tmp/nav.log | head -8; }
fi
echo; echo "$pass passed, $fail failed"; [ $fail -eq 0 ]

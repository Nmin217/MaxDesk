# Robot bridge API (v1)

The iPhone app talks to a small HTTP server on the robot's onboard computer (the "bridge"). The bridge translates to
the robot's internals (NetworkTables to the roboRIO, UPS data from Network UPS Tools). This file is the contract; the
simulator bridge in `bridge/` implements it and has been tested with curl.

- Transport: HTTP/1.1 and JSON, local network only. Header `Authorization: Bearer <token>` on everything except health
  and the static pages `/` (also `/app`), `/simulator` and `/manifest.webmanifest`, which are served without a token.
  The simulator bridge injects its token into `/simulator` only when the request comes from loopback (127.0.0.1 / ::1),
  so a page opened from another machine falls back to the default `dev-token`.
- Epoch: every JSON reply carries `epoch`, a counter bumped each time the simulator page (re)connects after being
  stale. Commands may send the last `epoch` they saw; a mismatch is refused with 409 and `stale_epoch: true`.
- Units: meters and radians. Room frame: origin top-left, x to the right, y DOWN, theta 0 = desk long axis along +x.
- Keys are snake_case.

| Method and path | Body | Result |
|---|---|---|
| `GET /v1/health` | none | `{ "ok": true, name, version, epoch }` (no token needed) |
| `GET /v1/state` | none | state object, see below |
| `GET /v1/map` | none | `{ width_m, height_m, cell_m, gw, gh, rows }` rows: `.` never seen, `_` free, `#` remembered obstacle, `+` busy |
| `GET /v1/lidar` | none | `{ source, simulator_connected, max_range, frame: "robot", units: "meters", points, updated_at, epoch }` points: `[{ angle (degrees, -180..180), range, type }]` |
| `POST /v1/summon` | `{ "x": 14.0, "y": 8.6 }` | `{ accepted, status, severity, command_id }` |
| `POST /v1/drive` | `{ "x": 0.5, "y": 0, "turn": 0, "speed": 0.4 }` | same. Manual joystick: x, y, turn clamped to -1..1, speed to 0.08..1; only the newest packet is kept. Sets mode `manual`. Optional `sid` (one random id per phone page) and `seq` (increasing): a packet with `seq` not above the last one from that `sid` gets 409 `stale_drive`, so a delayed packet can't overtake a newer one |
| `POST /v1/return` | `{}` | same |
| `POST /v1/learn` | `{}` | same (drives a mapping tour, then returns to the dock) |
| `POST /v1/stop` | `{}` | same. A SOFT stop. Always purges queued drive/summon/return/learn and clears the duplicate-navigation guard. A repeat within 250 ms while the first Stop is still queued is coalesced: `200 { accepted: true, status: "Stop already queued" }`, never 429. There is deliberately no way to release the E-stop. |
| `POST /v1/sim/cord` | `{ "plugged": true }` | simulator only: stands in for a person plugging the cord in |
| `POST /v1/zones` | `{ "zones": [{ "name": "Desk", "x": 7, "y": 3.8, "w": 2, "h": 1.6 }] }` | same. Replaces all keep-out zones (metres from the room's top-left; up to 20; `[]` clears). The planner treats them as solid and keeps the desk's footprint plus margin away. 422 if a zone is outside the room or smaller than 0.1 m |
| `GET /v1/sim/poll` | none | simulator page only: `{ commands, server_time, epoch }`, drains the queue (each command delivered once, current epoch only) |
| `POST /v1/sim/state` | state object (may include `lidar`) | simulator page only: `{ ok, epoch }`. Makes the page the authoritative simulation; a reconnect after 3 s stale bumps `epoch`, drops queued commands and queues a Stop |
| `POST /v1/sim/map` | map object | simulator page only: `{ ok, epoch }` |

Status codes: `200` accepted, `409` understood but refused: the cord interlock (cord plugged in and interlock on, for any
movement), a duplicate navigation command (same summon target, return or learn again within 2 s, unless a Stop came in
between), or a stale epoch (`stale_epoch: true, current_epoch`); `422` invalid target or non-numeric drive values,
`503` the simulator page was connected but has gone stale (open `/simulator` and keep it running), `401` bad token,
`404` unknown path, `400` bad JSON. The body of 409, 422 and 503 is still a normal result object, so the app can show
the reason.

## State object

    time, mode ("user" | "home" | "tour" | "manual"), status (text to show as-is), severity ("ok" | "warn" | "bad" | "info"),
    active (a trip is in progress), estop (read-only),
    pose {x, y, theta}, user {x, y},
    battery {robot_pct, ups_pct}, charging, cord {plugged, interlock_on, breakaways},
    dock_type ("manual" | "auto"), desk_height_cm, alert (the robot wants attention),
    explored_pct, remembered_cells, people [{x, y, settled}],
    simulator_connected (the simulator page is live and authoritative), epoch,
    zones [{id, name, x, y, w, h}] (metres),
    battery_plan {need_home_pct (to get back to the charger from here), need_trip_pct (to reach you and get back),
                  reserve_pct (never planned below), can_summon},
    alert_text (why the robot wants attention, e.g. "Battery low: heading back to the charger"; empty when none),
    trip (the trip in progress or null: {id, kind, distance_m, duration_s}),
    trips (last 10 finished: [{id, kind "summon"|"return"|"tour", outcome "arrived"|"stopped"|"battery"|"e-stop"|"cord"|"ended"|"replaced",
           started_at (ms), duration_s, distance_m, safety_holds, battery_used_pct}])

Battery rule: a summon is refused unless reaching you, getting back to the charger and the reserve all fit in the
battery; during a trip the robot turns back on its own as soon as getting home plus the reserve no longer fits.

## Rules the real bridge must keep

1. The app can never release the hardware E-stop, and `stop` is software only. The E-stop chain does not depend on this server.
2. If the cord is plugged in, `summon`, `drive`, `return` and `learn` are refused with 409, and the robot's cord lock holds the wheels off regardless of what the app sends.
3. Refuse targets outside the mapped room (422).
4. Every endpoint except health and the static pages needs the token. Local network only; do not expose the port to the internet.

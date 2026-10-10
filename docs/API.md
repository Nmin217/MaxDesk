# Robot bridge API (v1)

The iPhone app talks to a small HTTP server on the robot's onboard computer (the "bridge"). The bridge translates to
the robot's internals (NetworkTables to the roboRIO, UPS data from Network UPS Tools). This file is the contract; the
simulator bridge in `bridge/` implements it and has been tested with curl.

- Transport: HTTP/1.1 and JSON, local network only. Header `Authorization: Bearer <token>` on everything except health.
- Units: meters and radians. Room frame: origin top-left, x to the right, y DOWN, theta 0 = desk long axis along +x.
- Keys are snake_case.

| Method and path | Body | Result |
|---|---|---|
| `GET /v1/health` | none | `{ "ok": true, ... }` (no token needed) |
| `GET /v1/state` | none | state object, see below |
| `GET /v1/map` | none | `{ width_m, height_m, cell_m, gw, gh, rows }` rows: `.` never seen, `_` free, `#` remembered obstacle, `+` busy |
| `POST /v1/summon` | `{ "x": 14.0, "y": 8.6 }` | `{ accepted, status, severity }` |
| `POST /v1/return` | `{}` | same |
| `POST /v1/learn` | `{}` | same (drives a mapping tour, then returns to the dock) |
| `POST /v1/stop` | `{}` | same. A SOFT stop. There is deliberately no way to release the E-stop. |
| `POST /v1/sim/cord` | `{ "plugged": true }` | simulator only: stands in for a person plugging the cord in |

Status codes: `200` accepted, `409` understood but refused (for example the cord is plugged in), `422` invalid target,
`401` bad token, `404` unknown path, `400` bad JSON. The body of 409 and 422 is still a normal result object, so the app
can show the reason.

## State object

    time, mode ("user" | "home" | "tour"), status (text to show as-is), severity ("ok" | "warn" | "bad" | "info"),
    active (a trip is in progress), estop (read-only),
    pose {x, y, theta}, user {x, y},
    battery {robot_pct, ups_pct}, charging, cord {plugged, interlock_on, breakaways},
    dock_type ("manual" | "auto"), desk_height_cm, alert (the robot wants attention),
    explored_pct, remembered_cells, people [{x, y, settled}]

## Rules the real bridge must keep

1. The app can never release the hardware E-stop, and `stop` is software only. The E-stop chain does not depend on this server.
2. If the cord is plugged in, `summon`, `return` and `learn` are refused with 409, and the robot's cord lock holds the wheels off regardless of what the app sends.
3. Refuse targets outside the mapped room (422).
4. Every endpoint except health needs the token. Local network only; do not expose the port to the internet.

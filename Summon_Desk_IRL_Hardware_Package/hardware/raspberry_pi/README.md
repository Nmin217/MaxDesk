# Raspberry Pi Software
Handles LiDAR, obstacle detection, publishing safety data, and Pi heartbeat.

Does NOT drive motors directly.

Install: python3 -m pip install -r requirements.txt   (64-bit Raspberry Pi OS; pyntcore has no armv7 wheels)
Run:     python3 safety_monitor.py --config ../hardware-tests/config.json

Communication: NetworkTables 4 (pyntcore) to roboRIO, port 5810. Set `networktables.server`
(IP/hostname) or `networktables.team` (team number) in config.

Behaviour: a background thread reads the LiDAR; a fixed-rate loop (`safety.loop_hz`, default 20)
publishes an incrementing `Pi/Heartbeat` each iteration and sets `Safety/Stop` + `LiDAR/Status`:
- `STALE` (or `ERROR` after a driver error): no scan for `max_stale_scan_sec`
- `OBSTACLE`: forward-sector return closer than `stop_distance_m`
- `BLIND`: fewer than `min_sector_points` (default 5) valid returns in the forward sector
- `OK`: otherwise (Stop false)
rplidar reports millimetres; `Obstacle/Distance` is published in metres (-1 = unknown).

Tests (no hardware needed): from the repo root, `python3 -m unittest discover -s tests/pi_python -v`

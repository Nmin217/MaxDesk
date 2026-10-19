# Raspberry Pi Software
Handles LiDAR, obstacle detection, publishing safety data, and Pi heartbeat.

Does NOT drive motors directly.

Install: python3 -m pip install -r requirements.txt
Run:     python3 safety_monitor.py --config ../hardware-tests/config.json

Communication: NetworkTables to roboRIO. Set server under networktables.server in config.

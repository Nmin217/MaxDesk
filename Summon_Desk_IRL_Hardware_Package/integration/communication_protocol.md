# Communication Protocol

Transport: NetworkTables. Base table: SummonDesk

| Topic | Type | Publisher | Meaning |
|---|---|---|---|
| Pi/Heartbeat | double | Pi | Stale detection |
| LiDAR/Status | string | Pi | OK/STALE/OBSTACLE/BLIND/ERROR/SHUTDOWN (BLIND: too few valid returns ahead, e.g. sensor covered; Stop is true) |
| Obstacle/Forward | boolean | Pi | Obstacle in forward sector |
| Obstacle/Distance | double | Pi | Nearest forward distance (m), -1 unknown |
| Safety/Stop | boolean | Pi | roboRIO must stop |
| Movement/Forward | double | bridge/Pi | -1..1 |
| Movement/Strafe | double | bridge/Pi | -1..1 |
| Movement/Rotation | double | bridge/Pi | -1..1 |

Rules:
- roboRIO stops if Safety/Stop true, heartbeat stale, or NT disconnects.
- Pi never drives motors directly.
- Movement requests are always validated/limited before use.

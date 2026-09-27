# Swerve Desk — Build Plan

A summonable desk: a low-profile swerve chassis (REV MAXSwerve, already owned) that the desk bolts onto, with a 2D LIDAR for mapping/navigation and a phone/tag interface to summon it.

---

## 1. System Architecture

```
 Phone / web UI / Home Assistant / UWB tag
                 │  (Wi-Fi: "go to couch" / "come to me")
                 ▼
 ┌─────────────────────── Coprocessor (Jetson Orin Nano or Pi 5) ───────────────────────┐
 │ ROS 2 Jazzy: slam_toolbox (mapping) → Nav2 (localization + holonomic planning)        │
 │ robot_localization EKF (wheel odom + IMU) · collision_monitor (LIDAR safety zones)    │
 │ swerve_driver node: cmd_vel → 4× module states (inverse kinematics) → CAN             │
 └──────────────┬──────────────────────────────────────┬────────────────────────────────┘
          USB   │                               USB-CAN│ (CANable)
          ▼                                            ▼
   2D LIDAR (RPLIDAR C1/S2 or LD19)       4× MAXSwerve: SPARK MAX/Flex (drive) + SPARK MAX (turn)
                                          + through-bore absolute encoders, IMU (Pigeon 2 on CAN)
```

### Controller decision

| | **A. roboRIO 2 + coprocessor** | **B. Coprocessor drives CAN directly** |
|---|---|---|
| Motor control | REVLib + REV MAXSwerve template, works out of the box | Community SPARK CAN libraries or own frames/heartbeat |
| Enabling | Needs FRC Driver Station to enable — awkward for an appliance | Own enable/safety logic |
| Complexity | Two computers, NetworkTables bridge | One computer, one ROS 2 stack |

**Plan:** Start with **A** for bring-up (known-good MAXSwerve code), then migrate to **B** once a SPARK-over-SocketCAN library is verified with our firmware version.

---

## 2. Bill of Materials

| Item | Qty | Notes |
|---|---|---|
| REV MAXSwerve module | 4 | **Already owned.** Use 12T (low) pinion — software caps speed anyway. |
| **Custom drop adapter** | 4 | Lowers chassis relative to modules. 3D-printed (PETG-CF / nylon-CF) for test fit → final in laser-cut aluminum or stacked 2×1 tube. See §3. |
| 2×1 tube stubs (~6") | 4 | Sit in module tube slots (stacked-tube adapter version) |
| 2×1 aluminum tube (frame) | ~12 ft | Frame sized to desk footprint |
| Drive motors: NEO or NEO Vortex | 4 | Vortex requires SPARK Flex |
| Turn motors: NEO 550 | 4 | |
| SPARK MAX / Flex | 8 | |
| 2D LIDAR (RPLIDAR C1/A2/S2 or LDROBOT LD19) | 1–2 | 360°, 10–12 m range is plenty indoors |
| IMU (CTRE Pigeon 2 or BNO085) | 1 | Heading is critical for swerve odometry |
| Coprocessor (Jetson Orin Nano or Pi 5 8GB) | 1 | |
| USB-CAN adapter (CANable 2.0) | 1 | Option B only |
| roboRIO 2 | 1 | Option A only |
| Battery: 12 V LiFePO4, 30–50 Ah + BMS + charger | 1 | Better runtime/weight than FRC SLA |
| PDH or fuse block, 120 A main breaker | 1 | |
| **Physical E-stop (mushroom) + contactor** | 1 | Non-negotiable |
| Wireless E-stop / key-fob relay | 1 | |
| Bumper switches or ToF sensors | 4+ | Cover LIDAR blind spots |
| Belly pan (aluminum or polycarbonate sheet) | 1 | Low-mounted electronics + battery |
| #10-32 bolts, nylock nuts, large washers | ~40 | Adapter through-bolts |
| UWB anchors + tag (DWM3001C / ESP32-UWB) | optional | "Come to me" |

---

## 3. Mechanical Plan

### 3.1 Chassis + custom drop adapter
MAXSwerve modules clamp a 2×1 tube in a slot on the module (#10-32 through-bolts), and that slot sets the frame height. REV doesn't sell a lowering kit (SDS does, for MK4c/4i/4n/MK5 — 7/8" drop, ~1/2" clearance — but it doesn't fit MAXSwerve). So each corner gets a **custom drop adapter**: a tongue that fills the module's tube slot, with a mounting face 1–2" lower for the main frame rail.

- **Prototype:** 3D-printed PETG-CF or nylon-CF.
  - 100% infill (or 6+ walls) around bolt holes.
  - Orient so the desk load **compresses** layers, never peels them.
  - Through-bolts with large washers carry the load, not layer adhesion.
  - **No PLA** — it creeps under constant load.
- **Final:** either
  - laser-cut aluminum version of the adapter (SendCutSend), or
  - the proven stacked-tube method: 2×1 stubs in module slots, main rails bolted directly beneath, with **through-bolts between the layers** (gussets alone aren't enough).
- **Target:** ≥ 1/2" floor clearance under the lowest point of frame/belly pan.
- **Measure first (on a real module):**
  1. Floor → bottom of the tube slot (stock clearance)
  2. Wheel/belt clearance below the slot through full steering rotation
  3. Bolt-hole positions in the slot

### 3.2 Desk mounting
- Desk legs bolt to the lowered frame rails **beside** the modules, not on top — NEO tops (6.75" off floor) stay clear.
- Shorten the legs, or lower an adjustable desk, by the frame height → near-zero net height gain.

### 3.3 Center of gravity
- Belly pan hangs from the lowered rails; battery, SPARKs, coprocessor, and power distribution mount on it.
- Modules at or slightly outside the desk footprint to widen the wheelbase (more tip resistance).
- Hand tip-test the loaded desk before first power-on; this sets accel limits.

### 3.4 LIDAR placement
- Mount under the desktop at ~20–35 cm height with maximum 360° visibility.
- Mask desk legs with `laser_filters` (angular bounds), or use two LIDARs on opposite corners and merge scans.

### 3.5 Floor + power
- Test tread on the actual floor (marking on hardwood, wear on carpet).
- Desk accessories: either run monitors off onboard battery via inverter, or "unplug before summon" with a power-sense input the software checks.

---

## 4. Software Plan (ROS 2 Jazzy)

```
swerve-desk/
├── hardware/
│   ├── drop-adapter/    # parametric CAD (CadQuery/OpenSCAD) → STL + DXF
│   ├── cad/             # chassis CAD
│   └── wiring/          # wiring diagram
├── firmware/roborio/    # (Option A) WPILib Java from REV MAXSwerve template + NT bridge
├── ros2_ws/src/
│   ├── desk_bringup/    # launch files, URDF, params
│   ├── swerve_driver/   # cmd_vel → module states, odometry, CAN/NT interface
│   ├── desk_nav/        # Nav2 + slam_toolbox configs, collision_monitor, keepout zones
│   ├── desk_summon/     # named-waypoint server, REST/WebSocket API, UWB "come to me"
│   └── desk_safety/     # watchdog, E-stop state, bumpers, speed governor
└── app/                 # phone web UI / Home Assistant integration
```

- **Kinematics:** port WPILib `SwerveDriveKinematics` (or use it on the roboRIO); module state optimization (never rotate >90°) + cosine compensation.
- **Odometry:** wheel odometry + IMU fused in `robot_localization`; AMCL / slam_toolbox localization corrects drift.
- **Mapping:** drive manually with a gamepad in slam_toolbox mapping mode, save the map.
- **Navigation:** Nav2 with **MPPI controller, `Omni` motion model** to use holonomic strafing. Limits:
  - Max speed **0.3–0.5 m/s**
  - Max accel **~0.3 m/s²**
  - Max yaw rate **~0.5 rad/s**
- **Docking:** Nav2 `opennav_docking` against a LIDAR-visible L-bracket on the wall, or an AprilTag + small camera.
- **Summon:**
  1. **Named locations** ("office", "couch") via web page / Home Assistant / voice.
  2. **Come to me** via UWB tag + 3–4 wall anchors → Nav2 goal with a standoff distance.

---

## 5. Safety Requirements

- **Hardware E-stop** cutting motor power via contactor, independent of software.
- **Watchdog:** no cmd_vel / heartbeat for 200 ms → stop (brake mode).
- **Nav2 `collision_monitor`** slowdown + stop polygons around the footprint.
- **Low bumpers / ToF sensors** — a 2D LIDAR misses chair legs outside its scan plane, cables, pets lying flat, drop-offs. Cliff sensors if there are stairs.
- **Current limits:** ~40 A drive, ~20 A turn.
- **Hard speed cap** in the driver layer, independent of Nav2 params.
- **Refuse to move** on IMU tilt threshold, low battery, or "desk occupied / items unsecured" flag.

---

## 6. Roadmap

| Phase | Goal | Done when |
|---|---|---|
| **0. Design** | Measure modules, design + print-test the drop adapter, CAD chassis around desk, wiring diagram | Adapter test-fits on all 4 corners with ≥ 1/2" clearance; tip margin calculated |
| **1. Drive bring-up** | Build lowered frame, wire modules, run REV MAXSwerve template, gamepad teleop | Smooth field-relative driving at low speed; E-stop verified |
| **2. Odometry** | Tune module PID, zero absolute encoders, IMU heading | 3 m square returns within ~5 cm |
| **3. ROS 2 bridge** | `swerve_driver` publishes `/odom`, consumes `/cmd_vel`; LIDAR `/scan` with legs masked | Teleop through ROS; TF tree correct |
| **4. Mapping + localization** | slam_toolbox map, AMCL | Desk knows where it is after reboot |
| **5. Autonomous nav** | Nav2 + MPPI Omni, collision monitor, keepout zones | Reliable point-to-point around furniture |
| **6. Summon** | Named waypoints + web/phone UI, docking | "Go to couch" works 20/20 |
| **7. Come to me** | UWB anchors + tag | Arrives within ~0.5 m of you |
| **8. Polish** | Onboard monitor power, quiet operation, charging dock | Daily-use ready |

---

## 7. Top Risks

1. **Tipping with monitors on top** → lowered chassis via drop adapter, belly pan, low accel, IMU tilt cutoff.
2. **Printed adapter creep/failure under constant load** → CF-filled filament, bolt-carried load, treat print as prototype; final in aluminum or stacked tube.
3. **LIDAR occlusion by desk legs** → mount height, scan masking, or second LIDAR.
4. **Obstacles outside the scan plane** → bumpers/ToF, optional depth camera (OAK-D Lite / RealSense) into the costmap.
5. **REV-without-roboRIO path (Option B)** → start on Option A.
6. **Cables/power while moving** → decide battery vs. unplug-before-summon early.

---

## References

- [MAXSwerve — REV docs](https://docs.revrobotics.com/ion-build/motion/maxswerve)
- [MAXSwerve SPARK MAX Mounting Bracket Assembly — REV docs](https://docs.revrobotics.com/ion-build/build-guides/maxswerve-module-assembly/maxswerve-spark-max-mounting-bracket-assembly)
- [3in MAXSwerve Module — REV Robotics](https://www.revrobotics.com/rev-21-3005/)
- [Mounting MaxSwerve above the frame — Chief Delphi](https://www.chiefdelphi.com/t/mounting-maxswerve-above-the-frame/502978)
- [Chassis Lowering Kit (MK4c/4i/4n) — SDS](https://www.swervedrivespecialties.com/products/kit-chassis-lowering-mk4c-4i-4n) (reference design; not MAXSwerve-compatible)

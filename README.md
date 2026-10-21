# Summon Desk (swerve-desk)

A Humanscale sit-stand desk on a swerve base that drives to you, parks facing you, returns to its charging station, and
is controlled from an iPhone. This repository combines the original swerve-desk build plan ([`PLAN.md`](PLAN.md):
mechanical design, BOM, roadmap) with the MaxDesk code (everything below).

**Bringing up hardware?** Start with [`firmware/module-test`](firmware/module-test/README.md): wiring,
REV Hardware Client setup, and a bench-test program for your MAXSwerve modules.

    firmware/module-test/   WPILib 2026 bench test for 1-4 MAXSwerve modules (readout, self test, joystick)
    Summon_Desk_IRL_Hardware_Package/   roboRIO + Raspberry Pi starter code, LiDAR test, bring-up checklists

    robot/simulator/   summon_table_sim.html   the browser simulator = the SINGLE SOURCE OF TRUTH for behaviour
    robot/java-core/   Java (WPILib-style) planner, safety supervisor, map memory, with tests; WPILib glue skeleton
    bridge/            Node server that serves the REAL simulator over the phone API (so the app needs no hardware)
    ios/               SwiftUI iPhone app + the Info.plist keys it needs
    mobile/            responsive phone web controller (Android/iPhone browser)
    docs/              API.md (the phone <-> robot contract), the project proposal, the bill of materials
    tests/sim/         headless scenarios that run the simulator's own code
    tools/ scripts/    3D-model check, and verify.sh: one command that checks what can be checked

## How the pieces fit

    iPhone app  --HTTP/JSON (docs/API.md)-->  bridge  -->  today: the simulator   |   on the robot: NetworkTables + UPS
                                                           roboRIO runs the Java robot code; a Pi/Jetson runs lidar drivers

The simulator defines how the desk should behave. The Java core reproduces its planner and safety logic for the real robot
(the cross-check prints identical routes on 4 of 5 scenes, the fifth is an equal-cost tie). The bridge exposes whichever
backend is running behind the same API, and the app only ever talks to that API.

## Check everything

    ./scripts/verify.sh            # about 1-2 minutes: Java tests, API through the bridge, 3D model, Swift typo check, cord lock
    ./scripts/verify.sh --full     # also slots, parking, round trips and batteries (several minutes)

Needs: a JDK (17 or newer), Node 18+, Python 3, curl. No other dependencies.

## What is verified, and what is not

| Part | Status |
|---|---|
| Simulator behaviour | Tested headlessly: stress runs with 0 collisions, slots, parking, round trips, batteries, the cord lock. |
| Java core (planner, supervisor, memory) | Builds and passes its own 18 checks; matches the simulator's routes (see above). |
| Bridge and API | Tested with curl by `verify.sh`. |
| 3D model | Geometry checked (every part connected at four desk heights, module dimensions match REV's figures). Not seen in a browser by the author. |
| **Swift app** | **Never compiled.** Only a delimiter check. Build it in Xcode and expect small fixes. |
| **WPILib glue** (`robot/java-core/wpilib-glue`) | **Never compiled** (needs WPILib). A skeleton. |
| **Not written yet in Java** | The path follower, stuck recovery, the state machine (summon / return / learn / battery rules / cord lock). Today these exist only in the simulator. |
| Hardware | Nothing here has run on a real robot. |

## Quick start for each part

- **Simulator:** open `robot/simulator/summon_table_sim.html` in a browser.
- **Java core:** `cd robot/java-core && java tools/Build.java && java -cp out frc.robot.summon.core.CoreTests`.
  To use it in a WPILib project, copy `src/main/java/frc/robot/summon/core/` and read `robot/java-core/README.md`.
- **Phone controller (Android or iPhone):** run `Start Summon Desk.vbs` on Windows, or run `node bridge/bridge_sim.js` from the project root. On the phone, connected to the same trusted Wi-Fi, open `http://COMPUTER-IP:8787` in Chrome/Safari. On Android, use the browser menu → **Add to Home screen** for an app-like launcher. `dev-token` is a local development token only.
- **Desktop simulator:** open `http://localhost:8787/simulator` after the bridge starts. The `/` route is the mobile controller. The visual simulator at `/simulator` publishes its live state (including both battery levels) to the bridge and receives commands from the phone controller. Keep the `/simulator` tab open while controlling it from the phone; the phone reports when the simulator is offline.
- **No-terminal Windows start:** double-click `Start Summon Desk.vbs`; it starts the Node bridge hidden and opens the simulator in your browser. There is no stop script yet: end the `node` process in Task Manager to stop the bridge. If your organization's policies block VBScript, use the terminal command above instead.
- **Change the simulator:** edit `robot/simulator/summon_table_sim.html`, then run `python3 scripts/sync_sim_core.py` so the
  bridge picks it up (`verify.sh` does this for you).

## Left out on purpose

The earlier Python package (`summon_desk.zip`) is not included: it is out of date against the simulator and the Java core.

## GitHub Codespaces

For Codespaces setup and troubleshooting, follow [`CODESPACES-QUICKSTART.md`](CODESPACES-QUICKSTART.md). Run the bridge from the repository root and open the simulator through the forwarded URL at `/simulator`; do not open the HTML file directly if you want phone synchronization.

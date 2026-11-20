# MaxDesk: handoff (state as of 2026-11-16)

Read this first in a new session. Repo: **github.com/Nmin217/MaxDesk** (public). Push to `main`.

## Working rules the owner set
- Work on **MaxDesk only**. Ignore the old BioNanomics/swerve-desk repo (a stop hook may complain about
  unpushed commits there; ignore it).
- Don't overload the app with features: only things that make life easier.
- No pull requests unless asked. Don't rewrite or re-author the owner's commits.
- Explain changes in plain language: the owner is learning as they go.
- `scripts/verify.sh` (quick, about 2 min) and `scripts/verify.sh --full` (adds about 15 min) must pass before
  pushing. GitHub CI (`.github/workflows/ci.yml`) also builds both roboRIO bench programs on every push.

## What the project is
A desk on MAXSwerve modules that drives itself to you (summon), returns to its dock, maps the room with LiDAR,
avoids people and obstacles, and is controlled from a phone app.
- **Brain:** `robot/simulator/summon_table_sim.html` is the single source of truth.
  `python3 scripts/sync_sim_core.py` regenerates `bridge/sim_core.js` from it, and must be run after every edit.
- **Phone app:** `mobile/index.html`, served by the **bridge** (`bridge/bridge_sim.js`, port 8787).
- **Pi runtime:** `robot/pi/` runs the same brain on a Raspberry Pi with real hardware:
  - `runtime.js`, `pi_io.js`, `motion_guard.js`, `hardware_check.js`, `black_box.js`, `swerve.js`;
  - `fake_spark_bus.js` for tests.
- **roboRIO bench test:** `firmware/module-test` (WPILib/REVLib 2026; the one in use) and
  `firmware/module-test-2024` (for SPARK firmware 24).

## Hardware: current state
- **Test cart:** a triangle frame on 2x4 blocks, 3 MAXSwerve modules, roboRIO 2, PDH, NEO drive and NEO 550
  steering on SPARK MAXes. Team number **2197**. A 4th module is waiting.
- **Front = the diagonal rail** (photo of 2026-11-17: the rail between the left and bottom-right modules; forward points
  out of it). CAN IDs (drive/turn), named by where they sit in that photo:
  - **FL** = bottom-right module, 5/6;
  - **FR** = left module, 1/2;
  - **Back** = top-right module, 3/4;
  - a 4th module (7/8) will be added later.
- **Module positions (estimated)** from distances to the centre (bottom-right 8 in, left 11 in, top-right 10 in) and
  the photo's directions: FL (0.077, 0.188) m, FR (0.143, -0.240) m, Back (-0.252, -0.027) m. They're in
  `firmware/*/Constants.java`. Measure properly once the 4th module is on.
- **Four-module square frame (2026-11-19).** The owner zeroed all four with the 3D-printed MAXSwerve jig, so the
  wheels sit 90° apart per corner at zero (as they should). `Constants.ModuleId` now has `zeroPointsDeg` (which way
  the wheel points in robot terms at encoder 0; REV pattern FL 90, FR 0, BL 180, BR -90) and a per-module
  `encoderV2` (one module has a V2 after a V1 broke). Readout prints robot and raw angles to the DS console every
  second. Confirmed: front = top of the photo, FL 1/2, FR 3/4, BL 5/6, BR 7/8 (odd drive,
  even turn), BR has the V2. Module spacing: ~1 ft from the centre to each module (owner, 2026-11-20) = 0.216 m each way (`kHalf`).
- **Pi drives the SPARKs directly (2026-11-20).** No roboRIO or Driver Station: `robot/pi/spark_can.py` (Python
  stdlib SocketCAN helper, firmware 24 frames from sparkcan + status 5 for the absolute encoder; steering/drive loops
  at 100 Hz on the Pi; heartbeat only while Node says enabled; runaway/stuck steering guards), `spark_bus.js`,
  `xbox.js`, `tools/pi_bench.js` (Readout / Self test / Joystick / Cart, RB deadman, same zero directions as
  module-test). Tests: `tests/pi_python/test_spark_can.py`, `tests/pi/test_pi_bench.js`, `tests/pi/test_spark_bus.js`.
  The owner will downgrade the SPARKs from 25 to 24. Untested on hardware: status 5 byte layout, gains, CANivore.
- **The real desk** (not converted to swerve yet): the top is **48 x 30 in** (1.22 x 0.76 m), the legs are 45 in apart, and
  the T-feet are **about 43 in apart, centre to centre** (hole to hole, give or take 1–2 in). That's the left-right
  wheel spacing if the modules mount on the feet. Still needed: foot length (front to back), where on the feet the modules
  mount, weight, height range, LiDAR mount spot.
  - **Don't just change the brain's desk size.** Tried on 2026-11-17: with `HW=30.5, HH=19, TR=36` (48 x 30 in), the
    simulator's bump recovery got worse. In 10 seeded shoves, 2 weren't recovered within 8 s; with the old size, 10 of
    10 recovered within 1.5 s.
  - Change the size together with the real LiDAR mount positions and a localisation check and retune. Places that
    must change together: `HW/HH/TR` in the simulator, `DESK_HALF_*` in `robot/pi/runtime.js`, and `HALF_*` in
    `tests/pi/fake_world.js`.
- **SPARK firmware:** updated to the latest (25+). IDs were reset by the update and have been set again and burned
  to flash. Lesson learned: set IDs with only ONE SPARK on the CAN bus, or duplicates all change together.
- **roboRIO:** imaged 2026, team 2197. Our program is deployed. Bench test status:
  - deploy works, and the program runs (it replaced an old team program that used CTRE devices);
  - the drive motors spin, and Start/Back on the Xbox controller pick the mode;
  - **the wheels don't all point the same way, so the steering needs RE-ZEROING with the NEW forward** (REV Hardware
    Client, turn SPARKs 6, 2, 4: Absolute Encoder Inverted, wheel pointing front, bevel gears on the same side, Set Zero
    Offset, Burn Flash);
  - SPARK 4 once timed out on CAN, and later looked fine;
  - SPARK 5 (FR drive) once misbehaved during the firmware update.
- **Earlier module damage:** the REV motor tester bent a key and burned an encoder bridge (P13). The owner has the
  P13 STEP file. Advice given: get the V1.1 kit (P26 bridge, new key and spacer). P13 is printable on an X1C/H2D
  (pegs down, supports; PETG to test, ABS/ASA or PA-CF for use).
- **Raspberry Pi 4 (2 GB):**
  - flashed with Raspberry Pi OS Lite 64-bit, and it boots (green light flickers);
  - **SSH login failed so far**: the username was typed as `MaxDesk`, but Linux usernames must be lowercase. Try
    `ssh maxdesk@raspberrypi.local`, or re-flash with hostname `maxdesk`, username `maxdesk`, SSH enabled
    (password), and the Wi-Fi country set;
  - the owner posted the password in chat, so tell them to change it with `passwd`.
- **CAN for the Pi:** the owner may have a CTRE **CANivore**. It probably works as a Linux SocketCAN interface
  (CTRE's `canivore-usb` package) but is untested with SPARKs (CAN FD adapter vs classic-CAN SPARKs).
  `sparkcan` needs SPARK firmware **24**: the SPARKs are now 25+. Decide later between downgrading and writing
  our own driver.
- **No gyro yet.** It's optional. Tests show a gyro (for example a BNO085) cuts position error 2–3x. Decide after
  real driving.

## How to drive the bench test (roboRIO)
1. Download the ZIP from GitHub and open **`firmware/module-test`** in WPILib 2026 VS Code (not the whole repo).
   Set Team Number 2197, then **Deploy** (not just Build). Use a USB-B cable to the roboRIO; no radio needed.
2. Driver Station: team 2197, **TeleOperated** (not Test). Xbox controller in slot 0.
3. While **disabled**, press **Start** for the next mode or **Back** for the previous one (it rumbles, and the DS
   message window shows "Mode n/6: …"). Modes:
   1. Readout;
   2. Self test;
   3. Joystick (hold RB);
   4. Phone app;
   5. Cart: joystick;
   6. Cart: phone app.
4. Enable, then **hold RB** to move. Space = emergency stop.
5. **Phone/browser drive page:** `tests/hardware/app-drive-test`. Needs **Python 3.13** (no 2026 pyntcore for
   3.15): `py -3.13 -m pip install -r requirements.txt`, then `py -3.13 app_link.py`, then open the printed URL
   with `?k=`.

## Software built (all pushed, CI green)
- **Simulator:**
  - drift and LiDAR scan-matching localisation, lost-detection and global search, bump recovery;
  - 2D/3D LiDAR, battery planning and auto-return to charge, trip log;
  - boxed-in alert (15 s without progress);
  - office floor room (`?room=office`) with frontier exploration for "Learn the room".
- **Pi stack:**
  - the brain's hardware-input mode: odometry, real LiDAR rays, real battery, stale-scan hold and fault;
  - `pi_io.js` on a 5-call "SPARK bus" interface (configure/setVelocity/setPosition/heartbeat/status);
  - jam and wrong-direction detection, hardware check (missing module, encoder, volts, faults);
  - black box recorder (last 60 s saved on faults; viewer at `tools/blackbox_viewer.html`);
  - an end-to-end fake-hardware test (`tests/pi/test_hw_loop.js`, fake clock, repeatable).
- **App:**
  - "Real desk / Simulator" pill, and a problem banner that stays until fixed;
  - pairing (random key, 6-digit code, `bridge/pairing.js`; `dev-token` no longer works);
  - "Save last minute" (black box).
- **`tools/pi_dry_run.js`:** the whole stack in real time on fake hardware, to check the Pi is fast enough. It
  found and fixed 80 ms freezes: the lost-position search is now sliced to 4 ms per step on the real desk.

## Next steps
1. **Hardware (roboRIO):**
   - re-zero the steering;
   - Readout (bottom-right = FL, left = FR, top-right = Back, about 0° pointing out of the diagonal rail);
   - Self test (PASS all three);
   - Joystick, then Cart joystick on blocks, then the floor.
2. **Pi:** get SSH working, then install git, can-utils and Node 22, then
   `git clone https://github.com/Nmin217/MaxDesk.git`, then `node tools/pi_dry_run.js --seconds 90`. Judge by the
   reports after the first 10 s.
3. **CANivore test on the Pi:** `canivore-usb`, then `ip link`, then `candump` with the SPARKs powered. It only
   listens.
4. **Software queue:**
   - **push mode** (coast while E-stopped, to roll the desk by hand);
   - **motor heat limit**;
   - a Pi version of the bench test;
   - black box files in the app;
   - a desk health card;
   - Pi auto-start and install script;
   - Stop latency;
   - settings file;
   - battery voltage calibration (needs the battery type);
   - localisation retune for first-trip accuracy, as its own careful job (quick tweaks broke the bump tests).

# MaxDesk: summary for the next session (2026-11-25)

Hand this to a new chat. The full history is in `docs/HANDOFF.md`; this page is the short version plus the jobs
that need someone at the owner's computer.

**Repo:** https://github.com/Nmin217/MaxDesk (branch `main`, public). All work below is pushed and CI is green.
**Owner preferences:** explain plainly and in depth; one step at a time; don't clutter the app; no pull requests
unless asked. The owner can do software any time and hardware now and then.

## Where things stand

**Bench frame (4 MAXSwerve modules, roboRIO + Driver Station, SPARK firmware 25):** all four modules PASS the
self test. Front = top of the owner's photo. CAN IDs (drive/turn): FL 1/2, FR 3/4, BL 5/6, BR 7/8. Zeroed with
the 3D-printed jig, so each wheel's zero direction is set in code (`zeroPointsDeg`: FL 90, FR 0, BL 180, BR -90).
BR has the Through Bore **V2** encoder, the others V1. Spacing ~1 ft centre to each module (0.216 m each way).
Known niggle: SPARK 2 (FL steering) sometimes times out on CAN at startup; re-seat its CAN wires.
Program: `firmware/module-test` (firmware 25) or `firmware/module-test-2024` (firmware 24). Modes picked with the
Xbox Start/Back while disabled; hold RB to run. Spacebar in the Driver Station = E-stop (needs a roboRIO reboot).

**The real desk build (designed, parts not bought yet):**
- Desk top 48 x 30 in, legs 43 in apart (T-feet removed). Frame from AndyMark 2 x 1 in pre-drilled tube (47 in
  sticks, 7 to buy): back rail 44, side rails 26 x2, leg rails 29 x2 (legs bolt through them), cross brace 46.
  **No front rail**: the front is open so a chair rolls in. Each side rail is tied to its leg rail into one beam.
- Belly pan at the back only (1/8 in aluminium recommended). SPARK pairs on small 3D-printed shelves beside each
  module; 12 AWG power extensions with Powerpoles; one CAN chain ending at the PDH (terminator on).
- Four RPLIDAR C1: two low on the frame (front-left, back-right, ~10 in up, above the motors) for mapping; two hung
  under the desk top (front-right, back-left) for table/counter height. Powered USB hub with its own 5 V supply.
- Bumper strips with normally-closed microswitches on the desk-top edges and the side/back rails.
- Control: new **Raspberry Pi 5** + a **CAN FD HAT with an MCP2518FD chip** (not MCP2515) or a CANable 2.0.
  CANivore, Xbox/Logitech controller, Pi 4 and roboRIO are temporary test gear.
- TPU 3D-printed caps to round off the module corners.
- BOM: `docs/Summon_Desk_BOM.xlsx`, sheet "Have vs need" (about $1,277 still to buy, estimates).
- Drawings (claude.ai artifacts, owner's account): 3D model https://claude.ai/artifact/Bqm4uKXW8i6BVpCntfKC5M ,
  wiring plan https://claude.ai/artifact/RoZQ86JR58cGh7ELdGLohV

**Software built recently (all tested, see `robot/pi/README.md`):**
- Pi drives the SPARKs without a roboRIO or Driver Station: `robot/pi/spark_can.py` (CAN helper, SPARK
  **firmware 24** format), `spark_bus.js`, `xbox.js`, and the Pi bench program `tools/pi_bench.js`
  (`--sim` for pretend SPARKs). Safety: SPARKs only get the enable heartbeat while RB is held and the program is
  alive; runaway/stuck steering guards.
- Bumpers: `io.readBumpers()` -> stop on a new bump, only drive away while pressed. GPIO reader still to write.
- Bay watch (`robot/pi/bay_watch.js`): someone sitting at the desk = no motion; an empty chair = the desk slides
  straight out from over it first. Thresholds need checking with real LiDARs.

## Open decision: keep SPARK firmware 25 (no downgrade)?

The CAN helper speaks firmware 24. Community work (github.com/orribbie/sparkcan_pybindings, "sparklib-py" on
Chief Delphi) has reverse-engineered much of firmware 25 (status frames moved to API class 46; absolute encoder in
status 5 as velocity + position floats; heartbeat and duty-cycle commands look unchanged). Not official, not fully
proven. Plan: add a `--firmware 25` mode to `spark_can.py`, then confirm it against a real capture (below). If 25
has a blocker, downgrading all eight SPARKs to 24 is the proven fallback.

## Jobs that need someone at the owner's computer

1. **CAN capture on firmware 25** (10 minutes, frame on blocks). The roboRIO keeps driving; the Pi only listens.
   - Pi 4 + CANivore on the same CAN wires (CTRE's `canivore-usb` package makes it a Linux CAN interface).
   - `sudo ip link set can0 up type can bitrate 1000000` (skip if the CANivore interface is already up), then
     `candump -L can0 > capture.log` while running Readout and then Self test from the Driver Station.
   - Ctrl+C, and get `capture.log` to the chat (or commit it under `docs/captures/`).
2. **Pi login:** the username is lowercase `maxdesk` (capitals broke SSH before). Change the password with
   `passwd`: an earlier one was posted in a chat. Never paste passwords into a chat.
3. **On the Pi:** install Node 22, git and can-utils; `git clone https://github.com/Nmin217/MaxDesk`; try
   `node tools/pi_bench.js --sim` with the controller plugged into the Pi.
4. **Ordering** from the BOM: tube, Pi 5 kit, CAN FD HAT, 4 LiDARs + hub, fasteners, belly-pan sheet, wire.

## Next software jobs (no hardware needed)

- `--firmware 25` mode for `spark_can.py` (then verify with the capture).
- LiDAR reader for the RPLIDAR C1 (four units, each with its mount position), and the bumper GPIO reader.
- Pi auto-start on boot; Pi 5 + CAN HAT setup notes.
- When the desk frame is built: real wheel positions and desk size into the brain, retuned together with the
  localisation (changing the size alone made bump recovery worse in the simulator).

# MAXSwerve Module Bench Test

A WPILib 2026 / REVLib 2026 robot program that checks MAXSwerve modules before they go on a
frame. It's set up for three modules (FL, FR, RL); the modules are listed in `Constants.java`, so
it works with any number from one to four. Based on REV's
[MAXSwerve Java template](https://github.com/REVrobotics/MAXSwerve-Java-Template) (2026 branch),
with the same configuration and CAN IDs, so the IDs carry over to the full drivetrain.

> **Status:** I couldn't compile this in the environment I wrote it in (the FRC Maven servers
> were blocked there). Every WPILib call was checked against the WPILib 2026 source and every
> REVLib call against REV's 2026 template and docs. Run `./gradlew build` once before deploying,
> and expect at most a small fix.

Assumed hardware for each module: NEO drive motor with SPARK MAX, NEO 550 steering motor with
SPARK MAX, and the REV Through Bore Encoder on the steering SPARK MAX. If you have a NEO Vortex
or SPARK Flex, change `SparkMax`/`SparkMaxConfig` to `SparkFlex`/`SparkFlexConfig` for that motor.

---

## Why two SPARK MAXes per module?

A swerve module has **two motors**, and every brushless motor needs its own controller:

| Motor | Job | Controller |
|---|---|---|
| **NEO** (big) | **Drive**: spins the wheel, so the desk moves | SPARK MAX #1, "drive" |
| **NEO 550** (small) | **Steer**: rotates the whole wheel to point it in any direction | SPARK MAX #2, "turn" |

A SPARK MAX switches the motor's three phase wires on and off thousands of times a second, timed
by that motor's own built-in encoder. One SPARK can only do that for one motor. Swerve needs both
motors on every module controlled independently (a wheel can drive while it steers), so each
module needs two. Three modules = 6 SPARK MAXes; the full desk = 8.

The steering SPARK also reads the module's **absolute encoder** (the Through Bore Encoder), so it
knows which way the wheel points even right after power-on.

## 1. Wiring (three modules)

```
                 12 V battery
                      │  (6 AWG)
                120 A main breaker ◄── this is your E-stop on the bench: flip it off
                      │
   ┌─────────────────────── REV PDH (or PDP) ───────────────────────────────┐
   │ ch0 40A ch1 40A │ ch2 40A ch3 40A │ ch4 40A ch5 40A │  10A fused port   │
   └───┬───────┬─────┴───┬───────┬─────┴───┬───────┬─────┴────────┬────────┘
       │12 AWG │         │       │         │       │              │ 18 AWG
    FL-turn FL-drive  FR-turn FR-drive  RL-turn RL-drive      roboRIO 2 (power in)
     ID 10   ID 11     ID 14   ID 15     ID 12   ID 13

 CAN (yellow = CAN-H, green = CAN-L), ONE daisy chain through all six SPARKs, no branches:

 roboRIO ─► FL-turn ─► FL-drive ─► FR-turn ─► FR-drive ─► RL-turn ─► RL-drive ─► PDH CAN
 (built-in 120 Ω)                                                        (termination switch ON)

 Each module (shown for FL; FR and RL are wired the same way):
   NEO 550  ── 3 phase wires (red/black/white, match colours) ──► FL-turn SPARK MAX output
   NEO 550  ── encoder cable (6-pin JST) ──────────────────────► FL-turn SPARK MAX front encoder port
   Through Bore Encoder ── 6-pin cable ─► Absolute Encoder Adapter ─► FL-turn SPARK MAX DATA PORT (top)
   NEO      ── 3 phase wires ──────────────────────────────────► FL-drive SPARK MAX output
   NEO      ── encoder cable ──────────────────────────────────► FL-drive SPARK MAX front encoder port

 Laptop ── USB-B (or Ethernet) ──► roboRIO          (no radio needed on the bench)
 Xbox controller ──► laptop USB                     (only for Joystick mode)
```

The order of SPARKs along the CAN chain doesn't matter (IDs identify them, not position), but
keeping it module by module makes a broken link easy to find. Label each SPARK with its ID and
module (tape + marker) **before** you wire. Swapped SPARKs are the most common bring-up mistake.

Checklist:

- [ ] **Polarity**: red to +, black to −, on every SPARK MAX and on the roboRIO. A reversed SPARK
      MAX power lead destroys it. Check with a multimeter before connecting the battery.
- [ ] **Breakers**: 40 A on each SPARK MAX channel (6 channels for 3 modules). The NEO 550 is also
      current-limited to 20 A in software. The roboRIO goes on the PDH's fused low-current port
      (10 A fuse), never on a 40 A channel.
- [ ] **CAN is one line**: roboRIO, then each SPARK, then the PDH, with the PDH termination switch
      ON. The roboRIO end is already terminated. Don't make stubs or Y-splits. If you have no
      PDH, put a 120 Ω resistor across CAN-H/CAN-L at the last SPARK. To check termination: with
      everything powered OFF, you should measure about **60 Ω** between CAN-H and CAN-L.
- [ ] **Steering motor phase wires** go to the SPARK whose data port holds the encoder adapter.
      Putting the encoder on the drive SPARK is the most common MAXSwerve wiring mistake, and
      the self test reports it as "angle did not change".
- [ ] **Wire routing**: keep wires off the wheel and the steering gear, and zip-tie them to the
      module plate. The module rotates without limit, but your wires can't.
- [ ] **On blocks**: every module clamped down, wheels clear of the table and of each other.

### No frame yet? Build a bench board

Never run a module loose. The steering motor's reaction twists the whole module, and the drive
wheel will throw it off the table. Each module needs to be held by its tube slot, wheel in the air.

- **Per module:** bolt a 6–12 in stub of 2×1 aluminium tube into the module's tube slot with the
  #10-32 through-bolts, exactly as the frame rail will go later. Clamp the stub in a vise, or C-clamp
  it to the bench edge so the wheel overhangs and clears the bench and the floor.
- **All modules at once:** screw the stubs to a sheet of ¾ in plywood (about 2 × 3 ft), wheels hanging
  over the edges, and clamp the plywood to the bench. Mount the SPARK MAXes, PDH, breaker and
  controller on the same board with screws, Velcro or 3D-printed holders, keeping each SPARK's cables
  short and clear of its module's rotation. This board becomes a test copy of the belly-pan layout.
- The 2×1 tube stubs are the same material the frame needs (`PLAN.md` §3), so nothing is wasted.


## 2. One-time setup in REV Hardware Client (USB-C to each SPARK MAX)

Do this with **one SPARK connected at a time** (new SPARKs all ship with CAN ID 0, and IDs must
be unique).

1. **Firmware: decide first.** This program (roboRIO, REVLib 2026) needs current SPARK firmware.
   But if you go with Option B (Raspberry Pi + `sparkcan`), the SPARKs must stay on **24.0.x**:
   `sparkcan` does not work with 25.x or newer. Going back down later means finding and loading
   an older firmware file, which I haven't verified works. **Check which version your SPARKs are on
   now, and don't update until you've chosen.** See
   [`docs/OPTION-B-PI-CAN.md`](../../docs/OPTION-B-PI-CAN.md).
2. **Set the CAN ID** and power-cycle the SPARK after changing it:

   | Module | Turn SPARK | Drive SPARK |
   |---|---|---|
   | FL | 10 | 11 |
   | FR | 14 | 15 |
   | RL | 12 | 13 |
   | RR (later) | 16 | 17 |

3. **Zero the steering (turn SPARKs only)**: open *Absolute Encoder*, set **Direction = Inverted**
   *first*, then fit the MAXSwerve calibration tool on the module so the wheel is locked straight,
   and click **Set Zero Offset**. Repeat for every module. This puts 0° in the same physical place
   on all of them. The program never overwrites this offset.
4. Optional: in Hardware Client, spin each motor at low duty cycle to check that it runs at all
   before involving the roboRIO.

Hardware Client also has a CAN view: with everything connected through one SPARK, you should see
all six SPARKs plus the PDH. If one is missing, the CAN chain is broken at or before that device.

**Test one module first.** If this is your first time, wire only FL, comment FR and RL out of
`kModules` in `Constants.java`, and get FL to PASS. Then add the others. Debugging one module is
much easier than debugging six SPARKs at once.

## 3. Build, deploy, run

Install the [WPILib 2026 suite](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html),
open this folder (`firmware/module-test`) in **WPILib VS Code**, and set your team number
(`Ctrl+Shift+P → WPILib: Set Team Number`; it's 0 here, so update it). Then:

```
./gradlew build          # first build downloads WPILib + REVLib
./gradlew deploy         # laptop connected to the roboRIO
```

Open the Driver Station and a dashboard (Elastic or SmartDashboard). The **Test Mode** chooser
appears there. Pick a mode, then enable **TeleOperated**. **Enter disables and Space is
emergency stop** in the Driver Station, so keep a hand near them. Every enable restarts the
chosen test.

| Mode | Moves? | What it's for |
|---|---|---|
| **Readout** (default) | No | Turn each wheel by hand. `FL/AngleDeg`, `FR/AngleDeg`, `RL/AngleDeg` should change smoothly, 0→360, increasing when the wheel turns counter-clockwise seen from above. With the calibration tool fitted, each should read ≈0. |
| **Self test** | Yes, slow | Runs on all modules at once and reports PASS/FAIL per module (see below). |
| **Joystick** | Only while **RB is held** | Left stick: all wheels point and drive the way the stick points (max 15 %). A/B/X/Y: snap to 0/90/180/270°. Let go of RB and everything stops. |
| **Phone app** | Only while your finger is on the test page | The same as Joystick, driven from your phone through `tests/hardware/app-drive-test` (see its README). Stops if commands stop for 0.3 s. |

**Drive stall cut-off.** On the bench the drive current limit is 20 A (`kBenchDriveCurrentLimitAmps`).
If a wheel is told to move but stays still while drawing over 10 A for half a second, that module's
drive shuts off, `Stalled` shows on the dashboard, and the self test fails. It resets on the next
enable. **Before enabling again, remove power and turn the wheel by hand: it should spin freely.**
This only protects this program. REV Hardware Client's motor test uses the SPARK's own settings,
so keep its speed low and its current limit set when spinning a drive motor installed in a module.

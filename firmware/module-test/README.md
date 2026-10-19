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

## 2. One-time setup in REV Hardware Client (USB-C to each SPARK MAX)

Do this with **one SPARK connected at a time** (new SPARKs all ship with CAN ID 0, and IDs must
be unique).

1. **Update firmware** to the newest version the Hardware Client offers. REVLib 2026 needs current
   SPARK firmware.
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

Each module also shows `ConfigOK`, currents, temperatures and faults under its name on the
dashboard, and its result under `Result/FL`, `Result/FR`, `Result/RL`.

### Self test steps (per module)

1. **Direction check**: 0.6 s of +10 % open-loop steering, which must *increase* the angle.
   This catches a non-inverted encoder **before** closed-loop control runs, because that bug
   makes the module spin non-stop under PID.
2. **Steer sweep**: 0°, 90°, 180°, 270°, 0°. Each must settle within 5° in under 2 s.
3. **Drive**: wheel at 0°, +10 % for 2 s, stop, −10 % for 2 s, stop. The drive encoder must move.

Sample console output (Driver Station → gear icon → View Console) for a good module:

```
[Module FL] running
[Module FL] direction check: +0.1 output moved 63.0 deg
[Module FL] steer   0 deg OK (0.6 s, error 1.2 deg)
[Module FL] steer  90 deg OK (0.7 s, error 0.9 deg)
...
[Module FL] drive +10%: encoder moved +0.790 m
[Module FL] drive -10%: encoder moved -0.805 m
[Module FL] PASS
```

The three modules' lines are interleaved because they run at the same time.

### What failures mean

| Message | Likely cause |
|---|---|
| `SPARKs did not configure` | Wrong CAN ID, CAN wire broken before that SPARK, SPARK unpowered, or firmware too old. |
| `encoder counts backwards` | Encoder inversion isn't taking effect: check the firmware, and that the encoder is on the turn SPARK. |
| `angle did not change` | Encoder adapter not in the turn SPARK's **data port**, encoder cable loose, turn motor phase wire loose, or the drive and turn CAN IDs are swapped (the drive motor moved instead). |
| `steer to N deg timed out` | Mechanical bind (spin the module by hand when it's off), loose encoder magnet or shaft, or the 20 A limit is too low for a tight module. |
| `drive encoder did not move` | Drive motor phase or encoder cable loose, or drive/turn CAN IDs swapped. |
| One module's motor moves when another is tested | Two modules' SPARK IDs are swapped. Check the labels against the ID table. |
| Wheel stutters or buzzes | A phase wire is loose. Check all three crimps. |
| Readout jumps or is noisy | Wrong encoder version: set `kThroughBoreEncoderV2` in `Constants.java` to match your encoder. |

## 4. What to write down for the full drivetrain

- Which physical direction the wheel rolls on `drive +10%` with the wheel at 0°. On the frame, REV's
  chassis angular offsets (FL −90°, FR 0°, RL 180°, RR 90°) assume the standard mounting. Note any
  module you mount differently.
- Drive distance at 10 %. Unloaded, expect roughly 0.4 m/s (about 0.8 m per 2 s step) with the 12T
  pinion, and all modules within a few percent of each other. One much slower module usually has
  a tight bearing or a rubbing wheel.
- Steering settle time at 90°. It should be well under 1 s. Much slower means friction or a low gain.

When the fourth module is built, add `RR` back into `kModules` and test all four. Once every module
passes, mount them and use REV's MAXSwerve template (same CAN IDs) or the
`Summon_Desk_IRL_Hardware_Package/hardware/roborio` code for driving.

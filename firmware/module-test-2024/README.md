# MAXSwerve Module Bench Test: 2024 version (SPARK firmware 24.x)

This is the same bench test as [`../module-test`](../module-test/README.md): same modes, CAN IDs and safety
guards. It's built on **WPILib 2024 and REVLib 2024**, which go with **SPARK MAX firmware 24.0.x**. That's
the firmware the Raspberry Pi route (`sparkcan`) needs, so you can bench test with a roboRIO now and move to
the Pi later without changing firmware.

**Use this one if your SPARKs are on 24.x.** Use `../module-test` (2026) only if you've updated them to 25.x
or later. The 2024 and 2026 versions can't both run on the same firmware.

## What's different from the 2026 instructions

Follow `../module-test/README.md` for wiring, CAN IDs, zeroing, the test modes and the safety checks. Only
the software setup changes:

1. **Install the 2024 tools.** Get the **WPILib 2024** installer (2024.3.2) and **FRC Game Tools 2024**
   (Driver Station and roboRIO Imaging Tool). They install alongside the 2026 versions; open this folder with
   the **2024** WPILib VS Code.
2. **Image the roboRIO for 2024.** Use the 2024 roboRIO Imaging Tool, over USB, with your team number.
   The robot program, the Driver Station and the roboRIO image must all be from the same year.
3. **Leave the SPARK firmware on 24.x.** Don't let REV Hardware Client update it. Check the version on each
   SPARK first.
4. Build and deploy as usual: **WPILib: Build Robot Code**, then **WPILib: Deploy Robot Code**.

## Changes in the code (only `TestModule.java`)

- REV's 2024 classes: `CANSparkMax`, and one call per setting, instead of the 2025+ configuration objects.
- **No "restore factory defaults"**, unlike REV's 2024 example: it can wipe the steering zero offset set in
  Hardware Client. Every setting the module needs is set explicitly, then saved with `burnFlash()`.
- WPILib 2024 has no `optimize()`/`cosineScale()` on a module state, so those lines are written out (never
  turn more than 90°; less drive while the wheel is still turning).
- Assumes the original Through Bore Encoder (see `kThroughBoreEncoderV2` in `Constants.java`).

**Status:** GitHub CI compiles this against WPILib 2024 and REVLib 2024 on every push. It hasn't been run on
hardware yet.

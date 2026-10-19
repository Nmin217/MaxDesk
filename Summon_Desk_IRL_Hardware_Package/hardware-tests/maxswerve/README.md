# MAXSwerve Module Test

Superseded by [`firmware/module-test`](../../../firmware/module-test/README.md), a complete
WPILib 2026 / REVLib 2026 project for one to four modules (readout, scripted self test with
pass/fail, joystick with deadman).

The old `SingleModuleTest.java` was removed because it:
- used the pre-2025 REVLib API (`CANSparkMax`), which no longer compiles;
- did not invert the absolute encoder, so on a MAXSwerve the steering loop pushes the wrong way
  and the module spins continuously;
- had no current limits (a stalled NEO 550 burns out in seconds without one);
- steered without angle wrap (350 deg -> 10 deg went the long way, 340 deg);
- started its step timer at boot, so the first drive step was skipped on enable and the
  sequence never restarted on re-enable.

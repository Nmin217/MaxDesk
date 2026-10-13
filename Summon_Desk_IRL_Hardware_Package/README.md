# Summon Desk — IRL Hardware Package

This package adds real-hardware code to the existing MaxDesk project.
It does NOT replace the simulator, bridge, phone app, or existing repo files.

## Safety first
- Do NOT drive motors from Raspberry Pi GPIO.
- Do NOT run the robot on the ground until bench tests pass.
- Verify your exact LiDAR model before running LiDAR code.
- Verify CAN IDs, encoder offsets, gear ratios, and motor directions.

## Install
Copy folders `hardware-tests/`, `hardware/`, `integration/`, `docs/` into the repo root.
If a folder exists, merge carefully. Do not overwrite unrelated files.

## Communication
Raspberry Pi publishes safety/obstacle data over NetworkTables.
roboRIO subscribes and owns the final drivetrain safety state machine.

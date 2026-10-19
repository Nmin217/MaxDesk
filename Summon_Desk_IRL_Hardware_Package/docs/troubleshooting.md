# Troubleshooting

## LiDAR not found
- Check /dev/ttyUSB*, dmesg, baudrate, dialout group.

## LiDAR opens but no scans
- Check power, cable, model config. Treat as fault, not healthy.

## roboRIO doesn't stop on NT loss
- Check heartbeat updates, Constants.NT_STALE_TIMEOUT_SEC, SafetyStateMachine in loop.
- Add test: disconnect Pi, confirm motors stop.

## MAXSwerve steers wrong
- Check offset, inversion, gear ratio, CAN IDs, firmware.

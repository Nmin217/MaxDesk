# Wiring Checklist
Verify against official REV, WPILib, roboRIO, Pi, and RPLIDAR docs.

## Raspberry Pi
- [ ] Pi powered from a proper regulated supply
- [ ] LiDAR USB adapter connected
- [ ] No motor power through Pi GPIO
- [ ] Network to roboRIO verified

## roboRIO
- [ ] CAN bus wired and terminated per FRC rules
- [ ] MAXSwerve CAN IDs set
- [ ] Motor power fused per rules
- [ ] Emergency disable method available

## MAXSwerve
- [ ] One module on blocks
- [ ] Drive and steer spin freely
- [ ] Absolute encoder reads correct angle
- [ ] Encoder offset measured

## LiDAR
- [ ] Exact model identified
- [ ] Baud rate confirmed
- [ ] Serial port confirmed
- [ ] Forward sector matches robot forward

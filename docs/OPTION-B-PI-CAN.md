# Option B: driving the SPARK MAXes from a Raspberry Pi (no roboRIO)

Researched October 2026. Library and firmware support move quickly, so check each link before buying
anything. A roboRIO stays the fallback if none of these work.

## What any Pi-only setup has to do

A roboRIO does four jobs that the Pi must take over:

1. **Talk CAN.** A Raspberry Pi has no CAN port, so it needs an adapter (below).
2. **Send the enable heartbeat.** SPARKs only drive while they keep hearing a periodic "enabled"
   frame. Stop sending it and they stop. That is also the Pi's disable.
3. **Configure the SPARKs.** For MAXSwerve steering: absolute encoder as the feedback sensor,
   encoder inverted, position wrapping on, 20 A limit on the NEO 550, 40 A on the drive NEO.
4. **Do the swerve maths.** Turn "drive this way at this speed" into an angle and speed for each of
   the four modules. `robot/pi/runtime.js` would call this through `robot/pi/io.js`'s
   `writeWheels(vx, vy, omega)`, in a new `pi_io.js`.

Without a roboRIO's enforced disable, the **hardware E-stop that cuts motor power through a
contactor** is the only safety that doesn't depend on software. Build it before anything spins.

## CAN adapter choices

| Adapter | Connects by | Notes |
|---|---|---|
| CANable 2.0 / candleLight-firmware clones | USB | Shows up as a normal Linux SocketCAN device (`can0`). The one `sparkcan` is tested with. |
| Waveshare-style 2-channel CAN HAT (MCP2515) | Pi GPIO/SPI | No USB cable to knock loose, but needs SPI and device-tree setup. Check 1 Mbit/s support. |

Put a 120 Ω terminator at the adapter end of the CAN chain, as the roboRIO would have had.

## Route 1: `sparkcan` (C++) + a small helper process

[grayson-arendt/sparkcan](https://github.com/grayson-arendt/sparkcan), last updated July 2026.

- Linux library for SPARK MAX and SPARK Flex over SocketCAN, tested with a CANable.
- **Only works with SPARK firmware 24.0.x.** Its README says not to update to 25.x. My roboRIO
  bench test in `firmware/module-test` needs current firmware, so you can't use both on the
  same controllers.
- Has the heartbeat (`SparkBase::Heartbeat()` broadcasts to all devices) and the configuration
  calls swerve needs: feedback sensor selection, duty-cycle (absolute) encoder inversion, zero
  offset and position factor, position-PID wrapping, smart current limits, burn flash.
- **Gap:** it reads status frames 0–4 only. The absolute encoder's angle comes in a later status
  frame, so the Pi could *steer to* an angle (the SPARK runs that loop itself) but not *read back*
  where each wheel points. That reading is needed for the app's wheel display and for odometry.
  It would have to be added (a small change to the library or our own frame parser).
- Our runtime is Node.js and `sparkcan` is C++. The simplest link is a small C++ program that
  owns the CAN bus and takes `{module, angle, speed}` lines from the Node runtime over stdin or a
  local socket, sending back encoder readings. That also keeps the heartbeat in one place.

## Route 2: `sparklib-py` (Python)

[Announcement on Chief Delphi](https://www.chiefdelphi.com/t/sparklib-py-a-python-driver-and-diagnostic-toolkit-for-spark-max-flex-over-raw-can/524123).
A Python driver and diagnostic toolkit for SPARK MAX/Flex over raw CAN that needs only a CAN
interface and `python-can`. I couldn't open the thread from here, so I haven't confirmed which
firmware versions it supports. Check that first: if it supports 25.x/26.x, it removes the firmware
lock-in of Route 1. Its diagnostics alone would be useful for bench testing.

## Route 3: our own SocketCAN driver in Node

Use a Node SocketCAN package to send the heartbeat and setpoint frames and parse status frames
directly, using `sparkcan`'s source as the reference for frame IDs and layouts. One language,
no helper process, and we control which frames we read (including the absolute encoder). The cost:
it's tied to the 24.0.x protocol `sparkcan` documents, and REV changed things in 25.x.

## Route 4: run SystemCore software on the Pi 5

SystemCore is FIRST's new control system for 2027, built on a Raspberry Pi Compute Module 5
([WPILib docs](https://docs.wpilib.org/en/latest/docs/software/systemcore-info/systemcore-introduction.html)).

- A community guide, [netarcx/systemcore-rpi5-guide](https://github.com/netarcx/systemcore-rpi5-guide),
  boots the SystemCore beta on a standard Pi 5. It was tested against WPILib 2027 **Alpha 2**,
  and its author warns later alphas may break it. WPILib is already at 2027.0.0 Alpha 7, whose
  release notes say Alpha 6-and-earlier vendordeps don't work with it
  ([releases](https://github.com/wpilibsuite/allwpilib/releases)).
- If it works, you'd write normal WPILib Java with REV's own library, as on a roboRIO, plus the
  2027 Driver Station to enable.
- Unknowns: whether REVLib's 2027 build runs on this unofficial image, and how CAN is wired
  (a bare Pi 5 still needs a CAN HAT or adapter; the real SystemCore has CAN built in).
- Treat it as an experiment: alpha software on an unofficial image.

**If you ever buy a controller:** the roboRIO is being replaced by SystemCore for 2027, so compare
it with a real SystemCore ([pricing and availability](https://community.firstinspires.org/2026-systemcore-pricing-and-availability))
before buying a roboRIO.

## Suggested order

1. **Check your SPARK firmware** in REV Hardware Client and don't update yet.
2. Buy a CANable-type USB adapter. It's cheap and needed by Routes 1–3, and by Route 4 on a bare Pi.
3. Spin **one** SPARK from the Pi with `sparkcan`'s example program (`example_control`): heartbeat,
   then duty cycle. That proves the adapter, wiring, termination and firmware.
4. Then decide between Route 1 (fastest, firmware-locked) and Route 3 (more work, more control), and
   write a Pi version of `firmware/module-test` (readout, steering-direction check, steer sweep, drive).
5. Try Route 4 in parallel only if you want the WPILib path.

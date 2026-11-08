## Swerve kinematics (swerve.js)

Pure, dependency-free kinematics and odometry for the four REV MAXSwerve modules, for the future
`pi_io.js`. The runtime does not use it. Test: `node tests/pi/test_swerve.js`.

- Robot frame follows WPILib: +x forward (the monitor's back), +y left, CCW+. Modules are ordered FL, FR, RL, RR.
- `WHEELBASE` / `TRACK_WIDTH` are placeholders (0.40 m x 0.60 m). Measure them on the real frame.
- IO field frame (io.js): `heading = theta_io + pi/2` (`ioThetaToHeading`). The axes are shared, so
  `writeWheels` does `fieldToRobot(vx, vy, heading)` -> `toModuleStates` -> `optimize` -> `cosineScale` -> `applyOffset`.
  The header of swerve.js has the details.

## Saved data (room map and no-go zones)

The runtime keeps the learned room map and the no-go zones in `~/.maxdesk` on the Pi (set
`MAXDESK_DATA_DIR`, or pass `dataDir` to `createRuntime`, to use another folder; `dataDir: null`
keeps them in memory only). The map is saved every 10 s while it changes and again on `stop()`;
zones are saved as soon as you change them in the app. Every write goes to a temp file, is flushed,
then renamed, so pulling the power mid-save leaves the previous copy intact. Delete the folder to
make the desk forget the room.

## Motion guard (motion_guard.js)

Two checks sit between the planner and `io.writeWheels`:

- **Slew backstop.** Speed may rise by at most 1.5 m/s² and turning rate by 6 rad/s². The simulator
  core already ramps more gently than that (0.5 m/s² and 2 rad/s² up), so in normal driving this
  changes nothing (a test checks a full summon is passed through untouched). It only catches
  sudden jumps from a bug or state reset. Slowing down and stopping are never delayed.
- **Jam stop.** If `io.readModules()` reports a wheel asked to move (≥ 0.1 m/s) that turns at less
  than a quarter of that while its drive draws over 20 A, or steering more than 30° off target
  while the NEO 550 draws over 12 A, for 0.5 s, the trip is cancelled, the wheels stop, the app
  shows "FL wheel can't turn: jammed or pushing against something" and the History tab logs the
  trip as "Wheel jammed". Nothing resumes on its own; a new summon tries again. `pi_io.js` must
  provide `readModules()` for this to work (see `io.js`).
- **Boxed in** (in the simulator core, so the Pi gets it too). If a trip has neither moved 30 cm
  nor got 30 cm closer for 15 s, for example because it's blocked all round or something is
  sitting on or under the desk where the LiDAR can't see it, the trip ends, the app shows
  "I'm boxed in - clear the space around me, then call me again", and History logs "Boxed in".
  While the desk is lost, E-stopped or cord-locked this timer doesn't run, since those have their
  own messages. Test: `tests/sim/scenarios/boxed.js`.

## Hardware check (hardware_check.js)

At start-up and twice a second after that, the runtime checks the module readings from
`io.readModules()`: every expected module there (`MAXDESK_MODULES`, default `FL,FR,RL,RR`; a
three-wheel cart sets `FL,FR,RL`), its steering encoder reading, supply voltage at least 10.5 V, and no
controller faults. Any problem stops the wheels, ends the current trip ("Hardware problem" in
History), refuses new summons, and shows the first problem in plain words, for example "FR not
answering on CAN: check its CAN wires and power." It clears after 2 s of normal readings. It never
restarts a trip by itself. If `readModules()` throws, motion is blocked. If it returns null (an IO
without module readings), there is no check. Tests: `tests/pi/test_hardware_check.js` and the
hardware tests in `test_runtime.js`.

## Real hardware: pi_io.js and the brain's hardware input

**`pi_io.js`** drives the MAXSwerve modules. It turns the brain's field-frame command into each module's
steering angle and wheel speed, and sends them to the right CAN IDs in motor units:
- rpm through the 5.08:1 gearing and the 3 in wheel;
- steering as absolute-encoder rotations.

It uses the brain's LiDAR-corrected heading to convert from the field frame. A wheel more than 90° from its
target turns the short way and runs backwards, and a wheel that hasn't finished turning gets less speed
(cosine scaling).

It reads back each module's angle, speed, current, temperature, voltage and faults. From those it gives
odometry (from the drive encoders' positions; heading from a gyro if one is configured, else from the
wheels), the hardware check's readings, the jam and wrong-direction checks, and the battery percentage.
If the E-stop or cord input isn't wired in, it reports them as unsafe.

CAN IDs default to the cart's wiring: odd = drive NEO, even = turn NEO 550. FL is 1/2, FR 3/4, RL 5/6 and
RR 7/8. Zero each module's absolute encoder with the wheel pointing at the desk's front.

**The SPARK bus.** `pi_io.js` reaches the controllers only through five calls: `configure`, `setVelocity`,
`setPosition`, `heartbeat` and `status`. The Pi-to-CAN route (a `sparkcan` helper on firmware 24.x, our own
SocketCAN driver, or Python; see `docs/OPTION-B-PI-CAN.md`) only has to provide those five calls.
`fake_spark_bus.js` is the test version:
- drive motors lag;
- steering takes the short way round;
- the controllers stop if the heartbeat stops for 100 ms;
- faults can be switched on: unplugged, encoder unplugged, jammed, wired backwards, low voltage.

**Hardware input.** When the IO is real (`isHardware`), the runtime switches the brain to hardware input. Its
pretend room is emptied (no furniture, no people), and from then on:
- **position** moves by the measured odometry (the change each tick, so LiDAR corrections stick);
- **LiDAR points** are the real scan, minus returns that land on the desk itself;
- **the battery** is the real reading.

The planner, supervisor, map and localisation are unchanged. A scan older than 0.3 s holds the wheels;
older than 2 s ends the trip with "LiDAR not sending data".

**Wrong direction.** A wheel turning against its command for 0.3 s stops the desk with "FL wheel turning the
wrong way: check the motor's phase wires or its inversion setting" (`motion_guard.js`).

**End-to-end test without hardware** (`tests/pi/test_hw_loop.js`): runtime → brain in hardware mode →
`pi_io` → fake SPARKs → a fake room where the true desk moves by what the motors actually did (2% more
distance than the encoders count, and a heading creep of 0.57°/m) → two fake LiDARs → back into the brain.
The brain only ever sees encoder counts and LiDAR rays. The test runs on a fake clock with fixed random
seeds, so every run is identical. It also covers an unplugged LiDAR or SPARK stopping the desk with a plain
message.

### How accurate is it

Worst position error at any moment of a trip, over several random seeds:

| Situation | No gyro | With a gyro |
|---|---|---|
| Summon in a learned room (everyday use) | 4–32 cm | 4–10 cm |
| First trip into an unknown room | 35–37 cm | 17 cm |
| First trip with heavy slip (5% + 1.7°/m creep) | 100–125 cm | 39–41 cm |

Every run arrived, made no contact, and ended within about 7 cm of its true position. The big numbers happen
mid-trip, before walls ahead pull the estimate back. Safety doesn't depend on them: the supervisor checks live
LiDAR points relative to the desk.

- **Fit a gyro.** Without one, the heading creeps between LiDAR corrections. A BNO085 IMU board (about $25,
  on the Pi's pins) cuts the error by two to three times. Pass it to `pi_io` as `imu: { yaw() }`, in radians,
  counter-clockwise positive.
- **Learn the room before first use.** A learned room is where accuracy is best.
- **First-trip accuracy is limited by design.** The LiDAR correction only matches against map marks at least
  8 s old, which is what keeps bump recovery safe. Shortening it to 2 s, scoring returns on the far walls,
  or correcting heading more eagerly each helped here but broke the bump and bad-floor tests (one made a 5 m
  wrong jump), so none was kept. Improving it means retuning the localisation as its own job.

## Black box (black_box.js)

The runtime keeps the last 60 s of what the desk did in memory: 20 samples a second, a LiDAR snapshot once a
second, and every status message. When something goes wrong it records 5 s more, so you also see what
happened next, then saves it as `<dataDir>/blackbox/<date>-<time>-<reason>.json` (on the Pi:
`~/.maxdesk/blackbox/`).

**What each sample holds:**
- where the desk thinks it is and what speed was asked for;
- what went to the wheels: `drive`, `idle`, or `stop:` with the reason (e-stop, bridge silent, cord,
  hardware, lidar old, drive or steer jam, reverse, error);
- mode, localisation state, battery, LiDAR age, E-stop, cord, bridge and the hardware check;
- for each module: angle, asked-for and measured wheel speed, drive and steering current, both motor
  temperatures and voltage.

**When it saves:**
- a trip ending as jam, hardware problem, E-stop, boxed in or cord;
- the E-stop, the hardware check, the LiDAR going quiet or getting lost, each on the moment it starts;
- an error inside the control loop;
- "Save last minute" in the app's History tab (`POST /v1/blackbox`).

Triggers within 10 s of a save belong to the same incident: they're added as events, not saved again. Files
are written atomically (temp file, fsync, rename), and only the newest 20 are kept (a full minute is about
550 KB). A file that can't be written never disturbs the control loop.

**To look at one,** open `tools/blackbox_viewer.html` in a browser (no server needed) and drop the file
on it (try `tools/blackbox_example.json`, a jammed RR wheel from the fake-hardware test). It shows the path on the floor plan with the LiDAR snapshot at each moment, a play/scrub timeline,
the event list, and charts for each module: wheel speed (asked for and measured), drive current, steering
current and temperature. Tests: `tests/pi/test_black_box.js`, and the black box tests in
`tests/pi/test_hw_loop.js` (a jammed wheel, the LiDAR going quiet, saving from the app).

## Position: drift, LiDAR correction and getting lost (prototyped in the simulator)

The simulator's "Sensors & drift" panel models what the real desk will face: wheel odometry and the
gyro drift, so the desk's *believed* position slowly walks away from the *real* one. The fix is in
the simulator core (`robot/simulator/summon_table_sim.html`, search for "LiDAR position correction"),
so `runtime.js` already carries it; on the robot it will run against `pi_io.js`'s real scans.

1. **Tracking**, 5 times a second: slide the scan around the believed pose (±16 cm, ±4°) to line it
   up with the learned map and the walls; small corrections win ties, so it doesn't slide along a
   bare wall.
2. **Suspect**: when the scan disagrees with the map (points on long-known open floor, or beyond a
   wall), or the IMU feels a shove, map updates pause and a wider search (±60 cm, ±12°) runs.
3. **Lost**: if that fails, the desk stops, refuses summons, and searches the whole room every
   second. It only accepts a clear, unique winner (or the same clean winner three times running);
   otherwise it stays stopped and after 10 s asks to be pushed near the dock or for the room to be
   relearned. Being stopped is safe; being confidently wrong is not.

People are tracked separately and left out of the matching. Learn the room first: with a learned map
the error stayed within 6 cm (good floor, gyro) / 15 cm (slippery floor, no gyro) over three round
trips; without correction it reached 68 cm and missed the dock every time. Repeated shoves of up to
~2 m: 97 of 100 random cases found themselves (all but one within 5 s), 2 stayed safely stopped, and 1
on the slippery no-gyro floor ended 66 cm off while showing "weak" (not "locked") after a small
bump: on that floor, long stretches without a good match still drift (`tests/sim/scenarios/drift.js`
runs 12 of the bump cases).

What the real desk needs for this: a LiDAR range of at least ~5 m, a gyro/IMU (also used to feel
shoves), and the room learned once with "Learn the room".

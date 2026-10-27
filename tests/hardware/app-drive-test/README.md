# Bench drive test from your phone

A test tool, separate from the main app and bridge. It lets you drive the MAXSwerve modules on
the bench from your phone, and shows each module's angle, wheel speed, current, temperature and
self-test result.

```
phone (this page) ──Wi-Fi──► laptop: app_link.py ──USB / radio──► roboRIO: module-test, "Phone app" mode
                                     + Driver Station (enable / disable)
```

**Modules on blocks, wheels off the ground.** Speeds are capped by `firmware/module-test`
(15 % of free speed, about 0.67 m/s at the wheel with the 13T pinion), and the page's slider
scales that down further.

## Safety

- The page only sends while your finger is on the pad or on a direction button. Lift it and the page
  sends Stop.
- The robot program stops on its own if no new command has arrived for 0.3 s: phone locked, browser
  switched, Wi-Fi drop, or `app_link.py` closed.
- **The Driver Station is still the real disable.** Keep the laptop in reach. Enter disables and
  space bar is the emergency stop.

## Setup (once)

1. Deploy `firmware/module-test` to the roboRIO (see its README).
2. On the laptop, in this folder: `pip install -r requirements.txt` (Python 3.10+, 64-bit).

## Each session

1. Connect the laptop to the roboRIO (USB cable is fine) and open the Driver Station.
2. Run `python app_link.py`. If you're on the radio instead of USB, use `python app_link.py --team <number>`.
3. It prints an address such as `http://192.168.1.20:8090/?k=Ab3dE9`. Open that on your phone.
   The phone must be on the same Wi-Fi as the laptop. The `k=` part is a one-time key, so only that
   link works, and it changes each time you start the script.
4. In the dashboard, set **Test Mode → Phone app (test link)**, then **Enable TeleOp** in the Driver Station.
   The page's top-right badge shows `APP` when it's ready.
5. Drag on the pad: up is forward, left is left. Every module listed in `Constants.kModules` follows
   it. Hold a direction button to point the wheels without driving.

If Windows asks whether Python may use the network, allow it on private networks, or the phone can't
connect.

## What to look for

- Dragging forward: every wheel points forward and the angle reads about 0°. If one points backwards
  or sideways, redo that module's zero offset.
- Dragging left: the wheels turn to 90°, the short way, and the reading goes up (counter-clockwise).
- Amps settle low when the wheel is spinning freely. A module drawing much more than the others has
  something rubbing.
- `CAN?` on a card means that module's SPARKs didn't answer: check the CAN IDs and the chain.

## Tests

`python3 test_app_link.py` checks the link with a fake roboRIO (no pyntcore needed).

# Summon Desk — GitHub Codespaces quick start

This is the complete project, including the browser simulator, Node bridge, mobile web controller, Java core, iOS starter, tests, proposal, and bill of materials.

## Important: do not open the simulator HTML as a local file

Opening `robot/simulator/summon_table_sim.html` directly (a `file://` URL) will show the visual simulation, but it will **not** be connected to the bridge. The simulator must be served by the bridge at `/simulator` so the same-origin state and command API work.

## Start it

1. Put the contents of this project into your GitHub repository and open that repository in a Codespace.
2. In VS Code, select **Terminal → Run Task… → Summon Desk: Start bridge**. Alternatively, open a terminal at the repository root and run:

   ```bash
   node bridge/bridge_sim.js
   ```

3. Wait for the terminal to say `Summon Desk local web app + simulator bridge on port 8787`.
4. Open the **Ports** tab in the bottom panel. Port `8787` should be forwarded. Use the forwarded HTTP URL for the next two steps.
5. Open `<forwarded-url>/simulator` in a browser tab. Keep this tab open. This is the live simulator instance that publishes state and polls for phone commands.
6. Open `<forwarded-url>/` in another tab or on your Android phone. The root page is the mobile controller.
7. Confirm the controller shows the simulator connected, then test Return, Stop, or Summon. Battery percentages on the controller should match the values in the simulator tab.

## If the controller says disconnected

- Check that the bridge terminal is still running.
- Open `<forwarded-url>/v1/health`; it should return JSON with `"ok":true`.
- Make sure you opened `/simulator` from the forwarded URL, not the file directly from VS Code or a local `file://` path.
- Keep the simulator tab open. The bridge intentionally marks it disconnected if it has not received a fresh state update for a few seconds.
- In the Ports tab, use the same forwarded port URL on the phone. If the port is private, sign in to GitHub in the phone browser or change the port visibility only if your organization permits it. Do not expose this unauthenticated development setup publicly; the API uses a development token and is not hardened for internet exposure.
- If Codespaces shows a different forwarded URL, use the current URL displayed in the Ports tab rather than reusing an old one.

## What the battery values mean

These are simulated battery percentages, not readings from real batteries. The simulator page is the source of truth while it is connected. When the simulator is offline, the controller should indicate that state rather than be treated as showing live readings.

## Physical robot safety

This package is a software simulator and development bridge. It is not a safety-certified controller and must not be connected to motors or used to move a real desk without separate hardware interlocks, validated controls, emergency-stop circuitry, and a full safety review.

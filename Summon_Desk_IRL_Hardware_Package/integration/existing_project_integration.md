# Existing Project Integration

Do not replace the simulator, bridge, phone app, or bridge_sim.js.

Recommended: add bridge/summon_desk_hardware_adapter.js that:
- subscribes to existing Summon/Return/Stop messages
- publishes SummonDesk/Movement/* topics
- reads SummonDesk/Safety/Stop and Obstacle/*
- forwards status back to the phone app

If you must patch an existing file, document exact old lines, new lines, and why.
Never silently overwrite.

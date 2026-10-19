# Safety and command invariants

- When `cordPluggedIn` and the interlock are both enabled, movement commands are refused.
- Stop clears active motion and preempts queued Summon, Return, and other navigation commands.
- A fresh Summon after Stop is accepted only for the current command epoch.
- A browser reconnect advances the epoch and invalidates commands from the prior epoch.
- Once a browser simulator has connected, `drainHeadless()` never replays commands into the internal simulation.
- Poll drains queued commands atomically; a queued command is delivered at most once per epoch.
- A movement command carrying an old epoch returns HTTP 409 with `stale_epoch: true`.

#!/usr/bin/env python3
"""bridge/sim_core.js is GENERATED from robot/simulator/summon_table_sim.html (the single source of truth for behaviour).
   python3 scripts/sync_sim_core.py          regenerate it
   python3 scripts/sync_sim_core.py --check  exit 1 if it is out of date"""
import re, sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
html = (root / "robot/simulator/summon_table_sim.html").read_text()
js = re.findall(r"<script>(.*?)</script>", html, re.S)[0]
core = js[: js.index("// ============ 3D concept model")]      # the 3D preview needs WebGL and is not simulation
out = root / "bridge/sim_core.js"
if "--check" in sys.argv:
    ok = out.exists() and out.read_text() == core
    print("bridge/sim_core.js is up to date" if ok else "bridge/sim_core.js is STALE: run scripts/sync_sim_core.py")
    sys.exit(0 if ok else 1)
out.write_text(core)
print("wrote", out.relative_to(root), len(core), "bytes")

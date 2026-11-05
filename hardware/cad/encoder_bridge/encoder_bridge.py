#!/usr/bin/env python3
"""Parametric stand-in for the MAXSwerve Encoder Bridge (old style, REV-21-3005-P13).

NOT REV's CAD. Every number below is a PLACEHOLDER until it is measured from a real bridge with calipers
(see MEASURE.md). Run:
    pip install cadquery
    python3 encoder_bridge.py                 # writes encoder_bridge.step and encoder_bridge.stl
    python3 encoder_bridge.py --check         # prints which numbers are still placeholders

Shape modelled: a flat plate that screws to the module, a raised pocket on top that holds the Through Bore
Encoder, and a clearance hole through the middle for the shaft. If your bridge has a feature this doesn't (a
clip, a rib, a notch), send a photo and it gets added.
"""
import sys
import cadquery as cq

# ---- Dimensions in mm. measured=False means "a guess: measure it". -----------------------------------------------
P = {
    # Plate (the part that screws down)
    "plate_length":      (60.0, False),   # A: long side, end to end
    "plate_width":       (30.0, False),   # B: short side
    "plate_thickness":   (4.0,  False),   # C
    "plate_end_radius":  (6.0,  False),   # D: corner/end rounding (half of B = fully round ends)
    # Mounting holes (screws into the module)
    "mount_hole_dia":    (3.4,  False),   # E: hole diameter (a #10 screw would be ~5.0, M3 clearance 3.4)
    "mount_hole_spacing":(48.0, False),   # F: centre to centre
    "mount_counterbore_dia":  (0.0, False),   # G: 0 = none
    "mount_counterbore_depth":(0.0, False),   # H
    # Shaft clearance hole (through the centre)
    "shaft_hole_dia":    (10.0, False),   # J: the shaft must NOT touch this: measure the old one, keep >= 0.5 mm clearance
    # Encoder pocket / boss (what the Through Bore Encoder sits in)
    "boss_dia":          (26.0, False),   # K: outside of the round boss on top (0 = no boss)
    "boss_height":       (3.0,  False),   # L: how far it stands above the plate
    "pocket_dia":        (22.0, False),   # M: the recess inside the boss the encoder hub/body sits in (0 = none)
    "pocket_depth":      (2.0,  False),   # N
    # Printing
    "hole_shrink_comp":  (0.15, True),    # holes print small: add this to every hole diameter (tune for your printer)
}
v = {k: x[0] for k, x in P.items()}

def build():
    comp = v["hole_shrink_comp"]
    plate = (cq.Workplane("XY").rect(v["plate_length"], v["plate_width"]).extrude(v["plate_thickness"])
             .edges("|Z").fillet(min(v["plate_end_radius"], v["plate_width"] / 2 - 0.01)))
    body = plate
    if v["boss_dia"] > 0 and v["boss_height"] > 0:
        boss = (cq.Workplane("XY").workplane(offset=v["plate_thickness"])
                .circle(v["boss_dia"] / 2).extrude(v["boss_height"]))
        body = body.union(boss)
    top = v["plate_thickness"] + (v["boss_height"] if v["boss_dia"] > 0 else 0)
    if v["pocket_dia"] > 0 and v["pocket_depth"] > 0:
        body = body.cut(cq.Workplane("XY").workplane(offset=top - v["pocket_depth"])
                        .circle((v["pocket_dia"] + comp) / 2).extrude(v["pocket_depth"] + 1))
    body = body.cut(cq.Workplane("XY").workplane(offset=-1).circle((v["shaft_hole_dia"] + comp) / 2).extrude(top + 2))
    s = v["mount_hole_spacing"] / 2
    holes = cq.Workplane("XY").workplane(offset=-1).pushPoints([(-s, 0), (s, 0)])
    body = body.cut(holes.circle((v["mount_hole_dia"] + comp) / 2).extrude(v["plate_thickness"] + 2))
    if v["mount_counterbore_dia"] > 0 and v["mount_counterbore_depth"] > 0:
        cb = (cq.Workplane("XY").workplane(offset=v["plate_thickness"] - v["mount_counterbore_depth"])
              .pushPoints([(-s, 0), (s, 0)]).circle((v["mount_counterbore_dia"] + comp) / 2).extrude(v["mount_counterbore_depth"] + 1))
        body = body.cut(cb)
    return body

if __name__ == "__main__":
    todo = [k for k, (_, m) in P.items() if not m]
    if "--check" in sys.argv:
        print("still placeholders:" if todo else "all measured", *todo, sep="\n  ")
        sys.exit(1 if todo else 0)
    if todo:
        print(f"WARNING: {len(todo)} dimensions are placeholders (see MEASURE.md). This will NOT fit yet.")
    part = build()
    cq.exporters.export(part, "encoder_bridge.step")
    cq.exporters.export(part, "encoder_bridge.stl", tolerance=0.02, angularTolerance=0.1)
    bb = part.val().BoundingBox()
    print(f"wrote encoder_bridge.step / .stl  ({bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm)")

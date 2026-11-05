# Measuring the old encoder bridge (REV-21-3005-P13)

`encoder_bridge.py` builds a printable copy from YOUR measurements. It is not REV's CAD: until every letter
below is measured, it will not fit. Use calipers, in millimetres, to 0.1 mm. Measure twice.

    TOP VIEW                                   SIDE VIEW (cut through the middle)

     <-------------- A -------------->                 <---- K ---->
     .------------------------------.                   <-- M -->
    (   o          ( (  J  ) )    o  )  B          ____|__.     .__|____     L (boss above plate)
     '------------------------------'             |    |__|     |__|    |    N (pocket depth)
         <--------- F --------->                  |_______|  J  |_______|    C (plate)
     o = mounting hole, diameter E
     D = how round the ends/corners are (radius)

| Letter | What | Notes |
|---|---|---|
| A | Overall length | end to end |
| B | Overall width | |
| C | Plate thickness | at the screw holes |
| D | End / corner radius | half of B if the ends are fully round |
| E | Mounting hole diameter | |
| F | Mounting hole spacing | centre to centre: measure edge to edge across the outside of both holes, minus E |
| G, H | Counterbore diameter and depth | 0 if the screw heads sit flat on top |
| J | Centre hole diameter | the shaft goes through this. Keep at least 0.5 mm clearance: this is where yours rubbed |
| K | Outside diameter of the raised ring on top | 0 if there is none |
| L | How far the ring stands above the plate | |
| M | Inside diameter of the recess the encoder sits in | 0 if none |
| N | Depth of that recess | |

Also send **photos**: top, bottom and side, with a ruler in the shot. Clips, ribs, notches or anything the sketch
above doesn't show get added to the model from those.

The bridge you cut out is damaged where it rubbed. Measure the hole J on an undamaged edge, or from the
Through Bore Encoder it fits around.

## Printing it

- **Material:** PETG or plain ABS is fine. Nylon (PA) is best near a spinning shaft. Avoid ABS-GF on any
  face that could rub: the glass fibre is abrasive.
- Orientation: flat side down, 0.2 mm layers, 4 walls, 40%+ infill.
- Holes print small: `hole_shrink_comp` (0.15 mm) widens them. Print one and check against the screws and
  the shaft before printing more. Drill or ream to final size if needed.
- Before driving: fit it, spin the shaft by hand (it must not touch the centre hole), then bench test both
  directions at low power.

Remember this is a copy of the OLD design. It doesn't fix a key that slides out of place; the V1.1 kit
(new key + spacer + P26 bridge) does.

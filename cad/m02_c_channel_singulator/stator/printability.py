"""Printability batch for the stator — run, don't import.

    .venv/bin/khana run cad/m02_c_channel_singulator/stator/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../stator/assembly.py`); this script
runs the per-part `inspect()` pass and its waivers.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from .assembly import WALL_MIN, exit_floor, stator_part, stator_sectors


# Printed inverted (top face on the bed): the cylinder zone and the
# flange rise from the bed, the V-cone hangs inward at 45° — the
# brick-facing inner face is the overhung one, at the 50° threshold
# with a small margin. Upright, the flange's underside would be a 90°
# ledge.
method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=50.0, up_axis=(0, 0, -1))
# Inspected as the three printed sectors; the ears stand on the bed
# (the cylinder zone's top) as vertical-walled blocks. Every sector
# reaches the V-bottom knife edge, so each carries the same waiver.
_KNIFE_EDGE_WAIVER = {
    "wall_min": "wedge tip at the designed V-bottom knife edge, not "
    "a wall — min_wall_at witnesses (sectors, 2026-09-17): stator_0 "
    "(-82.4, 90.9, 0) and stator_1 (67.9, -102.2, 0), both r≈122.7 "
    "reading 0.70, stator_2 (114.1, 43.2, 0.03) r=122.0 reading 0.04 "
    "— all at z≈0 with alignment 0.71, and 0.707 = cos(V_HALF_ANGLE_DEG=45°) is "
    "exactly the dihedral between the flat bottom annulus (z=0, "
    "r=122..125) and the inner V-cone face (r = 122 + z). Rays "
    "leave the bottom face at r = 122 + δ and exit the cone after "
    "δ, so the reading is the closest facet centroid to that "
    "circle, not a design dimension: all 40 sub-1.5 mm samples "
    "cluster at z≈0, r ∈ [122, 123] at the same 0.71. The wall is "
    "STATOR_WALL_T (3.0) measured perpendicular to the cone; the "
    "inner edge tapering to zero at the V-bottom is the trough "
    "geometry itself (see the frame note in the module docstring) "
    "and prints as the topmost edge in the inverted orientation. "
    "Truncating it would put a ledge at the exact circle bricks "
    "slide through.",
}
for k, sector in enumerate(stator_sectors()):
    inspect(
        sector, method=method, out="outputs", name=stator_part(k),
        waive=_KNIFE_EDGE_WAIVER,
    )

# Exit floor: printed top face down (the brick surface on the bed),
# rib rising — a flat plate with a vertical arc wall, nothing
# overhangs.
inspect(
    exit_floor(), method=FDM(up_axis=(0, 0, -1)), out="outputs", name="exit_floor",
)

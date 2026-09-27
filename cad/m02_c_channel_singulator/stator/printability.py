"""Printability batch for the stator — run, don't import.

    .venv/bin/khana run cad/m02_c_channel_singulator/stator/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../stator/assembly.py`); this script
runs the per-part `inspect()` pass and its waivers.
"""

from cad_khana.printability.feature import Feature
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM
from cad_khana.printability.waiver import Waiver

from ..params import HOPPER_WALL_H
from .assembly import (
    WALL_MIN,
    exit_floor,
    knife_edge_ring,
    sector_solid_count,
    stator_part,
    stator_sectors,
)


# Printed inverted (top face on the bed): the cylinder zone and the
# flange rise from the bed, the V-cone hangs inward at 45° — the
# brick-facing inner face is the overhung one, at the 50° threshold
# with a small margin. Upright, the flange's underside would be a 90°
# ledge.
method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=50.0, up_axis=(0, 0, -1))
# Inspected as the three printed sectors; the ears stand on the bed
# (the cylinder zone's top) as vertical-walled blocks. Every sector
# reaches the V-bottom knife edge, so each carries the same feature
# waiver — scoped to `knife_edge_ring()`, the edge's own two faces, so
# a thin wall anywhere else on the sector (a boss's insert hole run
# toward the trough, a new cut) traces to no feature and fails. That is
# the check slice 5 did by hand for the bosses; probed 2026-09-27, a
# hole brought to 1.22 mm of the trough goes red here and one at
# 1.58 mm does not.
_KNIFE_EDGE = {
    "knife_edge": Feature(knife_edge_ring(), waive={"wall_min": Waiver(
        reason="wedge tip at the designed V-bottom knife edge, not a "
        "wall — every reading runs from the flat bottom annulus (z=0, "
        "r=122..125) into the inner V-cone (r = 122 + z), at alignment "
        "0.71 = cos(V_HALF_ANGLE_DEG=45°), their dihedral. A ray leaving "
        "the bottom at r = 122 + δ exits the cone after δ, so the reading "
        "is set by how near the edge the closest sample lands — not a "
        "design dimension (re-run 2026-09-27: 0.6956 at r≈122.7 on "
        "sectors 0 and 1, 0.0421 at (114.1, 43.2, 0.03) on sector 2, "
        "both stages). The wall is STATOR_WALL_T (3.0) perpendicular to "
        "the cone; the edge tapering to zero at the V-bottom is the "
        "trough itself and prints as the topmost edge inverted. "
        "Truncating it would put a ledge at the exact circle bricks "
        "slide through.",
        min_wall_mm=0.0416,
    )}),
}
# The two arcs of the split sector carry the free-end bosses
# (`../arc_fastening.py`): the insert holes are teardrops roofed toward
# −Z, this orientation's up, so nothing new overhangs, and they sit
# 5.5 mm from the trough — no thin wall of their own to waive.
# `solid_count=` from the same `sector_solid_count` the mechanism
# claim uses: the sector carrying the exit opening arrives as two
# arcs at wall height, and this file is the last gate before a print
# batch.
for k, sector in enumerate(stator_sectors()):
    inspect(
        sector, method=method, out="outputs", name=stator_part(k),
        features=_KNIFE_EDGE, solid_count=sector_solid_count(k),
    )
# The hopper stage's liner: same sectors, the cylinder zone (and the
# ears) up to HOPPER_WALL_H. Inverted, the wall over the exit opening
# faces up — its underside at STATOR_WALL_H is no overhang — and the
# paddle hangs off that wall on its 45° gusset.
for k, sector in enumerate(stator_sectors(height=HOPPER_WALL_H)):
    inspect(
        sector, method=method, out="outputs", name=f"hopper_{stator_part(k)}",
        features=_KNIFE_EDGE,
        solid_count=sector_solid_count(k, HOPPER_WALL_H),
    )

# Exit floor: printed top face down (the brick surface on the bed),
# rib rising — a flat plate with a vertical arc wall, nothing
# overhangs.
inspect(
    exit_floor(), method=FDM(up_axis=(0, 0, -1)), out="outputs", name="exit_floor",
)

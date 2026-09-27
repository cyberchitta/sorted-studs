"""
sorted-studs — driver access to the m02 liner's free-end screws. A
CHECK module: the whole machine plus the claims the fixture exists for.

    khana check cad/check_arc_screw_access.py

`khana check cad/assembly.py` covers every part the screws touch or
pass (the m02 unit's own claims, cross-unit interference); it does not
evaluate these claims, because the driver corridors exist only here.

The claim (slice 5 of the 2026-09-27 completion pass): on every stage
whose liner the exit opening cuts through, each free-end screw
(`m02_c_channel_singulator/arc_fastening.py`) can be driven from
outside with the machine assembled — a straight corridor
`ACCESS_D` across and `ACCESS_L` long runs out along the screw's own
axis from its head top, and every body in the machine stays
`ACCESS_MARGIN` clear of it. Three bodies are not measured, the
three the screw joins: the screw itself (the corridor starts on its
head), the bowl sector it is driven into (the corridor starts in that
sector's countersink) and the liner sector it fastens (its boss sits
right behind the skirt, 2.6 mm off the corridor's start, and would be
every claim's nearest body, masking whatever stands in front). Outside
the skirt the bowl sector carries only its seam ear pairs —
`arc_fastening_to_bowl_seam_deg` holds the fastening an ear's width off
every seam — and the liner only its cylinder zone above SKIRT_TOP_Z,
11 mm above the corridor's top.

Measured, not probed: the corridor is never placed (it would sit in
its own bowl sector's countersink and read as an interference there),
so each claim is a `<stage>_arc_screw_<end>_access_clearance` scalar —
the least distance from the corridor to any other body, capped at
`NEAR_MM` (bodies whose bounding box stays farther off than that are
not measured).
"""

from __future__ import annotations

import re

from build123d import Cylinder, Pos

from cad_khana.mechanism.assembly import Assembly

from .assembly import build_top
from .design_systems import DEFAULT_SCHEMA
from .m02_c_channel_singulator.arc_fastening import arc_screw_azimuth_deg
from .m02_c_channel_singulator.drive.assembly import bowl_part_at
from .m02_c_channel_singulator.params import LINER_SEAM_PHASE_DEG
from .m02_c_channel_singulator.seams import sector_index
from .m02_c_channel_singulator.stator.assembly import STATOR_PARTS
from .m02_chain.params import STAGES

# A hex driver for the M3 countersunk screw's 2 mm socket: Ø8 takes a
# driver's shank with the fingers' room of the M4 clamp bolts' corridor
# (m03 camera mount, CLAMP_BOLT_ACCESS_D), and 60 mm is a stubby
# driver's reach. Margin as there.
ACCESS_D = 8.0
ACCESS_L = 60.0
ACCESS_MARGIN = 0.5
NEAR_MM = 30.0

_SCREW = re.compile(r"m02_chain\.(s\d)\.arc_screw_(pos|neg)")


def build_arc_screw_access_check(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The whole machine, with each free-end screw's driver corridor
    measured against every other body."""
    top = build_top(schema)
    phase = {s.name: s.bowl_seam_phase_deg for s in STAGES}
    world = [(pp.name, pp.part.moved(pp.location)) for pp in top.placed_parts]
    # The claims are scalars measured on ``top``; the fixture carries
    # nothing else, so this run does not re-evaluate the machine's own
    # claims (`khana check cad/assembly.py` does).
    a = Assembly()
    screws = [pp for pp in top.placed_parts if _SCREW.fullmatch(pp.name)]
    assert screws, "no free-end screw in the machine — the claims would be vacuous"
    for pp in screws:
        stage, end = _SCREW.fullmatch(pp.name).groups()
        sign = +1 if end == "pos" else -1
        own_bowl = f"m02_chain.{stage}.drive.{bowl_part_at(arc_screw_azimuth_deg(sign), phase[stage])}"
        own_liner = f"m02_chain.{stage}.stator.{STATOR_PARTS[sector_index(0.0, LINER_SEAM_PHASE_DEG)]}"
        corridor = (Pos(0, 0, ACCESS_L / 2.0) * Cylinder(ACCESS_D / 2.0, ACCESS_L)).moved(pp.location)
        reach = corridor.bounding_box()
        clearance, nearest = NEAR_MM, "nothing"
        for name, body in world:
            if name in (pp.name, own_bowl, own_liner):
                continue
            box = body.bounding_box()
            if any(
                lo - NEAR_MM > hi_r or lo_r - NEAR_MM > hi
                for lo, hi, lo_r, hi_r in (
                    (box.min.X, box.max.X, reach.min.X, reach.max.X),
                    (box.min.Y, box.max.Y, reach.min.Y, reach.max.Y),
                    (box.min.Z, box.max.Z, reach.min.Z, reach.max.Z),
                )
            ):
                continue
            d = corridor.distance_to(body)
            if d < clearance:
                clearance, nearest = d, name
        a = a.assert_scalar(
            f"{stage}_arc_screw_{end}_access_clearance", clearance, ge=ACCESS_MARGIN,
            detail=f"mm from the Ø{ACCESS_D:g} × {ACCESS_L:g} driver corridor to the nearest other body, "
                   f"{nearest} (capped at {NEAR_MM:g})",
        )
    return a


# Degenerate memoized master, so ``khana check <file>`` resolves
# without a ``:factory``.
assembly = build_arc_screw_access_check(DEFAULT_SCHEMA)

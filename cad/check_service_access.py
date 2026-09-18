"""
sorted-studs — hand access to the IMAGE station from the machine's
sides. A CHECK module: probe fixture + the one claim it exists for.

    khana check cad/check_service_access.py

`khana check cad/assembly.py` covers cross-unit interference, the
anchor coincidences and the gantry rests; it does not evaluate this
claim, because the probe solids live only here (nothing imports a
check module, so no export ever carries them).

The claim (owner 2026-09-18, `_notes/state/m03-scanner.md` §Open
work item 0): the operator stands at −Y, where the hopper is loaded,
but the m02 chain fills that side of the turret, so IMAGE is serviced
from ±X — clearing a jam, reaching a platform. A corridor one hand
wide (`REACH_W`) and `REACH_H` tall runs from the IMAGE platform's
side edge out to the deck edge on each side, just behind the LOAD /
IDLE platforms (y band starts `REACH_GAP` past their outer edge) and
above the IMAGE platform top. Nothing in any unit may enter it. The
probe is a box, not a hand, and it is taken at the turret's rest pose
(a platform mid-index sweeps through it; nobody reaches in then).
"""

from __future__ import annotations

from build123d import Box, Part, Pos

from cad_khana.mechanism.assembly import Assembly

from .assembly import build_top
from .design_systems import DEFAULT_SCHEMA
from .m03_scanner.params import PLATFORM_ACRYLIC_SIZE, TURRET_ARM_R

REACH_W = 100.0    # across the knuckles ~90, plus a little
REACH_H = 130.0
REACH_GAP = 5.0    # off the LOAD / IDLE platforms' +Y edge and the IMAGE top


def _corridor(length: float) -> Part:
    """Along +X from x = 0, y 0…REACH_W, z 0…REACH_H."""
    return Pos(length / 2.0, REACH_W / 2.0, REACH_H / 2.0) * Box(length, REACH_W, REACH_H)


def build_service_access_check(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The whole machine plus a side-reach corridor to IMAGE on each
    side, asserted clear of every unit."""
    top = build_top(schema)
    nodes = {n.name: n for n in top.subassemblies}
    m03, m05 = nodes["m03"], nodes["m05"]

    # Turret hub in world: one arm back (+Y) from the DUMP centroid,
    # as in `check_turret_envelope.py`. LOAD / IDLE sit on the hub's
    # X line, so their outer +Y edge is half a platform past the hub.
    hub = (m03.location * m03.assembly.anchor("dump_centroid") * Pos(0, TURRET_ARM_R, 0)).position
    y0 = hub.Y + PLATFORM_ACRYLIC_SIZE / 2.0 + REACH_GAP

    world = {pp.name: (pp.location * pp.part).bounding_box() for pp in top.placed_parts}
    image_top_z = max(b.max.Z for n, b in world.items() if n.startswith("m03.turret.rotor.platform_image."))
    deck_r = world["m05.top_acrylic"].max.X
    x_in = PLATFORM_ACRYLIC_SIZE / 2.0          # IMAGE platform side edge
    length = deck_r - x_in
    z0 = image_top_z + REACH_GAP

    probes = ["service_reach_pos_x", "service_reach_neg_x"]
    a = (
        Assembly()
        .with_subassembly("m05", m05.assembly, location=m05.location)
        .with_subassembly("m03", m03.assembly, location=m03.location)
        .with_subassembly("m02_chain", nodes["m02_chain"].assembly, location=nodes["m02_chain"].location)
        .with_part("service_reach_pos_x", _corridor(length),
                   location=Pos(hub.X + x_in, y0, z0))
        .with_part("service_reach_neg_x", _corridor(length),
                   location=Pos(hub.X - x_in - length, y0, z0))
        .assert_scalar("service_reach_y0", y0, detail="corridor's -Y face, world")
        .assert_scalar("service_reach_z0", z0, detail="corridor floor: IMAGE platform top + REACH_GAP")
    )
    for unit in ("m02_chain", "m03", "m05"):
        a = a.assert_no_interference_between(probes, unit)
    return a


# Degenerate memoized master, so ``khana check <file>`` resolves
# without a ``:factory``.
assembly = build_service_access_check(DEFAULT_SCHEMA)

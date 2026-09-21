"""
sorted-studs — the m03 turret's loaded-platform envelope vs the m02
chain. A CHECK module: probe fixture + the one claim it exists for.

    khana check cad/check_turret_envelope.py

`khana check cad/assembly.py` covers cross-unit interference, the
anchor coincidences and the gantry rests; it does not evaluate this
claim, because the probe solid lives only here (nothing imports a
check module, so no export ever carries it).

The claim (owner 2026-09-14, `_notes/decisions/m02-chain.md` §11.6):
every part of the m02 chain — the LOAD ramp included since 2026-09-15 —
stays TURRET_SWEEP_CLEAR clear of the volume the turret sweeps — the probe is two tiers in one
solid: the turret hardware's own swept cylinder (hub to the platforms'
outer corners, deck to the rest-pose hardware top) over the full
turn, plus the *loaded* sweep — a platform square swept anticlockwise
from LOAD through IMAGE and IDLE to DUMP (a 270° annular sector plus
the two end squares, exactly the swept region) raised by
`BRICK_SETTLED_H`. The DUMP → LOAD return quadrant carries no brick,
so it stays at hardware height, as the explorer modelled it. This is
what set S1's raise (`m02_chain/params.py:S1_VBOT_WORLD_Z`); here the
geometry measures it, radially as well as vertically.

The LOAD ramp was excluded until 2026-09-15, when its outlet hung 7 mm
over the platform's centre inside the sweep. It now ends outside the
loaded platforms' corner circle by the same clearance
(`m02_chain/params.py:RAMP_OUTLET_X` is derived from the same
constants this probe is built from), so the ramp is held to the claim
like every other part. The hardware top is the turret's rest-pose
bounding box (the DUMP lifter's raised pose is not sampled).
"""

from __future__ import annotations

import math

from build123d import Box, Cylinder, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly

from .assembly import M03_ROT_DEG, build_top
from .design_systems import DEFAULT_SCHEMA
from .m02_chain.params import TURRET_SWEEP_CLEAR
from .m03_scanner.params import (
    BRICK_SETTLED_H,
    PLATFORM_ACRYLIC_SIZE,
    TURRET_ARM_R,
)

PLATFORM_CORNER_R = math.hypot(TURRET_ARM_R + PLATFORM_ACRYLIC_SIZE / 2.0, PLATFORM_ACRYLIC_SIZE / 2.0)
PLATFORM_INNER_R = TURRET_ARM_R - PLATFORM_ACRYLIC_SIZE / 2.0          # 65
LOADED_ARC_DEG = 270.0     # LOAD (0°) → IMAGE → IDLE → DUMP (270°), anticlockwise


def _probe(hw_h: float, loaded_h: float) -> Part:
    """Turret-local (hub at the origin, LOAD on +X, z = 0 at the
    deck): the hardware cylinder to `hw_h`, plus the loaded platform
    sweep to `loaded_h`."""
    hardware = Pos(0, 0, hw_h / 2.0) * Cylinder(PLATFORM_CORNER_R, hw_h)
    sector = Pos(0, 0, loaded_h / 2.0) * (
        Cylinder(PLATFORM_CORNER_R, loaded_h, arc_size=LOADED_ARC_DEG)
        - Cylinder(PLATFORM_INNER_R, loaded_h + 2.0)
    )
    square = Box(PLATFORM_ACRYLIC_SIZE, PLATFORM_ACRYLIC_SIZE, loaded_h)
    at_load = Pos(TURRET_ARM_R, 0, loaded_h / 2.0) * square
    at_dump = Rot(0, 0, LOADED_ARC_DEG) * Pos(TURRET_ARM_R, 0, loaded_h / 2.0) * square
    return hardware + sector + at_load + at_dump


def build_turret_envelope_check(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The chain at its world pose plus the loaded-sweep probe, with a
    clearance claim per chain part."""
    top = build_top(schema)
    nodes = {n.name: n for n in top.subassemblies}
    chain = nodes["m02_chain"]

    # Turret hub in world: the DUMP centroid is TURRET_ARM_R from the
    # hub along the turret's local -Y, so the hub is one arm back along
    # the placed unit's +Y.
    dump = top.anchor("m03.dump_centroid").position
    hub = (top.anchor("m03.dump_centroid") * Pos(0, TURRET_ARM_R, 0)).position
    deck_top_z = top.anchor("m05.deck_top").position.Z

    # Turret hardware top at rest: the highest point of anything under
    # the turret subtree.
    hw_top = max(
        (pp.location * pp.part).bounding_box().max.Z
        for pp in top.placed_parts
        if pp.name.startswith("m03.turret.")
    )
    hw_h = hw_top - deck_top_z
    loaded_h = hw_h + BRICK_SETTLED_H

    a = (
        Assembly()
        .with_subassembly("m02_chain", chain.assembly, location=chain.location)
        .with_part(
            "turret_loaded_envelope",
            _probe(hw_h, loaded_h),
            location=Pos(hub.X, hub.Y, deck_top_z) * Rot(0, 0, M03_ROT_DEG),
        )
        .assert_scalar("turret_hardware_top_z", hw_top,
                       detail="rest-pose bbox top of m03.turret.*; the probe rises BRICK_SETTLED_H above it")
        .assert_scalar("turret_hub_xy_x", hub.X, detail=f"dump centroid at ({dump.X:.1f}, {dump.Y:.1f})")
    )
    for pp in chain.assembly.placed_parts:
        a = a.assert_distance(
            "turret_loaded_envelope", f"m02_chain.{pp.name}", min_mm=TURRET_SWEEP_CLEAR,
        )
    return a


# Degenerate memoized master, so ``khana check <file>`` resolves
# without a ``:factory``.
assembly = build_turret_envelope_check(DEFAULT_SCHEMA)

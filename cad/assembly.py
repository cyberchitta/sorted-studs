"""
sorted-studs — top-level integration assembly.

m05 diverter + m03 scanner (rig + turret) + m02 chain (three
singulator stages, the gantry that carries them on two of m05's
through-deck columns, and the LOAD ramp). The chain delivers bricks
onto the m03 turret's LOAD station; the turret rotates them through
IMAGE → IDLE → DUMP over the m05 hub axis.

Constraints landed in this slice:

  * **DUMP over hub.** The turret's DUMP station is its "down
    position" — at every dwell the dump servo tilts that platform so
    the brick falls. For the brick to drop into the diverter's
    central column, DUMP must sit on the column's hub axis at world
    `(0, 0, *)`. Option (a') lands the firing heading at 270° (-Y in
    turret-local), so DUMP lives at turret-local `(0, -TURRET_ARM_R,
    0)` and the turret-local origin maps to world
    `(0, +TURRET_ARM_R, *)`. Since 2026-10-06 `M03_DUMP_OFFSET`
    moves DUMP out along the turret radial for the top feed (0 is
    exactly over the hub).
  * **Turret rests on the deck.** The turret floor frame's bottom
    face sits on the diverter top deck's top face at world z =
    `TOP_VERT_Z + TOP_DECK_T = 890` — and, with everything else
    standing there, inside its rim (`on_deck_*`, `DECK_EDGE_MARGIN`).

Cross-module no-interference is asserted (every pair of units), the
units' beliefs about the shared datums are asserted coincident, and
the gantry's rests on m05 — prop bars across the column ends, S2's
stem on V_4's face, the braces' clearances to the columns and the
deck — are declared here, the level that composes both. So are the
driver corridors to the m02 liner's free-end screws: keep-outs held
against every unit, because a corridor reaches past its stage — and
the hand-wide reach corridors to IMAGE from ±X, clear of every unit at
the rest pose. The turret's
loaded-platform envelope against the chain is a probe fixture, so it
lives in `check_turret_envelope.py` (a check module: `khana check` it
separately; this file never carries the probe). Within-module
assertions propagate and are evaluated here too (see below).

Coordinate frame (world):
    +X = operator's right
    +Y = away from operator (back of machine)
    +Z = up (gravity is -Z)
    z=0 = diverter base-floor rail datum.

Composition strategy:

`cad/` is a package: each unit is imported as a plain module
(`cad.m05_diverter.assembly` etc. — no name collisions, no loader
machinery) and composes as a true subtree — `with_subassembly(name,
…, location=world_pose)` — so parts keep their tree paths under the
unit root (`m05.…`, `m03.…`, `m02_chain.…`) and every unit's joints
survive to the top level: the whole machine can animate from here.
World poses are *derived from the units' exported anchors* (m05
`deck_top`, m03 `dump_centroid` / `floor_bottom` / `load_top`), and
the chain's mirrored beliefs are asserted coincident with the live
units after composition — no mirrored placement constants live here.
Within-unit assertions propagate: this root's `check()` evaluates
every unit's nested claims too, qualified into machine coordinates, so
only the cross-unit claims are declared here.
"""

from __future__ import annotations

from functools import cache

import math

from build123d import Box, Location, Pos, Rot

from cad_khana.mechanism.assembly import Assembly

from .design_systems import DEFAULT_SCHEMA
from .m02_c_channel_singulator.arc_fastening import FREE_END_SIGNS, arc_screw_azimuth_deg, free_end_name
from .m02_c_channel_singulator.drive.assembly import bowl_part_at
from .m02_c_channel_singulator.params import LINER_SEAM_PHASE_DEG
from .m02_c_channel_singulator.seams import sector_index
from .m02_c_channel_singulator.stator.assembly import STATOR_PARTS, exit_splits_wall
from .m02_chain.assembly import build_chain
from .m02_chain.params import BRACE_COLS, PROP_POST, STAGES
from .m03_scanner.assembly import build_m03
from .m03_scanner.params import PLATFORM_ACRYLIC_SIZE, TURRET_ARM_R
from .m05_diverter.assembly import build_m05


# m05 at world identity — its local frame IS the world frame.
_M05_POSE = Location()

# Turret rotation ψ (owner's lever, 2026-09-15): the whole m03 unit —
# and the chain hanging off its LOAD station — turned about the DUMP
# centroid (on the m05 hub axis while M03_DUMP_OFFSET is 0). 0 at
# family P (`_notes/decisions/m02-chain.md` §4, §11.8). The chain's
# mirrored beliefs (`m02_chain/params.py`) assume 0: a non-zero ψ
# moves the live anchors and the coincidence asserts below go red,
# which is the mirrors asking to be updated.
M03_ROT_DEG = 0.0

# DUMP offset (owner's lever, 2026-10-06): how far the turret's DUMP
# centroid sits from the m05 hub axis, measured outward along the
# turret's own radial — the direction from DUMP through the turret hub,
# so a positive offset moves the whole m03 unit, hub included, away
# from the m05 axis (+Y at ψ = 0, DUMP at world y = +offset). It
# composes with ψ: the radial turns with the turret. 0 is the
# DUMP-over-hub placement of option a'. The top feed (a rotating flare
# on the top column, `_notes/state/m05-diverter.md` §Top deck) wants
# DUMP's exit edge over the column mouth, ~35..45 mm out. The chain's
# mirrored beliefs (`m02_chain/params.py`) assume 0, as for ψ: the
# chain is placed from m03's LOAD, so at +40 it leaves its posts by
# 40 mm and the post anchors and gantry rests go red (2026-10-06) —
# held at 0 until the owner decides how the chain follows. Moving
# it moves the turret floor across the deck: grow
# `m05_diverter/params.py:TOP_DECK_OD` until the `on_deck_*` claims
# below are green.
M03_DUMP_OFFSET = 0.0


def m03_pose(m05: Assembly, m03: Assembly) -> Location:
    """m03's world pose, derived from the two units' anchors.

    - xy: the turret's DUMP centroid sits `M03_DUMP_OFFSET` out from
      the m05 hub axis along the turret radial (option a' at 0,
      2026-05-05) — m05's `deck_top` anchor carries the hub xy.
    - z: the turret floor's bottom face rests on the m05 deck top
      face — m03's `floor_bottom` meets m05's `deck_top` plane.

    Rotated by ψ (`M03_ROT_DEG`) about the DUMP centroid, then DUMP
    translated onto its target: world = T + R(ψ)·local with
    T = hub + offset·R(ψ)·û − R(ψ)·dump, where û is the m03-local unit
    vector from DUMP toward the turret hub (one arm along the DUMP
    anchor's +Y, as `_with_service_access` finds the hub). Shared with
    `cad/render.py`, so the render cannot place m03 differently from
    the check.
    """
    deck_top = m05.anchor("deck_top").position
    dump_at = m03.anchor("dump_centroid")
    dump = dump_at.position
    floor_bottom = m03.anchor("floor_bottom").position
    hub = (dump_at * Pos(0, TURRET_ARM_R, 0)).position
    ux, uy = hub.X - dump.X, hub.Y - dump.Y
    u_len = math.hypot(ux, uy)
    ux, uy = ux / u_len, uy / u_len
    psi = math.radians(M03_ROT_DEG)
    c, s = math.cos(psi), math.sin(psi)
    dump_rx = dump.X * c - dump.Y * s
    dump_ry = dump.X * s + dump.Y * c
    out_x = M03_DUMP_OFFSET * (ux * c - uy * s)
    out_y = M03_DUMP_OFFSET * (ux * s + uy * c)
    return Pos(
        deck_top.X + out_x - dump_rx,
        deck_top.Y + out_y - dump_ry,
        deck_top.Z - floor_bottom.Z,
    ) * Rot(0, 0, M03_ROT_DEG)


# Driver access to the m02 liner's free-end screws (slice 5 of the
# 2026-09-27 completion pass): on every stage whose liner the exit
# opening cuts through, each countersunk M3 can be driven from outside
# with the machine assembled. The corridors are the stage's named
# keep-outs (`arc_fastening.driver_corridor`); held here, where every
# body they must clear exists.
ARC_SCREW_ACCESS_MARGIN = 0.5    # as the m03 clamp bolts' CLAMP_BOLT_ACCESS_MARGIN


def _with_arc_screw_access(a: Assembly) -> Assembly:
    """Hold every body in the machine `ARC_SCREW_ACCESS_MARGIN` clear of
    each split stage's driver corridors. Declared here, not in the stage
    builder: the corridor reaches past the stage, and a keep-out holds
    only the parts of the assembly that declares it.

    Three parts are excluded, the three the screw joins: the screw (the
    corridor starts on its head), the bowl sector it is driven into
    (the corridor starts in its countersink) and the liner sector it
    fastens (its boss sits right behind the skirt, 2.6 mm off the
    corridor's start). The last two are held in the stage itself, the
    bowl less the screw's seat (`arc_screw_{end}_driver_own`,
    `m02_c_channel_singulator/assembly.py`). They stay out of this
    claim so that its reading is the nearest body that is not the
    fastening: with them in, it would read the seat's own 1 mm on every
    stage and say nothing of what stands in front.
    """
    split = [s for s in STAGES if exit_splits_wall(s.wall_h)]
    assert split, "no stage's liner is split — the claims would be vacuous"
    own_liner = STATOR_PARTS[sector_index(0.0, LINER_SEAM_PHASE_DEG)]
    for stage in split:
        unit = f"m02_chain.{stage.name}"
        for sign in FREE_END_SIGNS:
            end = free_end_name(sign)
            own_bowl = bowl_part_at(arc_screw_azimuth_deg(sign), stage.bowl_seam_phase_deg)
            a = a.assert_clear_of(
                ["m05", "m03", "m02_chain"], f"{unit}.arc_screw_{end}_driver",
                name=f"{stage.name}_arc_screw_{end}_driver",
                excluding=[f"{unit}.arc_screw_{end}", f"{unit}.drive.{own_bowl}", f"{unit}.stator.{own_liner}"],
                min_mm=ARC_SCREW_ACCESS_MARGIN,
            )
    return a


# Hand access to the IMAGE station from the machine's sides (owner
# 2026-09-18, `_notes/state/m03-scanner.md` §Open work item 0): the
# operator stands at −Y, where the hopper is loaded, but the m02 chain
# fills that side of the turret, so IMAGE is serviced from ±X — clearing
# a jam, reaching a platform. A corridor one hand wide and REACH_H tall
# runs from the IMAGE platform's side edge out to the deck edge on each
# side, just behind the LOAD / IDLE platforms and above the IMAGE
# platform top. The probe is a box, not a hand.
REACH_W = 100.0    # across the knuckles ~90, plus a little
REACH_H = 130.0
REACH_GAP = 5.0    # off the LOAD / IDLE platforms' +Y edge and the IMAGE top


def _with_service_access(a: Assembly) -> Assembly:
    """Hold every unit clear of a side-reach corridor to IMAGE on each
    side. Declared here, not in m03: the corridor's length runs to m05's
    deck edge and it must clear the chain, so only the machine knows its
    geometry and has every body it must clear.

    The corridor is placed from the rest pose — the turret is jointed
    and this level drives no motion. A platform mid-index sweeps through
    it; nobody reaches in then, so a parent that drives the turret must
    scope this claim to the dwell.
    """
    # Turret hub in world: one arm back (+Y) from the DUMP centroid, as
    # in `check_turret_envelope.py`. LOAD / IDLE sit on the hub's X
    # line, so their outer +Y edge is half a platform past the hub.
    hub = (a.anchor("m03.dump_centroid") * Pos(0, TURRET_ARM_R, 0)).position
    y0 = hub.Y + PLATFORM_ACRYLIC_SIZE / 2.0 + REACH_GAP
    image_top_z = max(
        (pp.location * pp.part).bounding_box().max.Z
        for pp in a.placed_parts
        if pp.name.startswith("m03.turret.rotor.platform_image.")
    )
    deck = a.part("m05.top_acrylic")
    deck_r = (deck.location * deck.part).bounding_box().max.X
    x_in = PLATFORM_ACRYLIC_SIZE / 2.0          # IMAGE platform side edge
    length = deck_r - x_in
    z0 = image_top_z + REACH_GAP
    # Along +X from x = 0, y 0…REACH_W, z 0…REACH_H.
    corridor = Pos(length / 2.0, REACH_W / 2.0, REACH_H / 2.0) * Box(length, REACH_W, REACH_H)
    a = (
        a
        .assert_scalar("service_reach_y0", y0, detail="corridor's -Y face, world")
        .assert_scalar("service_reach_z0", z0, detail="corridor floor: IMAGE platform top + REACH_GAP")
    )
    for side, x in (("pos_x", hub.X + x_in), ("neg_x", hub.X - x_in - length)):
        a = a.assert_clear_of(
            ["m05", "m03", "m02_chain"], Pos(x, y0, z0) * corridor,
            name=f"service_reach_{side}",
        )
    return a


# What stands on the deck stays on it (owner 2026-10-06): every part
# whose bottom face rests on m05's deck top keeps its footprint
# DECK_EDGE_MARGIN inside the rim. 5 mm: the m03 frame is set on the
# deck by hand with no locating feature yet, so a few mm of placement
# slop, plus the edge break / chips of a cut acrylic edge and the
# disc's OD tolerance — a footprint closer than that is a foot that can
# end up on the chamfer or over the edge. It is a margin on *where* the
# foot lands, not a load claim: whether 10 mm acrylic carries a
# cantilevered foot near its rim is bench item 21
# (`_notes/backlog/bench-validation.md`).
DECK_EDGE_MARGIN = 5.0


def _deck_standers(a: Assembly) -> list[str]:
    """Every m03 / m02-chain part whose lowest face lies on the deck-top
    plane — read from the placed geometry, so a part that comes to
    rest on the deck is selected without anyone listing it."""
    deck_z = a.anchor("m05.deck_top").position.Z
    return sorted(
        pp.name
        for pp in a.placed_parts
        if pp.name.startswith(("m03.", "m02_chain."))
        and abs((pp.location * pp.part).bounding_box().min.Z - deck_z) < 1e-3
    )


def _with_on_deck(a: Assembly) -> Assembly:
    """Hold each deck-standing part `DECK_EDGE_MARGIN` clear of m05's
    `off_deck` keep-out — the band on the deck-top plane from the rim
    outward — one claim per part, so the margin each one leaves is its
    own diffable number. A footprint over the rim overlaps the band and
    fails whatever the margin. Declared here: the deck is m05's, what
    stands on it is m03's and the chain's.

    Selected at the rest pose: every part selected today is static (the
    turret frame, the gantry's brace feet), and this level drives no
    motion."""
    standers = _deck_standers(a)
    assert standers, "nothing stands on the deck — the claims would be vacuous"
    for path in standers:
        a = a.assert_clear_of(
            path, "m05.off_deck", name=f"on_deck_{path}", min_mm=DECK_EDGE_MARGIN,
        )
    return a


# Cached because the degenerate ``assembly`` memo below and any
# composing parent (``cad/assembly.py``, ``cad/render.py``) each ask
# for the same schema, and the build is seconds of OCCT work. Replaces
# import-time memoization: the memo line is now free, and a parent
# calling the builder gets the same object rather than rebuilding.
# Safe to share — ``Assembly`` is a frozen dataclass and every
# ``with_*`` returns a copy.
#
# Call it with the schema **explicit** everywhere, including the memo
# below: ``@cache`` keys on the call signature, so ``build_x()`` and
# ``build_x(DEFAULT_SCHEMA)`` are two entries that build identical
# geometry twice. That mistake is invisible — same output, double the
# work — so ``cache_info().misses`` is the only thing that shows it.
@cache
def build_top(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The whole machine: the three units composed at their derived
    world poses, with the cross-unit checks.

    ``schema`` threads down to each unit builder, which binds its own
    tag dicts and paints its own subtree — so this level never touches
    a palette. One process can build every schema in turn; nothing is
    captured at import.

    The unit poses derive from the units' exported anchors rather than
    mirrored constants, and those reads happen *here*, inside the
    builder, against the assemblies this call just built. They used to
    run at import against module-level objects, which silently tied the
    poses to whatever schema those objects were built under — harmless
    while there was one global schema, wrong the moment there are
    several in flight.
    """
    m05 = build_m05(schema)
    m03 = build_m03(schema)
    chain = build_chain(schema)

    m03_at = m03_pose(m05, m03)

    # m02 chain placement: the chain's `load_top` anchor must land on
    # m03's live `load_top` (the LOAD platform top centroid the chain
    # delivers onto). The chain's own mirrored beliefs are asserted
    # against the live units after composition (the anchor asserts
    # below) instead of the retired namespace cross-checks.
    load_top_world = (m03_at * m03.anchor("load_top")).position
    chain_load_local = chain.anchor("load_top").position
    chain_pose = Pos(
        load_top_world.X - chain_load_local.X,
        load_top_world.Y - chain_load_local.Y,
        load_top_world.Z - chain_load_local.Z,
    )

    a = (
        Assembly()
        .with_subassembly("m05", m05, location=_M05_POSE)
        .with_subassembly("m03", m03, location=m03_at)
        .with_subassembly("m02_chain", chain, location=chain_pose)
        # Every cross-unit pair (m03 × m05, chain × m05, chain × m03),
        # each unit addressed by its subtree path. Within-unit pairs are
        # each unit's own claims, which propagate and evaluate here
        # too. The gantry's rests on m05 are tangent (no overlap), so
        # they coexist with the group form.
        .assert_no_interference_between("m03", "m05")
        .assert_no_interference_between("m02_chain", "m05")
        .assert_no_interference_between("m02_chain", "m03")
        # Each unit exports where it believes the shared datums are —
        # m05/m03 from their live values, the chain from the constants
        # it mirrors so it can build standalone. Composed, the beliefs
        # must coincide; a drifted chain mirror fails here loudly
        # instead of silently desyncing the machine. (m03 has nothing
        # left to drift: its pose is *derived* from the live anchors
        # above.)
        .assert_anchors_coincident("m02_chain.deck_top", "m05.deck_top")
    )
    # The chain's posts are m05's columns continued through the deck:
    # where the chain needs each column to end must be where m05 ends
    # it, and the prop bar across the column's end must actually rest
    # on it (a gap passes no-interference just as well).
    for stage, k in PROP_POST.items():
        a = (
            a
            .assert_anchors_coincident(
                f"m02_chain.post_top_{k}", f"m05.outer_vert_{k}_top"
            )
            .assert_tangent_contact(
                f"m02_chain.gantry.{stage}_prop_bar", f"m05.columns.outer_vert_{k}",
                name=f"{stage}_prop_bar_on_outer_vert_{k}",
            )
        )
    # The chain's printed post seats socket over the columns' ends
    # (the chain mirrors each column's yaw — a drift shows here as a
    # seat cutting its column).
    for stage, k in PROP_POST.items():
        a = a.assert_tangent_contact(
            f"m02_chain.gantry.{stage}_post_seat", f"m05.columns.outer_vert_{k}",
            name=f"{stage}_post_seat_on_outer_vert_{k}",
        )
    # S2's stem butts V_4's face and its three brackets sit on it; the
    # brace collar rings V_4 at the node; the braces themselves stand
    # clear of the post they leave, the columns they land beside and
    # the deck — the gaps their sockets bridge, measured — and their
    # feet stand on the deck, V_5's clear of the post and of the
    # through-deck vertex bracket's arm rising beside it.
    a = (
        a
        .assert_tangent_contact(
            "m02_chain.gantry.s2_stem", "m05.columns.outer_vert_4",
            name="s2_stem_on_outer_vert_4",
        )
        .assert_tangent_contact(
            "m02_chain.gantry.brace_head", "m05.columns.outer_vert_4",
            name="brace_head_on_outer_vert_4",
        )
    )
    for name in ("top", "left", "right"):
        a = a.assert_tangent_contact(
            f"m02_chain.gantry.s2_stem_bracket_{name}", "m05.columns.outer_vert_4",
            name=f"s2_stem_bracket_{name}_on_outer_vert_4",
        )
    for col in BRACE_COLS:
        brace = f"m02_chain.gantry.brace_{col}"
        a = (
            a
            .assert_distance(brace, "m05.columns.outer_vert_4", min_mm=1.0,
                             name=f"brace_{col}_clears_post")
            .assert_distance(brace, f"m05.columns.outer_vert_{col}", min_mm=1.0,
                             name=f"brace_{col}_clears_outer_vert_{col}")
            .assert_distance(brace, "m05.top_acrylic", min_mm=1.0,
                             name=f"brace_{col}_over_deck")
            .assert_tangent_contact(f"m02_chain.gantry.brace_foot_{col}", "m05.top_acrylic",
                                    name=f"brace_foot_{col}_on_deck")
        )
    a = (
        a
        .assert_distance("m02_chain.gantry.brace_foot_5", "m05.columns.outer_vert_5", min_mm=1.0,
                         name="brace_foot_5_clears_outer_vert_5")
        .assert_distance("m02_chain.gantry.brace_foot_5", "m05.top_cap.outer_vertex_bracket_5", min_mm=1.0,
                         name="brace_foot_5_clears_vertex_bracket_5")
    )
    return _with_on_deck(_with_service_access(_with_arc_screw_access(a)))


# Degenerate memoized master, so ``khana check cad/assembly.py``
# resolves without a ``:factory``.
assembly = build_top(DEFAULT_SCHEMA)

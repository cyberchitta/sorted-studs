"""
m02 chain — top-level standalone assembly.

Composes the chain's structural skeleton — two column extensions
(`col1_ext`, `col2_ext`) rising from the m05 top deck plane, and an
S1 yoke crossbar (`yoke`) spanning between them at the S1 m02
V-bottom plane — plus the LOAD ramp (`load_ramp`) and the S1 m02
singulator as the `s1` subtree (part identity is the tree path:
`s1.rotor.rotor`, `s1.drive.stem`, ...). Coordinate frame, anchors,
and the deferred stator-mounting interface are documented in
`params.py`.

Cross-boundary checks (within the chain):

- yoke ↔ m02 rotor / motor / stem — the yoke threads underneath
  the V-bottom plane and inside the stator's inner footprint;
  every chain skeleton part must clear every m02 internal part.
- col1 / col2 ↔ m02 stator / rotor / motor / stem — the columns
  end at the V-bottom plane; tangent contact at z=YOKE_TOP_Z_LOCAL
  is allowed but no part may overlap volumetrically.
- LOAD ramp ↔ m02 stator — the ramp inlet sits in the chute-exit
  azimuth (chain -X side of S1); the ramp must NOT crash the
  outer stator wall outboard of the chute opening.

m05 column tops + top deck and m03 turret parts are EXTERNAL to the
chain — cross-unit asserts live in `cad/assembly.py`.

The m02 stator-mounting interface (handoff sub-task 2) is DEFERRED;
the stator sits at its geometric S1 position with no plate / clamp /
cradle modeled. See `params.py` module docstring.

Composition strategy:

The m02 unit is imported as a plain package member
(`..m02_c_channel_singulator.assembly`) and composed as the `s1`
subtree at its chain-local pose. The subtree keeps m02's own
color/material bindings (no re-skin — the values come from the same
shared palette machinery); m02's within-unit assertions ride along
nested and are not evaluated by the chain's `check()` — the m02
standalone `khana check` validates them.
"""

from __future__ import annotations

from functools import cache

from build123d import Box, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly

from ..design_systems import DEFAULT_SCHEMA, paint
from ..m02_c_channel_singulator.assembly import build_m02
from .design_systems import design_system
from .materials import material_of
from .params import (
    COL1_LOCAL_XY,
    DECK_TOP_Z_LOCAL,
    COL1_ROT_DEG,
    COL2_LOCAL_XY,
    COL_LEN,
    COL_Z_BOT_LOCAL,
    COL_Z_TOP_LOCAL,
    EXT_SIZE,
    M03_TURRET_ARM_R,
    RAMP_FLOOR_T,
    RAMP_OUTLET_LOCAL,
    RAMP_PITCH_DEG,
    RAMP_SLOPE_LEN,
    RAMP_WALL_H,
    RAMP_WALL_T,
    RAMP_WIDTH,
    S1_AXIS_LOCAL_XY,
    S1_M02_HEADING_DEG,
    S1_VBOT_Z_LOCAL,
    YOKE_ANGLE_DEG,
    YOKE_LEN,
    YOKE_MID_XY,
    YOKE_TOP_Z_LOCAL,
)

# --- Part builders -----------------------------------------------------

def _column_ext() -> Part:
    """20×20×COL_LEN 2020 extrusion, centered at origin, axis +Z."""
    return Box(EXT_SIZE, EXT_SIZE, COL_LEN)


def _yoke_crossbar() -> Part:
    """20×20×YOKE_LEN 2020 extrusion, centered at origin, axis +X
    in part-local. Placement rotates and translates to the chain-local
    yoke position."""
    return Box(YOKE_LEN, EXT_SIZE, EXT_SIZE)


def _load_ramp() -> Part:
    """Open chute — flat floor plate + two side walls. Built in a
    ramp-local frame with origin at the OUTLET centerline on the floor
    TOP surface, +X pointing up the slope toward the inlet, +Y across
    the ramp's width, +Z normal to the floor (out of the brick contact
    surface). Placement rotates by -RAMP_PITCH_DEG about Y (tilting
    +X up into +Z) and translates the outlet origin to chain-local
    RAMP_OUTLET_LOCAL.
    """
    # Floor — top face at z=0, body below.
    floor = Box(RAMP_SLOPE_LEN, RAMP_WIDTH, RAMP_FLOOR_T)
    floor = Pos(RAMP_SLOPE_LEN / 2.0, 0.0, -RAMP_FLOOR_T / 2.0) * floor

    # Side walls — built off the floor's +Y / -Y edges, rising in +Z.
    wall = Box(RAMP_SLOPE_LEN, RAMP_WALL_T, RAMP_WALL_H)
    wall_pos = (
        Pos(
            RAMP_SLOPE_LEN / 2.0,
            (RAMP_WIDTH - RAMP_WALL_T) / 2.0,
            RAMP_WALL_H / 2.0,
        )
        * wall
    )
    wall_neg = (
        Pos(
            RAMP_SLOPE_LEN / 2.0,
            -(RAMP_WIDTH - RAMP_WALL_T) / 2.0,
            RAMP_WALL_H / 2.0,
        )
        * wall
    )
    return floor + wall_pos + wall_neg


# --- Compose -----------------------------------------------------------

# Column placements. col1 rotated 30° about Z to match m05
# `outer_vert_0`'s orientation (one face aligned with the bracket
# plate convention). col2 rotated to the yoke angle so its faces
# align with the yoke crossbar end.
_COL1_LOC = (
    Pos(COL1_LOCAL_XY[0], COL1_LOCAL_XY[1], (COL_Z_BOT_LOCAL + COL_Z_TOP_LOCAL) / 2.0)
    * Rot(0, 0, COL1_ROT_DEG)
)
_COL2_LOC = (
    Pos(COL2_LOCAL_XY[0], COL2_LOCAL_XY[1], (COL_Z_BOT_LOCAL + COL_Z_TOP_LOCAL) / 2.0)
    * Rot(0, 0, YOKE_ANGLE_DEG)
)

# Yoke placement. Box's long axis is +X (length YOKE_LEN). Rotate by
# YOKE_ANGLE_DEG about Z, translate to YOKE_MID_XY at z =
# YOKE_TOP_Z_LOCAL - EXT_SIZE/2 (top face at YOKE_TOP_Z_LOCAL).
_YOKE_LOC = (
    Pos(YOKE_MID_XY[0], YOKE_MID_XY[1], YOKE_TOP_Z_LOCAL - EXT_SIZE / 2.0)
    * Rot(0, 0, YOKE_ANGLE_DEG)
)

# LOAD ramp placement. Tilt about ramp-local Y by -RAMP_PITCH_DEG so
# +X tilts up into +Z, then translate to chain-local outlet.
_RAMP_LOC = (
    Pos(*RAMP_OUTLET_LOCAL)
    * Rot(0, -RAMP_PITCH_DEG, 0)
)

# S1 m02 pose. Translate to S1 axis at the V-bottom plane, then
# rotate by S1_M02_HEADING_DEG about Z so the chute fires in chain-local
# -X (toward LOAD).
_S1_POSE = (
    Pos(S1_AXIS_LOCAL_XY[0], S1_AXIS_LOCAL_XY[1], S1_VBOT_Z_LOCAL)
    * Rot(0, 0, S1_M02_HEADING_DEG)
)


# Chain skeleton parts, asserted against m02 internals at S1 and
# against each other. m02's internal pairs (rotor↔stator, rotor↔stem,
# etc.) are covered by m02's standalone `khana check` — its assertions
# ride along nested and are not evaluated here.

_CHAIN_SKELETON = ("col1_ext", "col2_ext", "yoke", "load_ramp")


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
def build_chain(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The m02 chain — singulator at S1 plus the skeleton that carries
    it — painted under ``schema``.

    Painted with ``skip=("s1",)``: the singulator subtree arrives from
    ``build_m02(schema)`` already painted against m02's own tag dict,
    and the chain's dict covers only its four skeleton parts. Without
    the skip this walk would look m02's part names up here and report
    every one as untagged. (That split is why the coverage check this
    replaces had a hand-written ``startswith("s1.")`` filter.)

    The anchors are chain-local *belief* anchors. The chain builds
    standalone from mirrored m05/m03 constants (``params.py``); these
    state where those mirrors put the shared datums, so the top level
    can assert them against the live units' own anchors — a drifted
    mirror stops coinciding there and fails loudly. ``load_top`` is the
    chain-local origin by construction (the top level derives the
    chain's world pose from m03's live ``load_top``); ``deck_top`` is
    the hub-axis point on the m05 deck top face; ``col1_base`` is where
    col1's base center must meet m05's ``outer_vert_0_top``.
    """
    def mat(name: str) -> dict:
        return {"material": material_of(name)}

    return paint(
        Assembly()
        .with_part("col1_ext", _column_ext(), location=_COL1_LOC, **mat("col1_ext"))
        .with_part("col2_ext", _column_ext(), location=_COL2_LOC, **mat("col2_ext"))
        .with_part("yoke", _yoke_crossbar(), location=_YOKE_LOC, **mat("yoke"))
        .with_part("load_ramp", _load_ramp(), location=_RAMP_LOC, **mat("load_ramp"))
        .with_subassembly("s1", build_m02(schema), location=_S1_POSE)
        .with_anchor("load_top", Pos(0, 0, 0))
        .with_anchor(
            "deck_top",
            Pos(-M03_TURRET_ARM_R, -M03_TURRET_ARM_R, DECK_TOP_Z_LOCAL),
        )
        .with_anchor("col1_base", Pos(*COL1_LOCAL_XY, DECK_TOP_Z_LOCAL))
        .assert_no_interference_between(_CHAIN_SKELETON, "s1")
        .assert_no_interference_within(_CHAIN_SKELETON),
        design_system(schema),
        skip=("s1",),
    )


# Degenerate memoized master, so ``khana check <file>`` resolves
# without a ``:factory``.
assembly = build_chain(DEFAULT_SCHEMA)

"""
m02 chain — top-level standalone assembly.

Composes the three singulator stages as the `s1` / `s2` / `s3`
subtrees (part identity is the tree path: `s2.rotor.rotor`,
`s3.drive.base_plate`, …), the LOAD ramp (`load_ramp`) from S1's
chute exit down to the m03 LOAD platform, and the gantry that carries
the stages on m05's two through-deck columns: S1's prop bar
(`s1_prop_bar`, across V_5's end), S3's prop bar and cross
(`s3_prop_bar`, `s3_cross_a`, `s3_cross_b`, across V_4's end), S2's T
arm (`s2_stem` off V_4's face, `s2_tee` under S2's plate) and the two
braces from V_4 down to the deck at V_3 and V_5 (`brace_3`,
`brace_5`). Coordinate frame, the placement family, the stage table
and the gantry sizes are in `params.py`.

The posts themselves are m05's columns (`m05_diverter/params.py:
THROUGH_DECK_POSTS`), so they are not modelled here: the chain
exports where it needs their tops (`post_top_4`, `post_top_5`) and the
top-level `cad/assembly.py` asserts those coincident with m05's live
column tops and declares the rests (bar on post end, stem on post
face) and the braces' clearances — claims that span both units.

Claims within the chain:

- stage ↔ stage: the upper plate clears the lower rotor's apex by
  STAGE_CLEAR (measured), and no part of one stage touches another.
- gantry ↔ stages: each prop bar and the T bar rest tangent under
  their plate; no gantry member touches a stage's motor, bearing,
  rotor or stator (S2's and S3's motors are swung for exactly this).
- gantry ↔ gantry: the stem butts the T bar (tangent); nothing else
  touches.
- ramp ↔ S1 stator: the ramp inlet sits in the chute-exit azimuth and
  must not crash the outer stator wall; ramp pitch inside its window.
  (The outlet's clearance from the turret's loaded sweep is the
  envelope check's, below — it is why the outlet sits where it does.)

m05 column tops + top deck and m03 turret parts are EXTERNAL to the
chain — cross-unit asserts live in `cad/assembly.py`, and the
turret's loaded-platform envelope in `cad/check_turret_envelope.py`.

The stator-mounting interface (the skirt / bowl that seats the stage
on its bars) is still open: each stage's base plate is the stand-in
that rests on the bars.
"""

from __future__ import annotations

import math
from functools import cache

from build123d import Box, Cylinder, Location, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly

from ..design_systems import DEFAULT_SCHEMA, paint
from ..m02_c_channel_singulator.assembly import build_m02
from ..m02_c_channel_singulator.params import ROTOR_BASE_DIAMETER
from .design_systems import design_system
from .materials import material_of
from .params import (
    ARM_POST,
    ARM_STEM_FACE_DEG,
    ARM_STEM_LEN,
    ARM_STEM_REACH,
    ARM_TEE_LEN,
    ARM_TOP_Z_LOCAL,
    BAR_DEPTH,
    BRACE_COLS,
    BRACE_END_TRIM,
    BRACE_H_ABOVE_DECK,
    CHUTE_LAND_R,
    CROSS_HALF_LEN,
    DECK_TOP_Z_LOCAL,
    EXT_SIZE,
    M03_TURRET_ARM_R,
    M05_AXIS_LOCAL_XY,
    M05_TOP_DECK_APERTURE_D,
    M05_TOP_DECK_OD,
    M05_TOP_DECK_T,
    POST_TOP_Z_LOCAL,
    PROP_BAR_FAR,
    PROP_BAR_NEAR,
    PROP_POST,
    RAMP_AZIMUTH_DEG,
    RAMP_FLOOR_T,
    RAMP_LANDING_LEN,
    RAMP_OUTLET_LOCAL,
    RAMP_PITCH_DEG,
    RAMP_PITCH_MAX_DEG,
    RAMP_PITCH_MIN_DEG,
    RAMP_SLOPE_LEN,
    RAMP_STATOR_CLEAR,
    RAMP_WALL_H,
    RAMP_WALL_T,
    RAMP_WIDTH,
    STAGE_CLEAR,
    STAGES,
    Stage,
    m05_outer_vertex_local_xy,
)
from ..m02_c_channel_singulator.params import R_STATOR_OUTER_KINK

# Constructed-equals-bound tolerance (cad/CLAUDE.md principle 6).
_MEAS_TOL = 0.01


# --- Part builders -----------------------------------------------------

def _bar(length: float) -> Part:
    """2020 extrusion, centered at origin, long axis +X."""
    return Box(length, EXT_SIZE, EXT_SIZE)


def _chute(length: float) -> Part:
    """Open chute segment — flat floor plate + two side walls — in a
    segment-local frame: origin at the near end's centerline on the
    floor TOP surface, +X along the segment, +Y across its width, +Z
    normal to the floor (out of the brick contact surface)."""
    floor = Pos(length / 2.0, 0.0, -RAMP_FLOOR_T / 2.0) * Box(length, RAMP_WIDTH, RAMP_FLOOR_T)
    wall = Box(length, RAMP_WALL_T, RAMP_WALL_H)
    wall_pos = Pos(length / 2.0, (RAMP_WIDTH - RAMP_WALL_T) / 2.0, RAMP_WALL_H / 2.0) * wall
    wall_neg = Pos(length / 2.0, -(RAMP_WIDTH - RAMP_WALL_T) / 2.0, RAMP_WALL_H / 2.0) * wall
    return floor + wall_pos + wall_neg


def _load_ramp(s: Stage) -> Part:
    """The LOAD ramp, built **in chain-local coordinates** (placed at
    identity): the slope from the outlet up to S1's chute exit, a flat
    landing continuing RAMP_LANDING_LEN past the exit at the exit
    height so the whole exit arc has floor under it, and the stator
    cut — everything within RAMP_STATOR_CLEAR of S1's stator (kink
    radius about the axis) removed, so the stator wall is the ramp's
    inner side. The cut is a vertical cylinder in the chain frame,
    which is why the part is built in place rather than in a tilted
    ramp-local frame and placed afterwards. The outlet end is trimmed
    to the vertical plane through the outlet: the chute's side walls
    stand normal to the floor, and at a 51° pitch their top corners
    would otherwise reach 15 mm back past the lip, over the turret's
    loaded sweep (the envelope check found exactly that).
    """
    yaw = Rot(0, 0, RAMP_AZIMUTH_DEG)
    slope = Pos(*RAMP_OUTLET_LOCAL) * yaw * Rot(0, -RAMP_PITCH_DEG, 0) * _chute(RAMP_SLOPE_LEN)
    inlet = (*s.chute_exit_xy, s.chute_exit_z)
    landing = Pos(*inlet) * yaw * _chute(RAMP_LANDING_LEN)
    cut = Pos(s.axis_xy[0], s.axis_xy[1], s.vbot_z) * Cylinder(
        R_STATOR_OUTER_KINK + RAMP_STATOR_CLEAR, 4.0 * RAMP_WALL_H + 2.0 * RAMP_FLOOR_T + 200.0,
    )
    big = 4.0 * RAMP_SLOPE_LEN
    end_trim = Pos(*RAMP_OUTLET_LOCAL) * yaw * Pos(-big / 2.0, 0.0, 0.0) * Box(big, big, big)
    return (slope + landing) - cut - end_trim


# --- Placement helpers -------------------------------------------------

def _stage_pose(s: Stage) -> Location:
    """m02-local origin (rotor axis at the V-bottom) at the stage's
    axis and V-bottom, m02-local +X (the chute) turned to the heading."""
    return Pos(s.axis_xy[0], s.axis_xy[1], s.vbot_z) * Rot(0, 0, s.heading_deg)


def _horizontal_bar_pose(
    a: tuple[float, float], b: tuple[float, float], top_z: float,
) -> tuple[Location, float]:
    """Pose + length for a flat 2020 whose centreline runs a → b in
    plan with its top face at `top_z`."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    loc = Pos((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, top_z - EXT_SIZE / 2.0) * Rot(
        0, 0, math.degrees(math.atan2(dy, dx)),
    )
    return loc, length


def _prop_bar(s: Stage) -> tuple[Location, float]:
    """S's prop bar: post-side end PROP_BAR_NEAR before the axis (over
    the post's whole end face), far end PROP_BAR_FAR past it, along the
    post → axis line, top face at the plate underside."""
    post = m05_outer_vertex_local_xy(PROP_POST[s.name])
    ax = s.axis_xy
    d = math.hypot(ax[0] - post[0], ax[1] - post[1])
    u = ((ax[0] - post[0]) / d, (ax[1] - post[1]) / d)
    near = (ax[0] - PROP_BAR_NEAR * u[0], ax[1] - PROP_BAR_NEAR * u[1])
    far = (ax[0] + PROP_BAR_FAR[s.name] * u[0], ax[1] + PROP_BAR_FAR[s.name] * u[1])
    return _horizontal_bar_pose(near, far, s.plate_bottom_z)


def _cross_half(s: Stage, side: float) -> tuple[Location, float]:
    """One half of S's cross: square off the prop bar's face (side ±1),
    from EXT_SIZE/2 off the axis line out to the ring's edge."""
    post = m05_outer_vertex_local_xy(PROP_POST[s.name])
    ax = s.axis_xy
    d = math.hypot(ax[0] - post[0], ax[1] - post[1])
    u = ((ax[0] - post[0]) / d, (ax[1] - post[1]) / d)
    n = (-u[1] * side, u[0] * side)
    a = (ax[0] + (EXT_SIZE / 2.0) * n[0], ax[1] + (EXT_SIZE / 2.0) * n[1])
    b = (ax[0] + (EXT_SIZE / 2.0 + CROSS_HALF_LEN) * n[0], ax[1] + (EXT_SIZE / 2.0 + CROSS_HALF_LEN) * n[1])
    return _horizontal_bar_pose(a, b, s.plate_bottom_z)


def _stem() -> tuple[Location, float]:
    """S2's stem: from V_4's face (EXT_SIZE/2 off the post axis along
    the face normal) to the T bar's near face."""
    post = m05_outer_vertex_local_xy(ARM_POST)
    n = (math.cos(math.radians(ARM_STEM_FACE_DEG)), math.sin(math.radians(ARM_STEM_FACE_DEG)))
    a = (post[0] + (EXT_SIZE / 2.0) * n[0], post[1] + (EXT_SIZE / 2.0) * n[1])
    b = (a[0] + ARM_STEM_LEN * n[0], a[1] + ARM_STEM_LEN * n[1])
    return _horizontal_bar_pose(a, b, ARM_TOP_Z_LOCAL)


def _tee() -> tuple[Location, float]:
    """S2's T bar: centred ARM_STEM_REACH from the post axis along the
    stem, square to it."""
    post = m05_outer_vertex_local_xy(ARM_POST)
    n = (math.cos(math.radians(ARM_STEM_FACE_DEG)), math.sin(math.radians(ARM_STEM_FACE_DEG)))
    c = (post[0] + ARM_STEM_REACH * n[0], post[1] + ARM_STEM_REACH * n[1])
    t = (-n[1], n[0])
    a = (c[0] - (ARM_TEE_LEN / 2.0) * t[0], c[1] - (ARM_TEE_LEN / 2.0) * t[1])
    b = (c[0] + (ARM_TEE_LEN / 2.0) * t[0], c[1] + (ARM_TEE_LEN / 2.0) * t[1])
    return _horizontal_bar_pose(a, b, ARM_TOP_Z_LOCAL)


def _brace(col: int) -> tuple[Location, float]:
    """Diagonal from V_4 at BRACE_H_ABOVE_DECK down to the deck top at
    outer vertex `col`, trimmed BRACE_END_TRIM at both ends."""
    post = m05_outer_vertex_local_xy(ARM_POST)
    foot = m05_outer_vertex_local_xy(col)
    top = (post[0], post[1], DECK_TOP_Z_LOCAL + BRACE_H_ABOVE_DECK)
    bot = (foot[0], foot[1], DECK_TOP_Z_LOCAL)
    v = (bot[0] - top[0], bot[1] - top[1], bot[2] - top[2])
    full = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    d = (v[0] / full, v[1] / full, v[2] / full)
    length = full - 2.0 * BRACE_END_TRIM
    c = (top[0] + (full / 2.0) * d[0], top[1] + (full / 2.0) * d[1], top[2] + (full / 2.0) * d[2])
    az = math.degrees(math.atan2(d[1], d[0]))
    el = math.degrees(math.asin(d[2]))
    # Yaw about Z after pitching the bar's +X down to the elevation
    # (Ry(-el) maps +X to (cos el, 0, sin el)); its Y stays horizontal.
    return Pos(*c) * Rot(0, 0, az) * Rot(0, -el, 0), length


# --- Compose -----------------------------------------------------------

_GANTRY = (
    "s1_prop_bar", "s3_prop_bar", "s3_cross_a", "s3_cross_b",
    "s2_stem", "s2_tee", "brace_3", "brace_5",
)
_SKELETON = ("load_ramp",) + _GANTRY
_STAGE_NAMES = tuple(s.name for s in STAGES)


# Cached because the degenerate ``assembly`` memo below and any
# composing parent (``cad/assembly.py``, ``cad/render.py``) each ask
# for the same schema, and the build is seconds of OCCT work. Safe to
# share — ``Assembly`` is a frozen dataclass and every ``with_*``
# returns a copy. Call it with the schema **explicit** everywhere,
# including the memo below (``@cache`` keys on the call signature).
@cache
def build_chain(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The m02 chain — three singulator stages, the LOAD ramp and the
    gantry — painted under ``schema``.

    Painted with ``skip=`` the three stage subtrees: each arrives from
    ``build_m02(schema, …)`` already painted against m02's own tag
    dict, and the chain's dict covers only its own members.

    The anchors are chain-local *belief* anchors. The chain builds
    standalone from mirrored m05/m03 constants (``params.py``); these
    state where those mirrors put the shared datums, so the top level
    can assert them against the live units' own anchors — a drifted
    mirror stops coinciding there and fails loudly. ``load_top`` is the
    chain-local origin by construction (the top level derives the
    chain's world pose from m03's live ``load_top``); ``deck_top`` is
    the hub-axis point on the m05 deck top face; ``post_top_k`` is
    where the chain needs m05's outer column k to end.
    """
    def mat(name: str) -> dict:
        return {"material": material_of(name)}

    a = Assembly()
    for s in STAGES:
        a = a.with_subassembly(
            s.name, build_m02(schema, s.motor_azimuth_deg), location=_stage_pose(s),
        )
    s1, s2, s3 = STAGES
    a = a.with_part("load_ramp", _load_ramp(s1), **mat("load_ramp"))
    for name, (loc, length) in (
        ("s1_prop_bar", _prop_bar(s1)),
        ("s3_prop_bar", _prop_bar(s3)),
        ("s3_cross_a", _cross_half(s3, +1.0)),
        ("s3_cross_b", _cross_half(s3, -1.0)),
        ("s2_stem", _stem()),
        ("s2_tee", _tee()),
        ("brace_3", _brace(BRACE_COLS[0])),
        ("brace_5", _brace(BRACE_COLS[1])),
    ):
        a = a.with_part(name, _bar(length), location=loc, **mat(name))

    a = (
        a
        .with_anchor("load_top", Pos(0, 0, 0))
        .with_anchor("deck_top", Pos(-M03_TURRET_ARM_R, -M03_TURRET_ARM_R, DECK_TOP_Z_LOCAL))
    )
    for k, z in POST_TOP_Z_LOCAL.items():
        a = a.with_anchor(f"post_top_{k}", Pos(*m05_outer_vertex_local_xy(k), z))

    # Stage over stage: the plate clears the lower rotor's apex by
    # STAGE_CLEAR (constructed from the same constant → tolerance
    # below the bound), the chute lands its brick on the lower rotor,
    # and no part of one stage touches another.
    for up, dn in ((s2, s1), (s3, s2)):
        a = (
            a
            .assert_clearance(
                f"{up.name}.drive.base_plate", f"{dn.name}.rotor.rotor",
                min_mm=STAGE_CLEAR - _MEAS_TOL,
                name=f"{up.name}_plate_over_{dn.name}_rotor",
            )
            .assert_no_interference_between(up.name, dn.name)
        )
    a = a.assert_no_interference_between(s3.name, s1.name)
    a = a.assert_scalar(
        "chute_land_r", CHUTE_LAND_R, ge=0.0, le=ROTOR_BASE_DIAMETER / 2.0,
        detail="upper chute exit's distance from the lower axis: on the lower rotor",
    )

    # Ramp: pitch inside the window; wraps S1's stator wall at the cut
    # clearance (constructed from the same constant → tolerance below).
    a = (
        a
        .assert_scalar(
            "ramp_pitch_deg", RAMP_PITCH_DEG, ge=RAMP_PITCH_MIN_DEG, le=RAMP_PITCH_MAX_DEG,
            detail="LOAD ramp pitch; 20/55 are placeholders, not measured friction",
        )
        .assert_distance(
            "load_ramp", "s1.stator.stator", min_mm=RAMP_STATOR_CLEAR - _MEAS_TOL,
            name="ramp_wraps_s1_stator",
        )
        # The seam: the ramp's landing starts RAMP_STATOR_CLEAR outside
        # the stator's exit floor, flush with its top (CHUTE_EXIT_ABOVE_VBOT
        # = 0) — a hairline in plan, never a step up.
        .assert_distance(
            "load_ramp", "s1.stator.exit_floor",
            min_mm=RAMP_STATOR_CLEAR - _MEAS_TOL, max_mm=RAMP_STATOR_CLEAR + 0.5,
            name="ramp_meets_exit_floor",
        )
    )

    # Gantry rests: each bar tangent under its plate.
    for bar, stage in (
        ("s1_prop_bar", s1), ("s3_prop_bar", s3),
        ("s3_cross_a", s3), ("s3_cross_b", s3), ("s2_tee", s2),
    ):
        a = a.assert_tangent_contact(bar, f"{stage.name}.drive.base_plate")
    # The stem butts the T bar's near face; the cross halves butt the
    # prop bar's faces.
    a = (
        a
        .assert_tangent_contact("s2_stem", "s2_tee")
        .assert_tangent_contact("s3_cross_a", "s3_prop_bar")
        .assert_tangent_contact("s3_cross_b", "s3_prop_bar")
    )
    # The swung motors are what keeps these pairs apart — measure them.
    a = (
        a
        .assert_clearance("s2_stem", "s2.drive.motor", min_mm=5.0)
        .assert_clearance("s3_cross_a", "s3.drive.motor", min_mm=5.0)
        .assert_clearance("s3_cross_b", "s3.drive.motor", min_mm=5.0)
        .assert_clearance("s1_prop_bar", "s1.drive.motor", min_mm=5.0)
        .assert_clearance("s3_prop_bar", "s3.drive.motor", min_mm=5.0)
    )

    # Nothing of the skeleton touches a stage except at the declared
    # rests (tangent pairs carry no overlap, so the group forms hold);
    # nothing in the skeleton touches itself except at the T and cross
    # joints (same).
    for s in STAGES:
        a = a.assert_no_interference_between(_SKELETON, s.name)
    a = a.assert_no_interference_within(_SKELETON)

    return paint(a, design_system(schema), skip=_STAGE_NAMES)


# --- Standalone fixture --------------------------------------------------

def _post_stub(k: int) -> Part:
    """m05's column k above the deck, as the chain believes it: a 2020
    from the deck top to `POST_TOP_Z_LOCAL[k]`. Origin at the deck top
    on the column axis."""
    h = POST_TOP_Z_LOCAL[k] - DECK_TOP_Z_LOCAL
    return Pos(0.0, 0.0, h / 2.0) * Box(EXT_SIZE, EXT_SIZE, h)


def _deck_stub() -> Part:
    """m05's top deck as a plain disc with its central aperture — no
    post notches, no bolt holes. Origin at the deck top on the m05
    axis."""
    return Pos(0.0, 0.0, -M05_TOP_DECK_T / 2.0) * (
        Cylinder(M05_TOP_DECK_OD / 2.0, M05_TOP_DECK_T)
        - Cylinder(M05_TOP_DECK_APERTURE_D / 2.0, M05_TOP_DECK_T + 2.0)
    )


_STUBS = ("post_4", "post_5", "deck")


def build_chain_standalone(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The chain plus stand-ins for what it hangs on — m05's two
    through-deck columns above the deck (`post_4`, `post_5`) and the
    deck itself (`deck`), placed where the chain's mirrored constants
    put them. For viewing the gantry with its posts and for checking
    the chain's own beliefs are self-consistent: each prop bar and the
    stem rest tangent on their post, the braces stand off V_4 and the
    deck. The claims against m05's real columns and deck are the top
    level's (`cad/assembly.py`); nothing composes this fixture.

        khana check cad/m02_chain/assembly.py:build_chain_standalone
        khana view  cad/m02_chain/assembly.py:build_chain_standalone
    """
    a = build_chain(schema)
    for k in sorted(POST_TOP_Z_LOCAL):
        x, y = m05_outer_vertex_local_xy(k)
        a = a.with_part(f"post_{k}", _post_stub(k), location=Pos(x, y, DECK_TOP_Z_LOCAL))
    a = a.with_part(
        "deck", _deck_stub(),
        location=Pos(M05_AXIS_LOCAL_XY[0], M05_AXIS_LOCAL_XY[1], DECK_TOP_Z_LOCAL),
    )
    for stage, k in PROP_POST.items():
        a = a.assert_tangent_contact(f"{stage}_prop_bar", f"post_{k}", name=f"{stage}_prop_bar_on_post_{k}")
    a = a.assert_tangent_contact("s2_stem", f"post_{ARM_POST}", name=f"s2_stem_on_post_{ARM_POST}")
    for col in BRACE_COLS:
        a = (
            a
            .assert_distance(f"brace_{col}", f"post_{ARM_POST}", min_mm=1.0, name=f"brace_{col}_clears_post")
            .assert_distance(f"brace_{col}", "deck", min_mm=1.0, name=f"brace_{col}_clears_deck")
        )
    a = a.assert_no_interference_between(_STUBS, _SKELETON)
    for s in STAGES:
        a = a.assert_no_interference_between(_STUBS, s.name)
    return a


# Degenerate memoized master, so ``khana check <file>`` resolves
# without a ``:factory``.
assembly = build_chain(DEFAULT_SCHEMA)

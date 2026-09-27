"""
m02 chain — top-level standalone assembly.

Composes the three singulator stages as the `s1` / `s2` / `s3`
subtrees (part identity is the tree path: `s2.rotor.rotor`,
`s3.drive.bowl_0`, …), the LOAD ramp (`load_ramp`) from S1's
chute exit down to the m03 LOAD platform, and the gantry — the
`gantry` subtree (`build_gantry`), so each name below is
`gantry.<leaf>` — that carries the stages on m05's two through-deck columns: S1's prop bar
(`s1_prop_bar`, across V_5's end), S3's prop bar and cross
(`s3_prop_bar`, `s3_cross_a`, `s3_cross_b`, across V_4's end), S2's T
arm (`s2_stem` off V_4's face, `s2_tee` under S2's plate), the two
braces from V_4 down to the deck at V_3 and V_5 (`brace_3`,
`brace_5`), and the joints (2026-09-17, `joints.py`): a printed seat
on each post's end cradling its prop bar, which lies square across
the end but yawed to the column's faces (`s1_post_seat`,
`s3_post_seat`), a printed collar on V_4 socketing both braces
(`brace_head`), a printed foot block on the deck at each brace's low
end (`brace_foot_3`, `brace_foot_5`), and bought 2020 corner brackets
wherever two members meet square: the stem on V_4's face
(`s2_stem_bracket_{top,left,right}`), the stem's T (`s2_tee_bracket_*`)
and S3's cross (`s3_cross_bracket_*`). Coordinate frame, the placement
family, the stage table and the gantry sizes are in `params.py`.

The posts themselves are m05's columns (`m05_diverter/params.py:
THROUGH_DECK_POSTS`), so they are not modelled here: the chain
exports where it needs their tops (`post_top_4`, `post_top_5`) and the
top-level `cad/assembly.py` asserts those coincident with m05's live
column tops and declares the rests (bar on post end, stem on post
face, seats and collar around the columns, feet on the deck) — claims
that span both units.

Claims within the chain:

- stage ↔ stage: the upper plate clears the lower rotor's apex by
  STAGE_CLEAR (measured), and no part of one stage touches another.
- gantry ↔ stages: each prop bar and the T bar rest tangent under
  their plate; no gantry member touches a stage's motor, bearing,
  rotor or stator (S2's and S3's motors are swung for exactly this).
- gantry ↔ gantry (`build_gantry`'s own): the stem butts the T bar (tangent); each brace
  sits in its head and foot sockets (tangent: end face on the stop,
  underside on the channel floor); each seat cradles its bar; each
  bracket touches both members it joins; nothing else touches.
- ramp ↔ S1 stator: the ramp inlet sits in the chute-exit azimuth and
  must not crash the outer stator wall; ramp pitch inside its window.
  (The outlet's clearance from the turret's loaded sweep is the
  envelope check's, below — it is why the outlet sits where it does.)

m05 column tops + top deck and m03 turret parts are EXTERNAL to the
chain — cross-unit asserts live in `cad/assembly.py`, and the
turret's loaded-platform envelope in `cad/check_turret_envelope.py`.

Each stage rests on its bars by its bowl's floor (`drive.bowl_0..2`, the
floor + skirt of `m02-chain.md` §5, 2026-09-16, printed in thirds —
each bar rests on the sector(s) its ends lie under); the bolt patterns
between floor and bars, and every joint's fasteners (T-nut bolts
through the seat and socket walls, the end-tap M5 down through each
prop bar into its column, the foot bolts through the deck), are
detail-layer, not modelled.
"""

from __future__ import annotations

import math
from functools import cache

from build123d import Box, Cylinder, Keep, Location, Part, Plane, Pos, Rot

from cad_khana.mechanism.assembly import Assembly

from ..design_systems import DEFAULT_SCHEMA, paint
from ..m02_c_channel_singulator.arc_fastening import FREE_END_SIGNS, free_end_name
from ..m02_c_channel_singulator.assembly import build_m02
from ..m02_c_channel_singulator.drive.assembly import BOWL_PARTS, bowl_part_at
from ..m02_c_channel_singulator.seams import angular_gap_to_span_deg, seam_azimuths, sector_index
from ..m02_c_channel_singulator.stator.assembly import STATOR_PARTS
from ..m02_c_channel_singulator.params import ROTOR_BASE_DIAMETER
from .design_systems import design_system
from .joints import brace_foot, brace_head, corner_bracket, post_seat
from .materials import material_of
from .params import (
    ARM_POST,
    ARM_STEM_FACE_DEG,
    ARM_STEM_LEN,
    ARM_STEM_REACH,
    ARM_TEE_LEN,
    ARM_TOP_Z_LOCAL,
    BAR_DEPTH,
    BOWL_SEAM_BAR_CLEAR_MIN_DEG,
    BRACE_COLS,
    BRACE_FOOT_START_H,
    BRACE_FOOT_STOP_H,
    BRACE_HEAD_STOP_H,
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
    brace_run,
    m05_column_rot_deg,
    m05_outer_vertex_local_xy,
    prop_bar_azimuth_deg,
)
from ..m02_c_channel_singulator.params import LINER_SEAM_PHASE_DEG, PLATE_R_IN, R_STATOR_OUTER_KINK


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


def _brace(col: int) -> Part:
    """Diagonal from V_4 at BRACE_H_ABOVE_DECK down to the deck top at
    outer vertex `col`, built **in chain-local coordinates** (placed
    at identity) because its two end faces are cut by *vertical*
    planes — BRACE_HEAD_STOP_H out from V_4's axis and
    BRACE_FOOT_STOP_H out from V_col's — so each bears full-face on
    the vertical stop in its printed socket (`joints.py`)."""
    post = m05_outer_vertex_local_xy(ARM_POST)
    foot = m05_outer_vertex_local_xy(col)
    az, el, run = brace_run(col)
    top = (post[0], post[1], DECK_TOP_Z_LOCAL + BRACE_H_ABOVE_DECK)
    full = math.hypot(run, BRACE_H_ABOVE_DECK)
    c, s = math.cos(math.radians(az)), math.sin(math.radians(az))
    # Yaw about Z after pitching +X down by the descent (Ry(el) maps
    # +X to (cos el, 0, -sin el)); the bar's Y stays horizontal. Over-
    # long by a bar each end, then cut to the two stops.
    prism = (
        Pos(*top) * Rot(0, 0, az) * Rot(0, el, 0)
        * Pos(full / 2.0, 0.0, 0.0) * Box(full + 2.0 * EXT_SIZE, EXT_SIZE, EXT_SIZE)
    )
    head_stop = Plane(
        origin=(top[0] + BRACE_HEAD_STOP_H * c, top[1] + BRACE_HEAD_STOP_H * s, 0.0),
        z_dir=(c, s, 0.0),
    )
    foot_stop = Plane(
        origin=(foot[0] - BRACE_FOOT_STOP_H * c, foot[1] - BRACE_FOOT_STOP_H * s, 0.0),
        z_dir=(-c, -s, 0.0),
    )
    return prism.split(head_stop, keep=Keep.TOP).split(foot_stop, keep=Keep.TOP)


# --- Joint placements ----------------------------------------------------

def _seat_pose(s: Stage) -> Location:
    """The seat's origin is the column's end face on its axis."""
    k = PROP_POST[s.name]
    x, y = m05_outer_vertex_local_xy(k)
    return Pos(x, y, POST_TOP_Z_LOCAL[k])


def _head_pose() -> Location:
    x, y = m05_outer_vertex_local_xy(ARM_POST)
    return Pos(x, y, DECK_TOP_Z_LOCAL + BRACE_H_ABOVE_DECK)


def _foot_pose(col: int) -> Location:
    x, y = m05_outer_vertex_local_xy(col)
    return Pos(x, y, DECK_TOP_Z_LOCAL)


def _bracket_pose(
    corner: tuple[float, float, float],
    leg_a: tuple[float, float, float],
    leg_b: tuple[float, float, float],
) -> Location:
    """A corner bracket with its corner line through `corner`, leg A
    along `leg_a` (on the face whose normal is `leg_b`) and leg B
    along `leg_b`."""
    return Plane(origin=corner, x_dir=leg_a, z_dir=leg_b).location


def _bracket_poses(s2: Stage, s3: Stage) -> dict[str, Location]:
    """Every corner bracket, keyed by part name: three where the stem
    meets V_4's face (top, and both sides — the bottom corner is the
    brace collar's), two in the stem's T, four at S3's cross."""
    post = m05_outer_vertex_local_xy(ARM_POST)
    f = math.radians(ARM_STEM_FACE_DEG)
    n = (math.cos(f), math.sin(f))          # V_4's face normal = the stem
    t = (-n[1], n[0])                        # along the face
    face = (post[0] + (EXT_SIZE / 2.0) * n[0], post[1] + (EXT_SIZE / 2.0) * n[1])
    tee_near = (post[0] + (EXT_SIZE / 2.0 + ARM_STEM_LEN) * n[0],
                post[1] + (EXT_SIZE / 2.0 + ARM_STEM_LEN) * n[1])
    poses = {
        "s2_stem_bracket_top": _bracket_pose(
            (face[0], face[1], ARM_TOP_Z_LOCAL), (n[0], n[1], 0.0), (0.0, 0.0, 1.0),
        ),
    }
    for side, name in ((+1.0, "left"), (-1.0, "right")):
        poses[f"s2_stem_bracket_{name}"] = _bracket_pose(
            (face[0] + side * (EXT_SIZE / 2.0) * t[0], face[1] + side * (EXT_SIZE / 2.0) * t[1],
             ARM_TOP_Z_LOCAL - EXT_SIZE / 2.0),
            (side * t[0], side * t[1], 0.0), (n[0], n[1], 0.0),
        )
        poses[f"s2_tee_bracket_{name}"] = _bracket_pose(
            (tee_near[0] + side * (EXT_SIZE / 2.0) * t[0], tee_near[1] + side * (EXT_SIZE / 2.0) * t[1],
             ARM_TOP_Z_LOCAL - EXT_SIZE / 2.0),
            (side * t[0], side * t[1], 0.0), (-n[0], -n[1], 0.0),
        )
    # S3's cross: each half butts the prop bar's face EXT_SIZE/2 off
    # the axis line; brackets in both inside corners of each junction.
    post3 = m05_outer_vertex_local_xy(PROP_POST[s3.name])
    ax = s3.axis_xy
    d = math.hypot(ax[0] - post3[0], ax[1] - post3[1])
    u = ((ax[0] - post3[0]) / d, (ax[1] - post3[1]) / d)
    for side, half in ((+1.0, "a"), (-1.0, "b")):
        nrm = (-u[1] * side, u[0] * side)
        for along, tag in ((+1.0, "1"), (-1.0, "2")):
            poses[f"s3_cross_bracket_{half}{tag}"] = _bracket_pose(
                (ax[0] + (EXT_SIZE / 2.0) * (nrm[0] + along * u[0]),
                 ax[1] + (EXT_SIZE / 2.0) * (nrm[1] + along * u[1]),
                 s3.plate_bottom_z - EXT_SIZE / 2.0),
                (along * u[0], along * u[1], 0.0), (nrm[0], nrm[1], 0.0),
            )
    return poses


# --- Compose -----------------------------------------------------------

# The bars that rest under a stage's floor.
_RESTS = (
    ("s1_prop_bar", "s1"), ("s3_prop_bar", "s3"),
    ("s3_cross_a", "s3"), ("s3_cross_b", "s3"), ("s2_tee", "s2"),
)


def _stage_local_az(s: Stage, xy: tuple[float, float]) -> float:
    """m02-local azimuth of chain-local point ``xy`` about stage s."""
    return math.degrees(math.atan2(xy[1] - s.axis_xy[1], xy[0] - s.axis_xy[0])) - s.heading_deg


def _bar_floor_spans(bar: str, s: Stage) -> list[tuple[float, float]]:
    """For each end of the bar that lies under s's floor (outside the
    floor's hole): the m02-local azimuths of (that end, the point where
    the centreline runs back into the hole — or the other end, if it
    never does). A bar through the axis sweeps no azimuth, so the two
    are equal; one standing beside it (S2's T, 9 mm off) sweeps a few
    degrees, and a seam must stay out of the whole sweep, not just off
    the end."""
    loc, length = {
        "s1_prop_bar": lambda: _prop_bar(s),
        "s3_prop_bar": lambda: _prop_bar(s),
        "s3_cross_a": lambda: _cross_half(s, +1.0),
        "s3_cross_b": lambda: _cross_half(s, -1.0),
        "s2_tee": _tee,
    }[bar]()
    p = loc.position
    yaw = math.radians(loc.orientation.Z)
    u = (math.cos(yaw), math.sin(yaw))
    cx, cy = p.X - s.axis_xy[0], p.Y - s.axis_xy[1]
    along, off = cx * u[0] + cy * u[1], -cx * u[1] + cy * u[0]      # the axis is at t = -along on the centreline
    half_chord = math.sqrt(max(PLATE_R_IN**2 - off**2, 0.0))

    def at(t: float) -> tuple[float, float]:
        return (p.X + t * u[0], p.Y + t * u[1])

    out = []
    for sign in (-1.0, 1.0):
        t_end = sign * length / 2.0
        end = at(t_end)
        if math.hypot(end[0] - s.axis_xy[0], end[1] - s.axis_xy[1]) <= PLATE_R_IN:
            continue
        t_hole = -along + sign * half_chord                         # the hole's edge on this end's side
        t_in = t_hole if sign * (t_end - t_hole) > 0.0 and abs(off) < PLATE_R_IN else -t_end
        t_in = max(min(t_in, length / 2.0), -length / 2.0)
        out.append((_stage_local_az(s, end), _stage_local_az(s, at(t_in))))
    return out


def _s1_exit_liner() -> str:
    """The liner sector the LOAD ramp wraps: the one spanning S1's exit."""
    return STATOR_PARTS[sector_index(0.0, LINER_SEAM_PHASE_DEG)]
_STAGE_NAMES = tuple(s.name for s in STAGES)


# Cached because the degenerate ``assembly`` memo below and any
# composing parent (``cad/assembly.py``, ``cad/render.py``) each ask
# for the same schema, and the build is seconds of OCCT work. Safe to
# share — ``Assembly`` is a frozen dataclass and every ``with_*``
# returns a copy. Call it with the schema **explicit** everywhere,
# including the memo below (``@cache`` keys on the call signature).
@cache
def build_gantry() -> Assembly:
    """The gantry — prop bars, S3's cross, S2's stem and T, the two
    braces, and the joints between them — in the chain-local frame,
    placed at identity. Its own claims are the joints: each seat
    cradles its bar, each brace sits in its head and foot sockets, each
    bracket touches both members it joins, and nothing touches itself
    anywhere else. What it rests on is the chain's (the stages) and
    the top level's (m05's columns and deck). Unpainted: the chain
    paints the whole tree, and its tag dict carries these leaves.
    """
    def mat(name: str) -> dict:
        return {"material": material_of(name)}

    s1, s2, s3 = STAGES
    a = Assembly()
    for name, (loc, length) in (
        ("s1_prop_bar", _prop_bar(s1)),
        ("s3_prop_bar", _prop_bar(s3)),
        ("s3_cross_a", _cross_half(s3, +1.0)),
        ("s3_cross_b", _cross_half(s3, -1.0)),
        ("s2_stem", _stem()),
        ("s2_tee", _tee()),
    ):
        a = a.with_part(name, _bar(length), location=loc, **mat(name))
    for col in BRACE_COLS:
        a = a.with_part(f"brace_{col}", _brace(col), **mat(f"brace_{col}"))

    # Joints. Seats and the brace sockets are printed; the brackets
    # are bought 2020 corner brackets.
    for s in (s1, s3):
        a = a.with_part(
            f"{s.name}_post_seat",
            post_seat(prop_bar_azimuth_deg(s), m05_column_rot_deg(PROP_POST[s.name])),
            location=_seat_pose(s), **mat(f"{s.name}_post_seat"),
        )
    a = a.with_part(
        "brace_head",
        brace_head(
            tuple(brace_run(col)[:2] for col in BRACE_COLS),
            BRACE_HEAD_STOP_H, m05_column_rot_deg(ARM_POST),
        ),
        location=_head_pose(), **mat("brace_head"),
    )
    for col in BRACE_COLS:
        az, el, _ = brace_run(col)
        a = a.with_part(
            f"brace_foot_{col}",
            brace_foot(az + 180.0, el, BRACE_FOOT_STOP_H, BRACE_FOOT_START_H[col], flange=(col == 3)),
            location=_foot_pose(col), **mat(f"brace_foot_{col}"),
        )
    for name, loc in _bracket_poses(s2, s3).items():
        a = a.with_part(name, corner_bracket(), location=loc, **mat(name))

    # The stem butts the T bar's near face; the cross halves butt the
    # prop bar's faces.
    a = (
        a
        .assert_tangent_contact("s2_stem", "s2_tee")
        .assert_tangent_contact("s3_cross_a", "s3_prop_bar")
        .assert_tangent_contact("s3_cross_b", "s3_prop_bar")
    )
    # Joints. Each seat cradles its bar; each brace sits in its head
    # and foot sockets; each bracket touches both members it joins.
    # The collar clears the stem above it (280 + the brace's
    # half-diagonal + the wall against the stem's underside at 305 —
    # measured, not constructed).
    for s in (s1, s3):
        a = a.assert_tangent_contact(f"{s.name}_post_seat", f"{s.name}_prop_bar")
    for col in BRACE_COLS:
        a = (
            a
            .assert_tangent_contact("brace_head", f"brace_{col}")
            .assert_tangent_contact(f"brace_foot_{col}", f"brace_{col}")
        )
    a = a.assert_distance("brace_head", "s2_stem", min_mm=5.0, name="brace_head_under_stem")
    for name in ("top", "left", "right"):
        a = a.assert_tangent_contact(f"s2_stem_bracket_{name}", "s2_stem")
        if name != "top":
            a = (
                a
                .assert_tangent_contact(f"s2_tee_bracket_{name}", "s2_stem")
                .assert_tangent_contact(f"s2_tee_bracket_{name}", "s2_tee")
            )
    for half in ("a", "b"):
        for tag in ("1", "2"):
            a = (
                a
                .assert_tangent_contact(f"s3_cross_bracket_{half}{tag}", "s3_prop_bar")
                .assert_tangent_contact(f"s3_cross_bracket_{half}{tag}", f"s3_cross_{half}")
            )
    # Nothing in the gantry touches itself except at the T, cross and
    # socket joints (tangent pairs carry no overlap, so the group form
    # holds).
    return a.assert_no_interference_within([p.name for p in a.placed_parts])


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
            s.name, build_m02(schema, s.motor_azimuth_deg, s.bowl_seam_phase_deg, s.wall_h),
            location=_stage_pose(s),
        )
    s1, s2, s3 = STAGES
    a = (
        a
        .with_part("load_ramp", _load_ramp(s1), **mat("load_ramp"))
        .with_subassembly("gantry", build_gantry())
    )

    a = (
        a
        .with_anchor("load_top", Pos(0, 0, 0))
        .with_anchor("deck_top", Pos(-M03_TURRET_ARM_R, -M03_TURRET_ARM_R, DECK_TOP_Z_LOCAL))
    )
    for k, z in POST_TOP_Z_LOCAL.items():
        a = a.with_anchor(f"post_top_{k}", Pos(*m05_outer_vertex_local_xy(k), z))

    # Stage over stage: the plate clears the lower rotor's apex by
    # STAGE_CLEAR (constructed from the same constant as the bound),
    # the chute lands its brick on the lower rotor, and no part of one
    # stage touches another.
    for up, dn in ((s2, s1), (s3, s2)):
        for part in BOWL_PARTS:
            a = a.assert_distance(
                f"{up.name}.drive.{part}", f"{dn.name}.rotor.rotor",
                min_mm=STAGE_CLEAR,
                name=f"{up.name}_{part}_over_{dn.name}_rotor",
            )
        a = a.assert_no_interference_between(up.name, dn.name)
    a = a.assert_no_interference_between(s3.name, s1.name)
    a = a.assert_scalar(
        "chute_land_r", CHUTE_LAND_R, ge=0.0, le=ROTOR_BASE_DIAMETER / 2.0,
        detail="upper chute exit's distance from the lower axis: on the lower rotor",
    )

    # Ramp: pitch inside the window; wraps S1's stator wall at the cut
    # clearance (constructed from the same constant as the bound).
    a = (
        a
        .assert_scalar(
            "ramp_pitch_deg", RAMP_PITCH_DEG, ge=RAMP_PITCH_MIN_DEG, le=RAMP_PITCH_MAX_DEG,
            detail="LOAD ramp pitch; 20/55 are placeholders, not measured friction",
        )
        .assert_distance(
            "load_ramp", f"s1.stator.{_s1_exit_liner()}", min_mm=RAMP_STATOR_CLEAR,
            name="ramp_wraps_s1_stator",
        )
        # The seam: the ramp's landing starts RAMP_STATOR_CLEAR outside
        # the stator's exit floor, flush with its top (CHUTE_EXIT_ABOVE_VBOT
        # = 0) — a hairline in plan, never a step up.
        .assert_distance(
            "load_ramp", "s1.stator.exit_floor",
            min_mm=RAMP_STATOR_CLEAR, max_mm=RAMP_STATOR_CLEAR + 0.5,
            name="ramp_meets_exit_floor",
        )
    )
    # S1's liner is split, so its two free-end screws sit in the skirt
    # just past each side of the exit, the ramp's side: their heads are
    # sunk inside the skirt's outer face, and the ramp keeps its cut
    # clearance from them as from the wall.
    for sign in FREE_END_SIGNS:
        a = a.assert_distance(
            "load_ramp", f"s1.arc_screw_{free_end_name(sign)}", min_mm=RAMP_STATOR_CLEAR,
        )

    # Gantry rests: each bar tangent under the bowl sector(s) its ends
    # lie under — and no bowl seam lands on, or under, a bar. The seams
    # are radial; a bar through the axis meets the floor along one
    # azimuth per end, a bar beside the axis (S2's T) sweeps a few
    # degrees between its end and the floor's hole, and the seam must
    # clear that whole sweep.
    spans_by_stage: dict[str, list[tuple[float, float]]] = {s.name: [] for s in STAGES}
    by_name = {s.name: s for s in STAGES}
    for bar, stage_name in _RESTS:
        stage = by_name[stage_name]
        for az, az_in in _bar_floor_spans(bar, stage):
            spans_by_stage[stage.name].append((az, az_in))
            a = a.assert_tangent_contact(
                f"gantry.{bar}", f"{stage.name}.drive.{bowl_part_at(az, stage.bowl_seam_phase_deg)}",
            )
    for s in STAGES:
        clear = min(
            angular_gap_to_span_deg(seam, lo, hi) - math.degrees((EXT_SIZE / 2.0) / R_STATOR_OUTER_KINK)
            for seam in seam_azimuths(s.bowl_seam_phase_deg)
            for lo, hi in spans_by_stage[s.name]
        )
        a = a.assert_scalar(
            f"{s.name}_bowl_seams_clear_of_bar_ends_deg", clear, ge=BOWL_SEAM_BAR_CLEAR_MIN_DEG,
            detail="closest bowl seam to the side of a gantry bar's run under the floor, degrees at the rim",
        )
    # The swung motors are what keeps these pairs apart — measure them.
    a = (
        a
        .assert_distance("gantry.s2_stem", "s2.drive.motor", min_mm=5.0)
        .assert_distance("gantry.s3_cross_a", "s3.drive.motor", min_mm=5.0)
        .assert_distance("gantry.s3_cross_b", "s3.drive.motor", min_mm=5.0)
        .assert_distance("gantry.s1_prop_bar", "s1.drive.motor", min_mm=5.0)
        .assert_distance("gantry.s3_prop_bar", "s3.drive.motor", min_mm=5.0)
    )

    # Each seat stands under the bowl's floor (its top is the plate
    # underside — the saddle) and keeps the swung motor's 5 mm. The
    # seat-in-gantry joints are `build_gantry`'s.
    for s in (s1, s3):
        seat = f"gantry.{s.name}_post_seat"
        a = (
            a
            .assert_tangent_contact(
                seat,
                f"{s.name}.drive.{bowl_part_at(_stage_local_az(s, m05_outer_vertex_local_xy(PROP_POST[s.name])), s.bowl_seam_phase_deg)}",
            )
            .assert_distance(seat, f"{s.name}.drive.motor", min_mm=5.0)
        )

    # Nothing of the ramp or the gantry touches a stage except at the
    # declared rests (tangent pairs carry no overlap, so the group
    # forms hold), and the ramp touches nothing of the gantry.
    for s in STAGES:
        a = (
            a
            .assert_no_interference_between("gantry", s.name)
            .assert_no_interference_between(["load_ramp"], s.name)
        )
    a = a.assert_no_interference_between(["load_ramp"], "gantry")

    return paint(a, design_system(schema), skip=_STAGE_NAMES)


# --- Standalone fixture --------------------------------------------------

def _post_stub(k: int) -> Part:
    """m05's column k above the deck, as the chain believes it: a 2020
    from the deck top to `POST_TOP_Z_LOCAL[k]`. Origin at the deck top
    on the column axis; the fixture yaws it to the column's real
    face orientation (`m05_column_rot_deg`)."""
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
    stem rest tangent on their post, the seats, the collar and the
    stem's brackets sit on the posts, the braces stand off V_4 and the
    deck while their feet stand on it. The claims against m05's real columns and deck are the top
    level's (`cad/assembly.py`); nothing composes this fixture.

        khana check cad/m02_chain/assembly.py:build_chain_standalone
        khana view  cad/m02_chain/assembly.py:build_chain_standalone
    """
    a = build_chain(schema)
    for k in sorted(POST_TOP_Z_LOCAL):
        x, y = m05_outer_vertex_local_xy(k)
        a = a.with_part(
            f"post_{k}", _post_stub(k),
            location=Pos(x, y, DECK_TOP_Z_LOCAL) * Rot(0, 0, m05_column_rot_deg(k)),
        )
    a = a.with_part(
        "deck", _deck_stub(),
        location=Pos(M05_AXIS_LOCAL_XY[0], M05_AXIS_LOCAL_XY[1], DECK_TOP_Z_LOCAL),
    )
    for stage, k in PROP_POST.items():
        a = (
            a
            .assert_tangent_contact(f"gantry.{stage}_prop_bar", f"post_{k}", name=f"{stage}_prop_bar_on_post_{k}")
            .assert_tangent_contact(f"gantry.{stage}_post_seat", f"post_{k}", name=f"{stage}_post_seat_on_post_{k}")
        )
    a = (
        a
        .assert_tangent_contact("gantry.s2_stem", f"post_{ARM_POST}", name=f"s2_stem_on_post_{ARM_POST}")
        .assert_tangent_contact("gantry.brace_head", f"post_{ARM_POST}", name=f"brace_head_on_post_{ARM_POST}")
    )
    for name in ("top", "left", "right"):
        a = a.assert_tangent_contact(
            f"gantry.s2_stem_bracket_{name}", f"post_{ARM_POST}", name=f"s2_stem_bracket_{name}_on_post_{ARM_POST}",
        )
    for col in BRACE_COLS:
        a = (
            a
            .assert_distance(f"gantry.brace_{col}", f"post_{ARM_POST}", min_mm=1.0, name=f"brace_{col}_clears_post")
            .assert_distance(f"gantry.brace_{col}", "deck", min_mm=1.0, name=f"brace_{col}_clears_deck")
            .assert_tangent_contact(f"gantry.brace_foot_{col}", "deck", name=f"brace_foot_{col}_on_deck")
        )
    a = a.assert_distance("gantry.brace_foot_5", "post_5", min_mm=1.0, name="brace_foot_5_clears_post_5")
    a = (
        a
        .assert_no_interference_between(_STUBS, ["load_ramp"])
        .assert_no_interference_between(_STUBS, "gantry")
    )
    for s in STAGES:
        a = a.assert_no_interference_between(_STUBS, s.name)
    return a


# Degenerate memoized master, so ``khana check <file>`` resolves
# without a ``:factory``.
assembly = build_chain(DEFAULT_SCHEMA)

"""
M05 hex-tower — frame sub-assembly (one generic middle floor).

Structural 2020-aluminum cage for one floor: an outer hex ring
(apothem R_HEX_OUTER) with 6 vertical corner columns at the outer-
hex vertices and an outer-vertex bracket at each; plus a concentric
inner hex ring (apothem R_HEX_INNER) sharing the outer hex's
rotation (aligned vertex azimuths) with 6 vertical columns at the
inner-hex vertices too. Per basket boundary, two radial 2020
segments run along the boundary azimuth: an inboard segment from
the spider hub region out to the inner-hex vertex bracket, and an
outboard segment from that bracket out to the matching outer-hex
vertex bracket. The split lets the inner-hex through-column back
both segments in compression instead of forcing a single rail to
work as a bending beam under basket load.

Why the hexes are aligned: placing both hex vertex rings on the same
six azimuths — basket boundaries (k+0.5)·60° — makes every radial
rail land at an outer-hex vertex (rather than a mid-edge), where a
vertical column already lives. The radial's outer end therefore
transfers bearing load straight down through the vertical in
compression instead of bending a horizontal hex edge. Each hex edge
midpoint falls on a basket bisector (k·60°), where the basket's
outer/inner face is free to rest on the edge top in Phase 2. The
earlier 30°-rotated variant terminated radials at outer-hex edge
midpoints with no vertical backing — that load path is what this
layout replaces.

V1 scope. This file models a MIDDLE floor. Explicitly deferred:
- Diagonal braces.
- Top cap and base floor (different corner-bracket variants).

Vertical corner columns are modeled as **through-columns**: they
extend *below* the floor datum too so the (now-flipped) outer-vertex
bracket can clamp the column at z < 0 as well as above. In the real
tower each column is a single extrusion spanning all floors (see
`_notes/decisions/m05-hex-cascade.md`); this per-floor model represents that by giving
each column an extra VERT_EXT_BELOW mm below z=0. The same span
applies to the inner-hex through-columns added alongside the outer
set.

Coordinate frame:
    origin = column axis (shared with basket / ramp_mechanism)
    +X     = basket 0 bisector
    +Z     = up
    z=0    = shared floor datum. Horizontal ring extrusions sit with
             top face at z=0 (bodies below). Vertical corner columns
             pass THROUGH z=0 continuously (no bottom face at the
             datum). Bucket floor and bearing/spider base also sit
             at z=0.

Vertex azimuths: `v_angle_k = (k + 0.5) * 60°`, shared by outer and
inner hex. Edge k runs between V_k and V_{k+1}; its midpoint is at
`e_angle_k = (k + 1) * 60°` for both hexes. Radial k runs along
`v_angle_k` (= basket-k/basket-(k+1) boundary).

Bracket orientation. Both `outer_vertex_bracket` and
`inner_vertex_bracket` place their local +X pointing toward the hex
center (INWARD). Each is positioned at its vertex with world
rotation `v_angle_k + 180°` so local +X maps to inward. The outer
bracket's radial arm then runs from the vertex inward (along local
+X) toward the inner hex; the inner bracket's radial arm runs from
the inner vertex outward (along local -X) toward the outer hex.
"""

import math
import sys
from pathlib import Path
from typing import Callable

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build123d import Box, Color, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

from inner_vertex_bracket.assembly import bracket as inner_bracket
from outer_vertex_bracket.assembly import (
    BRACKET_Z_TOP,
    EXT_SIZE,
    HOR_INNER_OFFSET,
    PLATE_T as OUTER_BRACKET_PLATE_T,
    bracket as outer_bracket,
)
from params import (
    BRACKET_HOR_OFFSET,
    FLOOR_H,
    RAIL_INNER_OUTER_END_R,
    RAIL_INNER_R,
    RAIL_OUTER_INNER_END_R,
    RAIL_OUTER_R,
    RAIL_SIZE,
    R_HEX_INNER,
    R_HEX_OUTER,
    R_VERT_INNER,
    R_VERT_OUTER,
)


# --- Derived geometry --------------------------------------------------

# Regular hexagon: edge length equals circumradius.
EDGE_LEN_OUTER = R_VERT_OUTER
EDGE_LEN_INNER = R_VERT_INNER

# Hex-edge extrusion lengths: each end reserves the bracket's arm
# offset so the extrusion butts neatly into the vertex bracket.
OUTER_HORIZ_LEN = EDGE_LEN_OUTER - 2.0 * HOR_INNER_OFFSET
INNER_HORIZ_LEN = EDGE_LEN_INNER - 2.0 * HOR_INNER_OFFSET

# Radial-rail spans. Each radial is split at the inner-hex vertex
# (see params.py docstring); the inner segment runs from the spider
# saddle out to the inner-vertex bracket's inboard arm, the outer
# segment from the inboard arm out to the outer-vertex bracket.
RADIAL_INNER_LEN = RAIL_INNER_OUTER_END_R - RAIL_INNER_R          # ≈ 49.66
RADIAL_OUTER_LEN = RAIL_OUTER_R - RAIL_OUTER_INNER_END_R          # ≈ 106.11

assert OUTER_HORIZ_LEN > 0, (
    f"OUTER_HORIZ_LEN = {OUTER_HORIZ_LEN:.1f} mm ≤ 0 — increase R_HEX_OUTER "
    f"or decrease HOR_INNER_OFFSET."
)
assert INNER_HORIZ_LEN > 0, (
    f"INNER_HORIZ_LEN = {INNER_HORIZ_LEN:.1f} mm ≤ 0 — increase R_HEX_INNER "
    f"or decrease HOR_INNER_OFFSET."
)
assert RAIL_SIZE == EXT_SIZE, (
    f"RAIL_SIZE ({RAIL_SIZE}) must equal frame EXT_SIZE ({EXT_SIZE}); "
    f"the radial rail is a standard 2020 and reuses the frame helper."
)
assert RADIAL_INNER_LEN > 0, (
    f"RADIAL_INNER_LEN = {RADIAL_INNER_LEN:.1f} mm ≤ 0 — "
    f"RAIL_INNER_OUTER_END_R must exceed RAIL_INNER_R."
)
assert RADIAL_OUTER_LEN > 0, (
    f"RADIAL_OUTER_LEN = {RADIAL_OUTER_LEN:.1f} mm ≤ 0 — "
    f"RAIL_OUTER_R must exceed RAIL_OUTER_INNER_END_R."
)
assert BRACKET_HOR_OFFSET == HOR_INNER_OFFSET, (
    f"params.BRACKET_HOR_OFFSET ({BRACKET_HOR_OFFSET}) must equal bracket "
    f"HOR_INNER_OFFSET ({HOR_INNER_OFFSET}); params.py copies this value "
    f"intentionally and this assertion catches drift."
)


# --- Part helpers -------------------------------------------------------

def _horiz_ext(length: float) -> Part:
    """2020 horizontal extrusion along local +X, top face at z=0."""
    return Pos(0, 0, -EXT_SIZE / 2.0) * Box(length, EXT_SIZE, EXT_SIZE)


# Through-column reach below the floor datum: enough to cover the
# outer-vertex bracket's below-floor body (hub bottom at
# BRACKET_Z_TOP - PLATE_T = -26) with a small clearance. In the real
# tower each column is one continuous 2020 spanning all floors; in
# the single-floor wrapper, `add_through_columns` defaults to a span
# of [-VERT_EXT_BELOW, FLOOR_H] for the visible per-floor slice.
VERT_EXT_BELOW = -(BRACKET_Z_TOP - OUTER_BRACKET_PLATE_T) + 10.0  # 36


# --- Assembly builders --------------------------------------------------

def add_floor_horizontals(
    a: Assembly,
    z: float = 0.0,
    prefix: str = "",
    color_fn: Callable[[str], Color | None] | None = None,
    material_fn: Callable[[str], str | None] | None = None,
) -> Assembly:
    """Add one floor's worth of horizontal frame members + brackets at
    the given world z (= rail-top datum for that floor).

    Adds: outer hex (`{prefix}outer_horiz_k`), inner hex
    (`{prefix}inner_horiz_k`), radial rails split at the inner-hex
    vertex (`{prefix}radial_inner_k` / `{prefix}radial_outer_k`),
    outer-vertex brackets (`{prefix}outer_vertex_bracket_k`), and
    inner-vertex brackets (`{prefix}inner_vertex_bracket_k`).

    Through-columns are NOT added here — they span all floors and live
    in `add_through_columns`.

    `color_fn` / `material_fn` are optional callables mapping each full
    instance name (with prefix) to a `Color` / material string. Default
    `None` keeps the standalone behavior (no color/material set);
    multi-floor consumers thread `color_of` / `material_of` from the
    module-level `design_systems.py`.
    """
    def _co(name: str) -> Color | None:
        return color_fn(name) if color_fn is not None else None

    def _mt(name: str) -> str | None:
        return material_fn(name) if material_fn is not None else None

    z_loc = Pos(0, 0, z)
    for k in range(6):
        # Shared vertex azimuth for outer and inner hex.
        v_angle = (k + 0.5) * 60.0
        vx_out = R_VERT_OUTER * math.cos(math.radians(v_angle))
        vy_out = R_VERT_OUTER * math.sin(math.radians(v_angle))
        vx_in = R_VERT_INNER * math.cos(math.radians(v_angle))
        vy_in = R_VERT_INNER * math.sin(math.radians(v_angle))

        # Outer-vertex bracket: local +X points INWARD toward hex center.
        n_ovb = f"{prefix}outer_vertex_bracket_{k}"
        a = a.with_part(
            n_ovb,
            outer_bracket(),
            location=z_loc * Pos(vx_out, vy_out, 0.0) * Rot(0, 0, v_angle + 180.0),
            color=_co(n_ovb),
            material=_mt(n_ovb),
        )

        # Inner-vertex bracket: same orientation convention as outer.
        n_ivb = f"{prefix}inner_vertex_bracket_{k}"
        a = a.with_part(
            n_ivb,
            inner_bracket(),
            location=z_loc * Pos(vx_in, vy_in, 0.0) * Rot(0, 0, v_angle + 180.0),
            color=_co(n_ivb),
            material=_mt(n_ivb),
        )

        # Outer horizontal on edge k (between V_k and V_{k+1}).
        # Edge midpoint at apothem angle (k + 1) * 60°.
        oe_angle = (k + 1) * 60.0
        oex = R_HEX_OUTER * math.cos(math.radians(oe_angle))
        oey = R_HEX_OUTER * math.sin(math.radians(oe_angle))
        n_oh = f"{prefix}outer_horiz_{k}"
        a = a.with_part(
            n_oh,
            _horiz_ext(OUTER_HORIZ_LEN),
            location=z_loc * Pos(oex, oey, 0.0) * Rot(0, 0, oe_angle + 90.0),
            color=_co(n_oh),
            material=_mt(n_oh),
        )

        # Inner horizontal on inner-hex edge k — same indexing + edge
        # azimuths as outer since the hexes are aligned.
        ie_angle = (k + 1) * 60.0
        iex = R_HEX_INNER * math.cos(math.radians(ie_angle))
        iey = R_HEX_INNER * math.sin(math.radians(ie_angle))
        n_ih = f"{prefix}inner_horiz_{k}"
        a = a.with_part(
            n_ih,
            _horiz_ext(INNER_HORIZ_LEN),
            location=z_loc * Pos(iex, iey, 0.0) * Rot(0, 0, ie_angle + 90.0),
            color=_co(n_ih),
            material=_mt(n_ih),
        )

        # Radial rails on basket boundary k (between baskets k and k+1).
        # Two segments split at the inner-hex vertex so the inner-hex
        # through-column backs both adjacent rail ends. Azimuth =
        # v_angle_k; endpoints offset from the vertices in params.py.
        r_angle = v_angle
        cos_r = math.cos(math.radians(r_angle))
        sin_r = math.sin(math.radians(r_angle))
        for seg_name, seg_len, seg_mid in (
            ("radial_inner",
             RADIAL_INNER_LEN,
             (RAIL_INNER_R + RAIL_INNER_OUTER_END_R) / 2.0),
            ("radial_outer",
             RADIAL_OUTER_LEN,
             (RAIL_OUTER_INNER_END_R + RAIL_OUTER_R) / 2.0),
        ):
            rx = seg_mid * cos_r
            ry = seg_mid * sin_r
            n_r = f"{prefix}{seg_name}_{k}"
            a = a.with_part(
                n_r,
                _horiz_ext(seg_len),
                location=z_loc * Pos(rx, ry, 0.0) * Rot(0, 0, r_angle),
                color=_co(n_r),
                material=_mt(n_r),
            )

    return a


def add_through_columns(
    a: Assembly,
    z_bot: float = -VERT_EXT_BELOW,
    z_top: float = FLOOR_H,
    color_fn: Callable[[str], Color | None] | None = None,
    material_fn: Callable[[str], str | None] | None = None,
) -> Assembly:
    """Add the twelve vertical 2020 corner columns spanning
    [z_bot, z_top]: six at the outer-hex vertices and six at the
    inner-hex vertices (same azimuths since outer and inner hex are
    rotation-aligned). Single-floor default matches the historical
    behavior; a multi-floor tower passes the full-tower span here so
    one continuous extrusion replaces per-floor stacks.

    The inner verticals back the inner-hex ring against per-floor
    basket load (~20 kg/floor → 60–80 kg cumulative across 3–4
    floors), so every radial rail has both ends in compression
    instead of bending the inner hex as a horizontal beam. Inner
    columns are rotated by `v_angle` so one face aligns with the
    inner-vertex bracket's -Y clamp plate, matching the outer
    convention.

    Names: `outer_vert_k` and `inner_vert_k` for k in 0..5 (no floor
    prefix — these are global to the tower).
    """
    length = z_top - z_bot
    z_center = (z_bot + z_top) / 2.0
    column = Pos(0, 0, z_center) * Box(EXT_SIZE, EXT_SIZE, length)
    for k in range(6):
        v_angle = (k + 0.5) * 60.0
        cos_v = math.cos(math.radians(v_angle))
        sin_v = math.sin(math.radians(v_angle))
        # Rotate by v_angle so one of the 2020's faces lines up with
        # the outer bracket's vert-arm plate (bracket local -X = world
        # +X rotated by v_angle at V_k). The 2020's 4-fold cross-
        # section makes v_angle mod 90° the only thing that matters.
        for r_vert, prefix in ((R_VERT_OUTER, "outer"), (R_VERT_INNER, "inner")):
            vx = r_vert * cos_v
            vy = r_vert * sin_v
            n = f"{prefix}_vert_{k}"
            a = a.with_part(
                n,
                column,
                location=Pos(vx, vy, 0.0) * Rot(0, 0, v_angle),
                color=color_fn(n) if color_fn is not None else None,
                material=material_fn(n) if material_fn is not None else None,
            )
    return a


def add_frame(
    a: Assembly,
    color_fn: Callable[[str], Color | None] | None = None,
    material_fn: Callable[[str], str | None] | None = None,
) -> Assembly:
    """Single-floor convenience wrapper: one floor's horizontals at
    z=0 plus through-columns spanning [-VERT_EXT_BELOW, FLOOR_H].
    Multi-floor consumers should call `add_floor_horizontals` per
    floor and `add_through_columns` once for the full tower span.
    """
    a = add_floor_horizontals(
        a, z=0.0, prefix="", color_fn=color_fn, material_fn=material_fn,
    )
    a = add_through_columns(
        a, z_bot=-VERT_EXT_BELOW, z_top=FLOOR_H,
        color_fn=color_fn, material_fn=material_fn,
    )
    return a


# --- Standalone assembly ------------------------------------------------

assembly = add_frame(Assembly())

assembly = (
    assembly
    # Outer-vertex consistency at V_0 (6-fold rotational symmetry ⇒
    # other 5 vertices follow). V_0 is endpoint of edges 0 and 5
    # (edge 0 between V_0→V_1, edge 5 between V_5→V_0).
    .assert_no_interference("outer_vertex_bracket_0", "outer_vert_0")
    .assert_no_interference("outer_vertex_bracket_0", "outer_horiz_0")
    .assert_no_interference("outer_vertex_bracket_0", "outer_horiz_5")
    .assert_no_interference("outer_vertex_bracket_0", "radial_outer_0")
    .assert_no_interference("outer_vert_0", "outer_horiz_0")
    .assert_no_interference("outer_vert_0", "outer_horiz_5")
    .assert_no_interference("outer_vert_0", "radial_outer_0")
    # Adjacent outer edges meeting at V_1.
    .assert_no_interference("outer_horiz_0", "outer_horiz_1")
    # Far-apart outer edges (sanity — trivial pass).
    .assert_no_interference("outer_horiz_0", "outer_horiz_3")
    # Inner-vertex consistency at inner V_0. Inner edges 0 and 5 meet
    # at inner V_0, the inner through-column passes through it, and
    # both radial segments butt against the inner-vertex bracket
    # here.
    .assert_no_interference("inner_vertex_bracket_0", "inner_horiz_0")
    .assert_no_interference("inner_vertex_bracket_0", "inner_horiz_5")
    .assert_no_interference("inner_vertex_bracket_0", "radial_inner_0")
    .assert_no_interference("inner_vertex_bracket_0", "radial_outer_0")
    .assert_no_interference("inner_vertex_bracket_0", "inner_vert_0")
    .assert_no_interference("inner_vert_0", "inner_horiz_0")
    .assert_no_interference("inner_vert_0", "inner_horiz_5")
    .assert_no_interference("inner_vert_0", "radial_inner_0")
    .assert_no_interference("inner_vert_0", "radial_outer_0")
    # Adjacent inner edges meeting at inner V_1.
    .assert_no_interference("inner_horiz_0", "inner_horiz_1")
    # Inner vs outer separation — same azimuth with aligned hexes but
    # ≫ 100 mm radial gap, so this should trivially pass.
    .assert_no_interference("inner_horiz_0", "outer_horiz_0")
    .assert_no_interference("inner_horiz_0", "outer_vert_0")
    .assert_no_interference("inner_horiz_0", "outer_vertex_bracket_0")
    .assert_no_interference("inner_vertex_bracket_0", "outer_vertex_bracket_0")
    .assert_no_interference("inner_vert_0", "outer_vert_0")
    .assert_no_interference("inner_vert_0", "outer_horiz_0")
    .assert_no_interference("inner_vert_0", "outer_vertex_bracket_0")
    # Both radial segments clear the adjacent inner-hex edges they
    # pass between at the inner vertex (edges 0 and 5 both terminate
    # at inner V_0).
    .assert_no_interference("radial_inner_0", "inner_horiz_0")
    .assert_no_interference("radial_inner_0", "inner_horiz_5")
    .assert_no_interference("radial_outer_0", "inner_horiz_0")
    .assert_no_interference("radial_outer_0", "inner_horiz_5")
    # Outer segment sits well clear of the outer hex edges adjacent
    # to its outer-vertex landing (edges 0 and 5).
    .assert_no_interference("radial_outer_0", "outer_horiz_0")
    .assert_no_interference("radial_outer_0", "outer_horiz_5")
    # Outer-segment clearance from non-adjoining verticals / brackets
    # (sanity).
    .assert_no_interference("radial_outer_0", "outer_vert_1")
    .assert_no_interference("radial_outer_0", "outer_vertex_bracket_1")
)


if __name__ == "__main__":
    check(assembly, out="outputs")

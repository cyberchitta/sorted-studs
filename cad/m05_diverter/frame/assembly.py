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
from typing import Callable


from build123d import Box, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly

from ..inner_vertex_bracket.assembly import bracket as inner_bracket
from ..outer_vertex_bracket.assembly import (
    BRACKET_Z_TOP,
    EXT_SIZE,
    HOR_INNER_OFFSET,
    PLATE_T as OUTER_BRACKET_PLATE_T,
    bracket as outer_bracket,
)
from ..params import (
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
RADIAL_INNER_LEN = RAIL_INNER_OUTER_END_R - RAIL_INNER_R          # ≈ 39.66
RADIAL_OUTER_LEN = RAIL_OUTER_R - RAIL_OUTER_INNER_END_R          # ≈ 106.11


# --- Part helpers -------------------------------------------------------

def _horiz_ext(length: float) -> Part:
    """2020 horizontal extrusion along local +X, top face at z=0."""
    return Pos(0, 0, -EXT_SIZE / 2.0) * Box(length, EXT_SIZE, EXT_SIZE)


# Through-column reach below the floor datum: enough to cover the
# outer-vertex bracket's below-floor body (hub bottom at
# BRACKET_Z_TOP - PLATE_T = -26) with a small clearance. In the real
# tower each column is one continuous 2020 spanning all floors; in
# the single-floor wrapper, `build_through_columns` defaults to a span
# of [-VERT_EXT_BELOW, FLOOR_H] for the visible per-floor slice.
VERT_EXT_BELOW = -(BRACKET_Z_TOP - OUTER_BRACKET_PLATE_T) + 10.0  # 36


# --- Assembly builders --------------------------------------------------

def build_floor_horizontals(
    material_fn: Callable[[str], str | None] | None = None,
) -> Assembly:
    """One floor's worth of horizontal frame members + brackets as a
    sub-`Assembly` in the floor-local frame (rail-top datum at z=0).
    The parent places the whole floor with one transform
    (`Pos(0, 0, idx * FLOOR_H)` in the tower); floor identity lives in
    the parent's subtree path, so the leaf names here are plain.

    Contains: outer hex (`outer_horiz_k`), inner hex (`inner_horiz_k`),
    radial rails split at the inner-hex vertex (`radial_inner_k` /
    `radial_outer_k`), outer-vertex brackets
    (`outer_vertex_bracket_k`), and inner-vertex brackets
    (`inner_vertex_bracket_k`).

    Through-columns are NOT built here — they span all floors and live
    in `build_through_columns`.

    `material_fn` is an optional callable mapping each leaf name to a
    material string. Default `None` keeps the standalone behavior (no
    material set); multi-floor consumers thread `material_of` from the
    module-level `materials.py`. Colour is not a parameter — the unit
    root paints the finished tree (`cad/design_systems.py:paint`).
    """
    # Member-length and mirror sanity for the spans this builder cuts.
    # Loose bounds with no geometric content — a zero-length extrusion
    # is a parameter mistake, not a fit problem — so they stay bare.
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

    def _mt(name: str) -> str | None:
        return material_fn(name) if material_fn is not None else None

    a = Assembly()
    for k in range(6):
        # Shared vertex azimuth for outer and inner hex.
        v_angle = (k + 0.5) * 60.0
        vx_out = R_VERT_OUTER * math.cos(math.radians(v_angle))
        vy_out = R_VERT_OUTER * math.sin(math.radians(v_angle))
        vx_in = R_VERT_INNER * math.cos(math.radians(v_angle))
        vy_in = R_VERT_INNER * math.sin(math.radians(v_angle))

        # Outer-vertex bracket: local +X points INWARD toward hex center.
        n_ovb = f"outer_vertex_bracket_{k}"
        a = a.with_part(
            n_ovb,
            outer_bracket(),
            location=Pos(vx_out, vy_out, 0.0) * Rot(0, 0, v_angle + 180.0),
            material=_mt(n_ovb),
        )

        # Inner-vertex bracket: same orientation convention as outer.
        n_ivb = f"inner_vertex_bracket_{k}"
        a = a.with_part(
            n_ivb,
            inner_bracket(),
            location=Pos(vx_in, vy_in, 0.0) * Rot(0, 0, v_angle + 180.0),
            material=_mt(n_ivb),
        )

        # Outer horizontal on edge k (between V_k and V_{k+1}).
        # Edge midpoint at apothem angle (k + 1) * 60°.
        oe_angle = (k + 1) * 60.0
        oex = R_HEX_OUTER * math.cos(math.radians(oe_angle))
        oey = R_HEX_OUTER * math.sin(math.radians(oe_angle))
        n_oh = f"outer_horiz_{k}"
        a = a.with_part(
            n_oh,
            _horiz_ext(OUTER_HORIZ_LEN),
            location=Pos(oex, oey, 0.0) * Rot(0, 0, oe_angle + 90.0),
            material=_mt(n_oh),
        )

        # Inner horizontal on inner-hex edge k — same indexing + edge
        # azimuths as outer since the hexes are aligned.
        ie_angle = (k + 1) * 60.0
        iex = R_HEX_INNER * math.cos(math.radians(ie_angle))
        iey = R_HEX_INNER * math.sin(math.radians(ie_angle))
        n_ih = f"inner_horiz_{k}"
        a = a.with_part(
            n_ih,
            _horiz_ext(INNER_HORIZ_LEN),
            location=Pos(iex, iey, 0.0) * Rot(0, 0, ie_angle + 90.0),
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
            n_r = f"{seg_name}_{k}"
            a = a.with_part(
                n_r,
                _horiz_ext(seg_len),
                location=Pos(rx, ry, 0.0) * Rot(0, 0, r_angle),
                material=_mt(n_r),
            )

    return a


def build_through_columns(
    z_bot: float = -VERT_EXT_BELOW,
    z_top: float = FLOOR_H,
    material_fn: Callable[[str], str | None] | None = None,
) -> Assembly:
    """The twelve vertical 2020 corner columns spanning [z_bot, z_top]
    as a sub-`Assembly`: six at the outer-hex vertices and six at the
    inner-hex vertices (same azimuths since outer and inner hex are
    rotation-aligned). The z span is tower-global, so the parent
    places this subtree at identity. Single-floor default matches the
    historical behavior; a multi-floor tower passes the full-tower
    span here so one continuous extrusion replaces per-floor stacks.

    The inner verticals back the inner-hex ring against per-floor
    basket load (~20 kg/floor → 60–80 kg cumulative across 3–4
    floors), so every radial rail has both ends in compression
    instead of bending the inner hex as a horizontal beam. Inner
    columns are rotated by `v_angle` so one face aligns with the
    inner-vertex bracket's -Y clamp plate, matching the outer
    convention.

    Names: `outer_vert_k` and `inner_vert_k` for k in 0..5 (no floor
    subtree — these are global to the tower).
    """
    a = Assembly()
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
        for r_vert, ring in ((R_VERT_OUTER, "outer"), (R_VERT_INNER, "inner")):
            vx = r_vert * cos_v
            vy = r_vert * sin_v
            n = f"{ring}_vert_{k}"
            a = a.with_part(
                n,
                column,
                location=Pos(vx, vy, 0.0) * Rot(0, 0, v_angle),
                material=material_fn(n) if material_fn is not None else None,
            )
    return a


# --- Standalone assembly ------------------------------------------------

def build_frame_standalone() -> Assembly:
    """One floor's frame, under the same subtree names every consumer
    uses: horizontals under `rails` (floor-local at z=0) and the
    through-columns under `columns` (single-floor span). Assertions
    address the qualified paths."""
    return (
        Assembly()
        .with_subassembly("rails", build_floor_horizontals())
        .with_subassembly("columns", build_through_columns())
        # Outer-vertex consistency at V_0 (6-fold rotational symmetry ⇒
        # other 5 vertices follow). V_0 is endpoint of edges 0 and 5
        # (edge 0 between V_0→V_1, edge 5 between V_5→V_0).
        .assert_no_interference("rails.outer_vertex_bracket_0", "columns.outer_vert_0")
        .assert_no_interference("rails.outer_vertex_bracket_0", "rails.outer_horiz_0")
        .assert_no_interference("rails.outer_vertex_bracket_0", "rails.outer_horiz_5")
        .assert_no_interference("rails.outer_vertex_bracket_0", "rails.radial_outer_0")
        .assert_no_interference("columns.outer_vert_0", "rails.outer_horiz_0")
        .assert_no_interference("columns.outer_vert_0", "rails.outer_horiz_5")
        .assert_no_interference("columns.outer_vert_0", "rails.radial_outer_0")
        # Adjacent outer edges meeting at V_1.
        .assert_no_interference("rails.outer_horiz_0", "rails.outer_horiz_1")
        # Far-apart outer edges (sanity — trivial pass).
        .assert_no_interference("rails.outer_horiz_0", "rails.outer_horiz_3")
        # Inner-vertex consistency at inner V_0. Inner edges 0 and 5 meet
        # at inner V_0, the inner through-column passes through it, and
        # both radial segments butt against the inner-vertex bracket
        # here.
        .assert_no_interference("rails.inner_vertex_bracket_0", "rails.inner_horiz_0")
        .assert_no_interference("rails.inner_vertex_bracket_0", "rails.inner_horiz_5")
        .assert_no_interference("rails.inner_vertex_bracket_0", "rails.radial_inner_0")
        .assert_no_interference("rails.inner_vertex_bracket_0", "rails.radial_outer_0")
        .assert_no_interference("rails.inner_vertex_bracket_0", "columns.inner_vert_0")
        .assert_no_interference("columns.inner_vert_0", "rails.inner_horiz_0")
        .assert_no_interference("columns.inner_vert_0", "rails.inner_horiz_5")
        .assert_no_interference("columns.inner_vert_0", "rails.radial_inner_0")
        .assert_no_interference("columns.inner_vert_0", "rails.radial_outer_0")
        # Adjacent inner edges meeting at inner V_1.
        .assert_no_interference("rails.inner_horiz_0", "rails.inner_horiz_1")
        # Inner vs outer separation — same azimuth with aligned hexes but
        # ≫ 100 mm radial gap, so this should trivially pass.
        .assert_no_interference("rails.inner_horiz_0", "rails.outer_horiz_0")
        .assert_no_interference("rails.inner_horiz_0", "columns.outer_vert_0")
        .assert_no_interference("rails.inner_horiz_0", "rails.outer_vertex_bracket_0")
        .assert_no_interference("rails.inner_vertex_bracket_0", "rails.outer_vertex_bracket_0")
        .assert_no_interference("columns.inner_vert_0", "columns.outer_vert_0")
        .assert_no_interference("columns.inner_vert_0", "rails.outer_horiz_0")
        .assert_no_interference("columns.inner_vert_0", "rails.outer_vertex_bracket_0")
        # Both radial segments clear the adjacent inner-hex edges they
        # pass between at the inner vertex (edges 0 and 5 both terminate
        # at inner V_0).
        .assert_no_interference("rails.radial_inner_0", "rails.inner_horiz_0")
        .assert_no_interference("rails.radial_inner_0", "rails.inner_horiz_5")
        .assert_no_interference("rails.radial_outer_0", "rails.inner_horiz_0")
        .assert_no_interference("rails.radial_outer_0", "rails.inner_horiz_5")
        # Outer segment sits well clear of the outer hex edges adjacent
        # to its outer-vertex landing (edges 0 and 5).
        .assert_no_interference("rails.radial_outer_0", "rails.outer_horiz_0")
        .assert_no_interference("rails.radial_outer_0", "rails.outer_horiz_5")
        # Outer-segment clearance from non-adjoining verticals / brackets
        # (sanity).
        .assert_no_interference("rails.radial_outer_0", "columns.outer_vert_1")
        .assert_no_interference("rails.radial_outer_0", "rails.outer_vertex_bracket_1")
    )


# Degenerate master, so `khana check <file>` resolves with no `:factory`.
assembly = build_frame_standalone()

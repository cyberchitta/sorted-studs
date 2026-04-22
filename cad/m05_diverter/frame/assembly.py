"""
M05 hex-tower — frame sub-assembly (one generic middle floor).

Structural 2020-aluminum cage for one floor: an outer hex ring
(apothem R_HEX_OUTER) with 6 vertical corner columns at the outer-
hex vertices and an outer-vertex bracket at each; plus a concentric
inner hex ring (apothem R_HEX_INNER) sharing the outer hex's
rotation (aligned vertex azimuths). Six radial 2020 rails run along
basket boundaries from each inner-hex vertex out to the matching
outer-hex vertex; an inner-vertex bracket at each inner-hex vertex
joins its two hex edges to the radial.

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
- Inner-hex vertical columns (full-height vs per-floor is open in
  hex-cascade.md).
- Diagonal braces.
- Top cap and base floor (different corner-bracket variants).

Coordinate frame:
    origin = column axis (shared with basket / ramp_mechanism)
    +X     = basket 0 bisector
    +Z     = up
    z=0    = shared floor datum. Horizontal ring extrusions sit with
             top face at z=0 (bodies below). Vertical corner columns
             rise UP from z=0 (bottom face at z=0) to carry the next
             floor above. Bucket floor and bearing/spider base also
             sit at z=0. This is a **temporary** per-floor model —
             the real tower will use continuous corner columns
             spanning all floors (hex-cascade.md), which will
             promote the verticals out of this per-floor file.

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

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build123d import Box, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

from inner_vertex_bracket.assembly import bracket as inner_bracket
from outer_vertex_bracket.assembly import (
    EXT_SIZE,
    HOR_INNER_OFFSET,
    bracket as outer_bracket,
)
from params import (
    BRACKET_HOR_OFFSET,
    FLOOR_H,
    RAIL_INNER_R,
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

# Radial rail length: the physical endpoints are already offset from
# the vertex radii in params.py (RAIL_INNER_R = R_VERT_INNER +
# BRACKET_HOR_OFFSET, etc.), so this is just the span.
RADIAL_LEN = RAIL_OUTER_R - RAIL_INNER_R

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
assert RADIAL_LEN > 0, (
    f"RADIAL_LEN = {RADIAL_LEN:.1f} mm ≤ 0 — RAIL_OUTER_R must exceed "
    f"RAIL_INNER_R."
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


def _vert_ext(length: float) -> Part:
    """2020 vertical extrusion rising above the floor datum: bottom
    face at z=0, extending up to z=+length."""
    return Pos(0, 0, length / 2.0) * Box(EXT_SIZE, EXT_SIZE, length)


# --- Assembly builder ---------------------------------------------------

def add_frame(a: Assembly) -> Assembly:
    """Add all frame parts (extrusions + brackets) for one generic
    floor and return the updated assembly. Exposed so the top-level
    diverter assembly can compose frame + ramp_mechanism + baskets."""
    for k in range(6):
        # Shared vertex azimuth for outer and inner hex.
        v_angle = (k + 0.5) * 60.0
        vx_out = R_VERT_OUTER * math.cos(math.radians(v_angle))
        vy_out = R_VERT_OUTER * math.sin(math.radians(v_angle))
        vx_in = R_VERT_INNER * math.cos(math.radians(v_angle))
        vy_in = R_VERT_INNER * math.sin(math.radians(v_angle))

        # Rotate the vertical extrusion by v_angle so one of its four
        # faces lines up with the outer bracket's vert-arm plate
        # (bracket local -X = world +X rotated by v_angle at V_k). The
        # 2020's 4-fold square cross-section makes v_angle mod 90°
        # the only thing that matters.
        a = a.add(
            f"outer_vert_{k}",
            _vert_ext(FLOOR_H),
            location=Pos(vx_out, vy_out, 0.0) * Rot(0, 0, v_angle),
        )

        # Outer-vertex bracket: local +X points INWARD toward hex center.
        a = a.add(
            f"outer_vertex_bracket_{k}",
            outer_bracket(),
            location=Pos(vx_out, vy_out, 0.0) * Rot(0, 0, v_angle + 180.0),
        )

        # Inner-vertex bracket: same orientation convention as outer.
        a = a.add(
            f"inner_vertex_bracket_{k}",
            inner_bracket(),
            location=Pos(vx_in, vy_in, 0.0) * Rot(0, 0, v_angle + 180.0),
        )

        # Outer horizontal on edge k (between V_k and V_{k+1}).
        # Edge midpoint at apothem angle (k + 1) * 60°.
        oe_angle = (k + 1) * 60.0
        oex = R_HEX_OUTER * math.cos(math.radians(oe_angle))
        oey = R_HEX_OUTER * math.sin(math.radians(oe_angle))
        a = a.add(
            f"outer_horiz_{k}",
            _horiz_ext(OUTER_HORIZ_LEN),
            location=Pos(oex, oey, 0.0) * Rot(0, 0, oe_angle + 90.0),
        )

        # Inner horizontal on inner-hex edge k — same indexing + edge
        # azimuths as outer since the hexes are aligned.
        ie_angle = (k + 1) * 60.0
        iex = R_HEX_INNER * math.cos(math.radians(ie_angle))
        iey = R_HEX_INNER * math.sin(math.radians(ie_angle))
        a = a.add(
            f"inner_horiz_{k}",
            _horiz_ext(INNER_HORIZ_LEN),
            location=Pos(iex, iey, 0.0) * Rot(0, 0, ie_angle + 90.0),
        )

        # Radial rail on basket boundary k (between baskets k and k+1).
        # Azimuth = v_angle_k; physical endpoints at RAIL_INNER_R and
        # RAIL_OUTER_R (already offset from the vertices in params.py).
        r_angle = v_angle
        r_mid = (RAIL_INNER_R + RAIL_OUTER_R) / 2.0
        rx = r_mid * math.cos(math.radians(r_angle))
        ry = r_mid * math.sin(math.radians(r_angle))
        a = a.add(
            f"radial_{k}",
            _horiz_ext(RADIAL_LEN),
            location=Pos(rx, ry, 0.0) * Rot(0, 0, r_angle),
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
    .assert_no_interference("outer_vertex_bracket_0", "radial_0")
    .assert_no_interference("outer_vert_0", "outer_horiz_0")
    .assert_no_interference("outer_vert_0", "outer_horiz_5")
    .assert_no_interference("outer_vert_0", "radial_0")
    # Adjacent outer edges meeting at V_1.
    .assert_no_interference("outer_horiz_0", "outer_horiz_1")
    # Far-apart outer edges (sanity — trivial pass).
    .assert_no_interference("outer_horiz_0", "outer_horiz_3")
    # Inner-vertex consistency at inner V_0. Inner edges 0 and 5 meet
    # at inner V_0, and radial_0's inner end lands here.
    .assert_no_interference("inner_vertex_bracket_0", "inner_horiz_0")
    .assert_no_interference("inner_vertex_bracket_0", "inner_horiz_5")
    .assert_no_interference("inner_vertex_bracket_0", "radial_0")
    # Adjacent inner edges meeting at inner V_1.
    .assert_no_interference("inner_horiz_0", "inner_horiz_1")
    # Inner vs outer separation — same azimuth with aligned hexes but
    # ≫ 100 mm radial gap, so this should trivially pass.
    .assert_no_interference("inner_horiz_0", "outer_horiz_0")
    .assert_no_interference("inner_horiz_0", "outer_vert_0")
    .assert_no_interference("inner_horiz_0", "outer_vertex_bracket_0")
    .assert_no_interference("inner_vertex_bracket_0", "outer_vertex_bracket_0")
    # Radial 0 clears the adjacent inner-hex edges it passes between
    # at the inner vertex (edges 0 and 5 both terminate at inner V_0).
    .assert_no_interference("radial_0", "inner_horiz_0")
    .assert_no_interference("radial_0", "inner_horiz_5")
    # Radial sits well clear of the outer hex edges adjacent to its
    # outer-vertex landing (edges 0 and 5).
    .assert_no_interference("radial_0", "outer_horiz_0")
    .assert_no_interference("radial_0", "outer_horiz_5")
    # Radial clearance from non-adjoining verticals / brackets (sanity).
    .assert_no_interference("radial_0", "outer_vert_1")
    .assert_no_interference("radial_0", "outer_vertex_bracket_1")
)


if __name__ == "__main__":
    check(assembly, out="outputs")

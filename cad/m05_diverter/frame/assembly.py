"""
M05 hex-tower — frame sub-assembly (one generic middle floor).

Structural 2020-aluminum cage for one floor: an outer hex ring
(apothem R_HEX_OUTER) with 6 vertical corner columns and 6 top-corner
brackets, plus a concentric inner hex ring (apothem R_HEX_INNER)
rotated 30° relative to the outer so its edge midpoints land on
basket bisectors.

Why the 30° rotation: outer hex vertices are placed at basket
bisectors so basket outer corners sit cleanly between hex edges and
hex vertices. If the inner hex shared the same orientation, its
vertices would land on basket bisectors too and poke into basket
interiors. Rotating the inner hex by 30° puts its vertices on basket
boundaries and its edge midpoints on basket bisectors, where the
inner hex edge lies radially inside the basket inner face
(R_BASKET_INNER·cos 30° ≈ 129.9 mm at R_BASKET_INNER=150 mm; inner
hex at apothem 120 mm leaves ≈ 10 mm radial clearance).

V1 scope. This file models a MIDDLE floor. Explicitly deferred:
- Inner-hex vertical columns (full-height vs per-floor is open in
  hex-cascade.md).
- Diagonal braces.
- 4-way inner-vertex bracket joining two inner-hex edges + one
  radial rail at each inner-hex vertex (inner-hex extrusions
  currently stop 20 mm short of each inner vertex and radial rails
  currently terminate at the same point without a physical joint).
- Outer-hex mid-edge bracket carrying the radial rail's outer end
  (the outer hex provides an edge midpoint, not a vertex, where the
  radial terminates).
- Top cap and base floor (different corner-bracket variants).

Coordinate frame:
    origin = column axis (shared with basket / ramp_mechanism)
    +X     = basket 0 bisector = outer hex vertex 0 direction
    +Z     = up
    z=0    = shared floor datum. Horizontal ring extrusions sit with
             top face at z=0 (bodies below). Vertical corner columns
             rise UP from z=0 (bottom face at z=0) to carry the next
             floor above. Bucket floor and bearing/spider base also
             sit at z=0. This is a **temporary** per-floor model —
             the real tower will use continuous corner columns
             spanning all floors (hex-cascade.md), which will
             promote the verticals out of this per-floor file.

Corner-bracket orientation note. corner_bracket/assembly.py places
its two horizontal arms at ±60° from its local +X (interior angle
120°). For the arms to align with the hex edges meeting at a vertex,
the bracket's local +X must point INWARD toward the hex center (the
120° opening of the V fits the 120° interior angle of the hex). So
we place each bracket at V_k with rotation (v_angle_k + 180°).
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build123d import Box, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

from corner_bracket.assembly import (
    EXT_SIZE,
    HOR_INNER_OFFSET,
    bracket,
)
from params import (
    FLOOR_H,
    RAIL_INNER_R,
    RAIL_OUTER_R,
    RAIL_SIZE,
    R_HEX_INNER,
    R_HEX_OUTER,
    R_VERT_INNER,
)


# --- Derived geometry --------------------------------------------------

# Regular hexagon: circumradius = apothem / cos(30°) = apothem * 2/√3,
# and edge length equals circumradius. R_VERT_INNER is shared via
# params.py (radial rails reference it too); R_VERT_OUTER stays local.
_APOTHEM_TO_VERTEX = 2.0 / math.sqrt(3.0)

R_VERT_OUTER = R_HEX_OUTER * _APOTHEM_TO_VERTEX
EDGE_LEN_OUTER = R_VERT_OUTER
EDGE_LEN_INNER = R_VERT_INNER

# Outer horizontals butt up to the corner-bracket arms at each end;
# HOR_INNER_OFFSET is the arm's reserved length from the hex vertex.
OUTER_HORIZ_LEN = EDGE_LEN_OUTER - 2.0 * HOR_INNER_OFFSET

# Inner hex has no brackets yet; reserve free length at each inner
# vertex so a future 4-way junction bracket (two inner-hex edges +
# one radial rail + optional vertical) can drop in without having
# to re-length the extrusions. The value is also sized to keep the
# inner-hex edge ends clear of the bearing_spider's saddle side
# walls, which pass over the inner-hex vertex azimuth; an extrusion
# end at clear=20 clips the saddle wall by ~1 mm.
INNER_HORIZ_VERTEX_CLEAR = 25.0
INNER_HORIZ_LEN = EDGE_LEN_INNER - 2.0 * INNER_HORIZ_VERTEX_CLEAR

# Radial rails: run along basket boundaries (azimuth (k+0.5)·60°) from
# inner-hex vertex out to outer-hex edge midpoint. Derived length
# follows the endpoint radii committed in params.py.
RADIAL_LEN = RAIL_OUTER_R - RAIL_INNER_R

assert OUTER_HORIZ_LEN > 0, (
    f"OUTER_HORIZ_LEN = {OUTER_HORIZ_LEN:.1f} mm ≤ 0 — increase R_HEX_OUTER "
    f"or decrease HOR_INNER_OFFSET."
)
assert INNER_HORIZ_LEN > 0, (
    f"INNER_HORIZ_LEN = {INNER_HORIZ_LEN:.1f} mm ≤ 0 — increase R_HEX_INNER "
    f"or decrease INNER_HORIZ_VERTEX_CLEAR."
)
assert RAIL_SIZE == EXT_SIZE, (
    f"RAIL_SIZE ({RAIL_SIZE}) must equal frame EXT_SIZE ({EXT_SIZE}); "
    f"the radial rail is a standard 2020 and reuses the frame helper."
)
assert RADIAL_LEN > 0, (
    f"RADIAL_LEN = {RADIAL_LEN:.1f} mm ≤ 0 — RAIL_OUTER_R must exceed "
    f"RAIL_INNER_R."
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
        v_angle = k * 60.0
        vx = R_VERT_OUTER * math.cos(math.radians(v_angle))
        vy = R_VERT_OUTER * math.sin(math.radians(v_angle))

        # Rotate the vertical extrusion by v_angle so one of its four
        # faces lines up with the bracket's vert-arm plate (bracket
        # local -X = world +X rotated by v_angle at V_k). The 2020's
        # 4-fold square cross-section makes v_angle mod 90° the only
        # thing that matters.
        a = a.add(
            f"outer_vert_{k}",
            _vert_ext(FLOOR_H),
            location=Pos(vx, vy, 0.0) * Rot(0, 0, v_angle),
        )

        # Bracket local +X points inward — see module docstring.
        a = a.add(
            f"corner_bracket_{k}",
            bracket(),
            location=Pos(vx, vy, 0.0) * Rot(0, 0, v_angle + 180.0),
        )

        # Outer horizontal on edge k (between V_k and V_{k+1}).
        # Edge midpoint at apothem angle (k + 0.5) * 60°.
        oe_angle = (k + 0.5) * 60.0
        oex = R_HEX_OUTER * math.cos(math.radians(oe_angle))
        oey = R_HEX_OUTER * math.sin(math.radians(oe_angle))
        a = a.add(
            f"outer_horiz_{k}",
            _horiz_ext(OUTER_HORIZ_LEN),
            location=Pos(oex, oey, 0.0) * Rot(0, 0, oe_angle + 90.0),
        )

        # Inner horizontal on inner-hex edge k. Inner hex is rotated
        # 30° from outer so its apothem angles sit at k*60° (basket
        # bisectors). Edge is perpendicular to the apothem direction.
        ie_angle = k * 60.0
        iex = R_HEX_INNER * math.cos(math.radians(ie_angle))
        iey = R_HEX_INNER * math.sin(math.radians(ie_angle))
        a = a.add(
            f"inner_horiz_{k}",
            _horiz_ext(INNER_HORIZ_LEN),
            location=Pos(iex, iey, 0.0) * Rot(0, 0, ie_angle + 90.0),
        )

        # Radial rail on basket boundary k (between baskets k and k+1).
        # Axis along azimuth (k+0.5)·60°; inner end at the inner-hex
        # vertex radius, outer end at the outer-hex edge midpoint. The
        # local helper places the extrusion along +X with root at
        # x=length/2; translate to place the midpoint at the midpoint
        # of the two endpoint radii.
        r_angle = (k + 0.5) * 60.0
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
    # Corner consistency at vertex 0 (6-fold rotational symmetry ⇒
    # other 5 vertices follow).
    .assert_no_interference("corner_bracket_0", "outer_vert_0")
    .assert_no_interference("corner_bracket_0", "outer_horiz_0")
    .assert_no_interference("corner_bracket_0", "outer_horiz_5")
    .assert_no_interference("outer_vert_0", "outer_horiz_0")
    .assert_no_interference("outer_vert_0", "outer_horiz_5")
    # Adjacent outer edges meeting at vertex 1.
    .assert_no_interference("outer_horiz_0", "outer_horiz_1")
    # Far-apart outer edges (sanity — trivial pass).
    .assert_no_interference("outer_horiz_0", "outer_horiz_3")
    # Inner edges meeting at an inner-hex vertex (inner hex rotated
    # 30° from outer; inner edges 0 and 1 meet at inner vertex 30°).
    .assert_no_interference("inner_horiz_0", "inner_horiz_1")
    # Inner vs outer separation.
    .assert_no_interference("inner_horiz_0", "outer_horiz_0")
    .assert_no_interference("inner_horiz_0", "outer_vert_0")
    .assert_no_interference("inner_horiz_0", "corner_bracket_0")
    # Radial 0 (between basket 0 and 1, azimuth 30°) is the
    # representative radial. 6-fold symmetry covers the other five.
    # It meets the inner hex at the shared vertex of inner_horiz_0 /
    # inner_horiz_1 (both end 20 mm short of the vertex), so the
    # junction bracket is deferred and the rail should clear both
    # inner edges today.
    .assert_no_interference("radial_0", "inner_horiz_0")
    .assert_no_interference("radial_0", "inner_horiz_1")
    # Radial outer end lands on outer-hex edge 0's midpoint (edges
    # 0 and 1 both meet the rail at the outer hex — edge 0 is the
    # one whose midpoint is at azimuth 30°). The rail is radial (not
    # along the edge), so it butts into the extrusion from the side
    # and currently overlaps it — mid-edge bracket is deferred.
    .assert_interference(
        "radial_0", "outer_horiz_0",
        reason="outer-hex mid-edge bracket for radial not yet designed",
    )
    # Radials sit well clear of outer verticals / corner brackets
    # (>40 mm tangential separation at the outer hex).
    .assert_no_interference("radial_0", "outer_vert_0")
    .assert_no_interference("radial_0", "corner_bracket_0")
)


if __name__ == "__main__":
    check(assembly, out="outputs")

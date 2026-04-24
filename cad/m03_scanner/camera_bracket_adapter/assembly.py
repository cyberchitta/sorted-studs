"""
M03 scanner — tilted camera-bracket adapter (rev 1).

Wedge-shaped printed spacer that bridges the flat enclosure front
panel (axis-aligned to world X-Y) and the 25°-tilted back face of the
camera bracket. Without this part the bracket's back face contacts
the front wall along a single line; with it, four M4 bolts clamp
bracket → adapter on the tilted face and four more M4 bolts clamp
adapter → wall on the flat face.

Coordinate frame (local to this sub-assembly):
    origin = centre of the adapter back face (the face that contacts
             the enclosure front-wall inner face)
    +Z     = away from the wall, toward the brick (the scanner top-
             level places this sub-assembly without rotation, so
             local +Z = world +Z)
    +X, +Y = span the back face (= world +X / +Y)
    z = 0                                 : back face (against wall)
    z ∈ [0, _tilt_plane(y)]               : adapter body
    _tilt_plane(y) = y·tan(TILT) + DEPTH  : tilted front face plane
       DEPTH = WEDGE_MID_DEPTH (back face → bracket back face centre,
       measured along world +Z)

The adapter origin is offset in world +Y from the brick centroid by
CAMERA_POS.y + PLATE_T·sin(TILT) (≈ 107.81 mm) so that the bracket
back face centre lands on the adapter front face centre and the
adapter is symmetric about the bracket's vertical axis.

Two bolt groups, both modelled as plain M4 through-holes at this rev
(matching `mirror_mount/` and `camera_bracket/` rev 1 conventions —
heat-set insert pockets and exact bolt-length modelling are a rev 2
follow-up):

    bracket bolts (4×) — tilted axis (0, +sin TILT, -cos TILT) through
        the four entries on the tilted front face. The 52 mm square
        pattern matches the bracket's CAMERA_BRACKET_BOLT_SPACING.
    wall bolts (4×)    — straight world +Z axis through the four
        entries on the flat back face. Pattern is an inner 36 mm
        square (WALL_BOLT_X = WALL_BOLT_Y = 18 mm) chosen to keep the
        wall-bolt cylinders in different x-planes (±18) from the
        bracket-bolt cylinders (±26) — minimum axis-to-axis distance
        across the two groups is 8 mm, leaving ≥ 3.5 mm wall between
        cylinder surfaces (well above WALL_MIN).

Print orientation: back face on the build plate, wedge extends
upward along +Z. Tilted front face is 25° off vertical → no overhang
problem at overhang_max_deg = 45°. Tilted bolt cylinders are also
25° off vertical → same.
"""

import math
import sys
from pathlib import Path

from build123d import Box, Cylinder, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

# Standalone-run import: params.py lives one directory up. Same
# pattern as the other m03 sub-assemblies.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
    CAMERA_BRACKET_BACK_CENTRE,
    CAMERA_BRACKET_BOLT_SPACING,
    CAMERA_BRACKET_PLATE_SIZE,
    CAMERA_BRACKET_PLATE_T,
    CAMERA_ROT_DEG,
    ENCLOSURE_INNER_Z_MIN,
    WALL_MIN,
)


# --- Parameters ---------------------------------------------------------

TILT_DEG = CAMERA_ROT_DEG[0]
_tilt_rad = math.radians(TILT_DEG)
_sin_tilt = math.sin(_tilt_rad)
_cos_tilt = math.cos(_tilt_rad)

# Back face / front face X-Y dimensions. Match the bracket plate so
# the tilted front face fully contains the bracket's footprint with
# every corner bolt landing inside the face perimeter.
ADAPTER_W = CAMERA_BRACKET_PLATE_SIZE
ADAPTER_H = CAMERA_BRACKET_PLATE_SIZE

# Distance from wall (back face, local z=0) to bracket back face
# centre, measured along world / local +Z. Derived in params.py.
WEDGE_MID_DEPTH = CAMERA_BRACKET_BACK_CENTRE[2] - ENCLOSURE_INNER_Z_MIN

# Stock prism height for the cut-and-subtract construction. Must
# exceed the tilted plane's max z over the back-face y range so the
# tilt cut lands fully inside the prism.
_max_plane_z = (ADAPTER_H / 2) * (_sin_tilt / _cos_tilt) + WEDGE_MID_DEPTH
PRISM_H = _max_plane_z + 5.0

# Bracket bolt pattern — half-spacing in each axis on a square.
BRACKET_BOLT_HALF = CAMERA_BRACKET_BOLT_SPACING / 2

# Wall bolt pattern — inner square. Spacing chosen so the wall-bolt
# cylinders stay in different x-planes from the bracket-bolt
# cylinders (BRACKET_BOLT_HALF = 26 → wall at 18 leaves an 8 mm gap
# between axis x-planes).
WALL_BOLT_X = 18.0
WALL_BOLT_Y = 18.0

# Bolt clearance hole diameter. M4 → 4.5 mm matches the bracket and
# mirror-mount rev 1 conventions.
BOLT_HOLE_D = 4.5


# --- Bolt entry geometry helpers ---------------------------------------

def _bracket_bolt_entry_local(sx: int, sy: int) -> tuple[float, float, float]:
    """Tilted-face entry point of one bracket bolt, in adapter-local
    coords. Derived by transforming the bracket's local bolt position
    (sx·HALF, sy·HALF, -PLATE_T) through Rot(TILT, 0, 0), then
    converting the world result into adapter-local."""
    bx = sx * BRACKET_BOLT_HALF
    by = sy * BRACKET_BOLT_HALF
    bz = -CAMERA_BRACKET_PLATE_T

    # Rot(TILT, 0, 0) acting on (bx, by, bz):
    wy = by * _cos_tilt - bz * _sin_tilt
    wz = by * _sin_tilt + bz * _cos_tilt

    # Convert to adapter-local. Adapter origin in world is
    # (0, CAMERA_BRACKET_BACK_CENTRE[1], ENCLOSURE_INNER_Z_MIN).
    # Camera origin (PCB rear) in world is CAMERA_POS — but we only
    # need the offset between bracket origin (PCB rear) and adapter
    # origin: (0, CAMERA_POS[1] - adapter_y, CAMERA_POS[2] - adapter_z)
    # = (0, -PLATE_T·sin TILT, CAMERA_POS[2] - ENCLOSURE_INNER_Z_MIN).
    # The bracket bolt's adapter-local position then folds the rotated
    # offset (wy, wz) onto that translation.
    ax = bx
    ay = wy - CAMERA_BRACKET_PLATE_T * _sin_tilt
    az = wz + (CAMERA_BRACKET_BACK_CENTRE[2] + CAMERA_BRACKET_PLATE_T * _cos_tilt
               - ENCLOSURE_INNER_Z_MIN)
    # CAMERA_BRACKET_BACK_CENTRE[2] + PLATE_T·cos TILT == CAMERA_POS[2],
    # so az = wz + (CAMERA_POS[2] - ENCLOSURE_INNER_Z_MIN). Kept in
    # this form so the derivation reads from params symbols only.
    return ax, ay, az


# --- Parametric sanity checks ------------------------------------------

# Tilted-bolt vs wall-bolt minimum axis-to-axis distance: each bracket
# bolt cylinder lies in plane x = ±BRACKET_BOLT_HALF; each wall bolt
# in plane x = ±WALL_BOLT_X. With both centres in YZ planes of
# different x, the perpendicular between them is purely along x and
# has magnitude |BRACKET_BOLT_HALF - WALL_BOLT_X|. Same-sign pair is
# the worst case.
_x_axis_gap = BRACKET_BOLT_HALF - WALL_BOLT_X
_surface_gap = _x_axis_gap - BOLT_HOLE_D
assert _surface_gap >= WALL_MIN, (
    f"tilted↔wall bolt surface gap {_surface_gap:.2f} mm "
    f"< {WALL_MIN} mm (axis x-gap = {_x_axis_gap:.2f} mm)"
)

# Wall-bolt to adapter-edge wall (X and Y).
_wall_edge_x = ADAPTER_W / 2 - WALL_BOLT_X - BOLT_HOLE_D / 2
_wall_edge_y = ADAPTER_H / 2 - WALL_BOLT_Y - BOLT_HOLE_D / 2
assert _wall_edge_x >= WALL_MIN, (
    f"wall-bolt → adapter-edge X wall {_wall_edge_x:.2f} mm < {WALL_MIN} mm"
)
assert _wall_edge_y >= WALL_MIN, (
    f"wall-bolt → adapter-edge Y wall {_wall_edge_y:.2f} mm < {WALL_MIN} mm"
)

# Bracket bolts must enter inside the adapter front face. Their
# entry y must be within ±ADAPTER_H/2 with a wall margin to the
# tilted-face long edges (the y=±ADAPTER_H/2 lines of the prism).
for _sx in (-1, 1):
    for _sy in (-1, 1):
        _, _ay, _az = _bracket_bolt_entry_local(_sx, _sy)
        _edge_clear = ADAPTER_H / 2 - abs(_ay) - BOLT_HOLE_D / 2
        assert _edge_clear >= WALL_MIN, (
            f"bracket bolt entry y={_ay:.2f} → tilted-face long-edge wall "
            f"{_edge_clear:.2f} mm < {WALL_MIN} mm"
        )
        assert _az > 0, (
            f"bracket bolt entry z={_az:.2f} ≤ 0 — wedge too thin "
            f"(bracket back face would intersect the wall)"
        )

# Bracket bolts in X must clear the adapter X edges.
_bracket_edge_x = ADAPTER_W / 2 - BRACKET_BOLT_HALF - BOLT_HOLE_D / 2
assert _bracket_edge_x >= WALL_MIN, (
    f"bracket-bolt → adapter-edge X wall {_bracket_edge_x:.2f} mm < {WALL_MIN} mm"
)


# --- Part function -----------------------------------------------------

def adapter(
    adapter_w: float = ADAPTER_W,
    adapter_h: float = ADAPTER_H,
    prism_h: float = PRISM_H,
    wedge_mid_depth: float = WEDGE_MID_DEPTH,
    tilt_deg: float = TILT_DEG,
    bolt_hole_d: float = BOLT_HOLE_D,
    wall_bolt_x: float = WALL_BOLT_X,
    wall_bolt_y: float = WALL_BOLT_Y,
) -> Part:
    """Tilted wedge with two bolt-hole groups.

    Body construction: a rectangular prism sized adapter_w × adapter_h
    × prism_h, then a half-space cut by the tilted plane
    z = y·tan(tilt) + wedge_mid_depth removes the top, leaving the
    wedge with a flat back at z=0 and a tilted front at the cut plane.

    Hole construction: each bolt is a long cylinder centred on its
    entry point. Bracket bolts are oriented along the bracket's
    bolt axis (Rot(tilt, 0, 0) on the canonical +Z-axis cylinder);
    wall bolts stay axis-aligned. All cylinders are long enough to
    cut clear through the wedge — modelling them as through-holes is
    a deliberate rev 1 simplification (see module docstring)."""
    prism = Pos(0, 0, prism_h / 2) * Box(adapter_w, adapter_h, prism_h)

    # Tilt cutter: rotated box whose local -Z face lies on the cut
    # plane and whose body sits on the +normal side (the side to
    # remove). Box width covers the prism with margin.
    cut_size = max(adapter_w, adapter_h) * 3
    cut_h = prism_h * 2
    cutter = (
        Pos(0, 0, wedge_mid_depth)
        * Rot(tilt_deg, 0, 0)
        * Pos(0, 0, cut_h / 2)
        * Box(cut_size, cut_size, cut_h)
    )
    body = prism - cutter

    # Bolt cylinders are all longer than any chord through the wedge,
    # so they cleanly punch through wherever the bolt axis enters the
    # body.
    cyl_len = max(adapter_w, adapter_h, prism_h) * 3

    # Bracket bolts (4×) — tilted axis.
    for sx in (-1, 1):
        for sy in (-1, 1):
            ex, ey, ez = _bracket_bolt_entry_local(sx, sy)
            body -= (
                Pos(ex, ey, ez)
                * Rot(tilt_deg, 0, 0)
                * Cylinder(bolt_hole_d / 2, cyl_len)
            )

    # Wall bolts (4×) — axis-aligned along +Z.
    for sx in (-1, 1):
        for sy in (-1, 1):
            body -= (
                Pos(sx * wall_bolt_x, sy * wall_bolt_y, prism_h / 2)
                * Cylinder(bolt_hole_d / 2, cyl_len)
            )

    return body


# --- Assembly ----------------------------------------------------------

# Single-part sub-assembly — the meaningful clearance/interference
# checks live on the adjacent geometry (front wall, camera bracket)
# and so run in the scanner top-level `assembly.py`. Standalone,
# `check()` reports volume + bbox so a numerical sanity glance is
# possible without spinning up the rest of the rig.
assembly = Assembly().add("adapter", adapter())


if __name__ == "__main__":
    check(assembly, out="outputs")
    inspect(
        adapter(),
        method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=45.0),
        out="outputs",
        name="adapter",
    )

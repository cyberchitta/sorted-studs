"""
M05 hex-tower — basket sub-assembly.

Annular-sector **buckets** with cylindrical-arc inner and outer
walls and two radial side walls, sitting on a rotating-column
floor. The floor is a **mixed layout** — baskets of four sizes
(N=6 / 9 / 12 / 18, each with half-angle 180°/N) placed around
the hex per `params.FLOOR_LAYOUT`, with the ramp-facing N=6 basket
straddling +X and the remaining sections filled by N=9 triples,
N=12 pairs, and an N=18 triple + (N=9, N=18) pair around the back.

Outer arc radius matches the top-deck acrylic disc OD/2 (330 mm)
and inner arc radius matches the hex inner-face apothem (130 mm),
so all baskets share the same outer **and** inner cylindrical
surfaces regardless of N. The lip-radius is constant at every
azimuth, giving uniform 16.4 mm overhang past the ramp tip across
all basket sizes (vs. the earlier straight-chord inner wall that
varied the lip radius from 129.9 mm at N=6 to 147.7 mm at N=18 —
N=18 sat outboard of the tip and a near-vertical drop fell into
the gap).

Each basket's radial side walls (and in one case, its center) have
open radial access channels around the through-columns at vertices
V_k = (k+0.5)·60°.
With outer and inner hex aligned, every vertex azimuth carries two
columns — one at R_VERT_OUTER and one at R_VERT_INNER — so each
flagged azimuth cuts the basket twice: from the inner arc out through
the inner column, and from the outer column out through the outer arc.
These channels let a basket move horizontally around the continuous
tower columns during installation and removal; closed column-footprint
holes would force it to slide the full tower height instead. The notch
set per basket is declared in `FLOOR_LAYOUT`:
`left`/`right` for boundary columns, `middle` for the V_1 column
that sits inside the N=9 MIDDLE basket's interior.

Floor sits on the shared stage datum (z=0); rim at z=+BASKET_DEPTH_Z.
Basket floor cantilevers past the outer-hex edge but still rests on
the hex outer extrusion top-face strip plus the radial rails, so
brick load drops through the supported zone. Open top for drop-in
delivery, closed bottom; withdraws horizontally through the open
column-access channels for emptying. Rack that
physically carries the bucket load is the hex ring itself — no
separate shelf.

Standalone validation: the full mixed layout + a minimal column-
footprint stub at the axis, with interference checks against the
stub and between every adjacent pair (incl. wrap). The authoritative
check against real column / bearing-spider / ramp / frame geometry
runs in the top-level `../assembly.py`.

Coordinate frame:
    origin = column axis ∩ floor datum
    +Z     = up
    +X     = radial outward; `basket_000_06` sits on the +X azimuth
    z=0    = floor datum (basket floor sits here)
"""

import math
from typing import Callable


from build123d import Box, Color, Cylinder, Part, Polygon, Pos, Rot, extrude

from cad_khana.mechanism.assembly import Assembly

from ..params import (
    BASKET_DEPTH_Z,
    BASKET_FLOOR_Z,
    BASKET_RIM_Z,
    COL_OUTER_SIDE,
    FLOOR_LAYOUT,
    R_BASKET_INNER,
    R_HEX_OUTER,
    R_VERT_INNER,
    R_VERT_OUTER,
)


# --- Parameters ---------------------------------------------------------

# Basket outer wall is a cylindrical arc at r = R_BASKET_OUTER,
# aligned with the clear-acrylic top deck OD/2 (660/2 = 330) so the
# floor of buckets reads as a continuous cylindrical skirt below the
# disc edge. Sits BASKET_HEX_OVERHANG mm outside R_HEX_OUTER along
# the apothem direction, so the basket floor cantilevers past the
# hex outer face by up to BASKET_HEX_OVERHANG. The floor still rests
# on the hex outer extrusion top-face strip (r ∈ [260, 280] along
# apothem azimuths) plus the radial rails along boundary azimuths,
# so brick load drops through the supported zone — only the
# outermost ~50 mm of floor hangs free.
#
# Basket inner wall is a cylindrical arc at r = R_BASKET_INNER =
# 130, exactly on the hex inner-face apothem. Along bisector
# azimuths the basket floor sits over the full 20 mm hex top strip
# (r ∈ [130, 150]) for solid support. At vertex azimuths the hex
# inner corner sits at r ≈ 150.1, so the basket floor extends ~20
# mm inboard of the hex as an unsupported flap (max 20.1 mm at the
# vertex line, tapering to 0 mm at the bisectors). The flap shares
# the z ∈ [0, 3] band with the inner-vertex bracket bodies; their
# coexistence is resolved at the top-level assembly.
#
# The basket's radial side walls lie on the shared (outer = inner)
# hex vertex azimuths and would otherwise pass straight through the
# through-columns at both radii — at R_VERT_OUTER the column body
# spans r ∈ [~301.77, ~321.77] along the vertex azimuth (fully
# inside the basket envelope at r=330); at R_VERT_INNER it spans
# r ∈ [~151.66, ~171.66] (just outboard of the basket's inner arc
# at R_BASKET_INNER=130). `basket()` subtracts an open radial channel
# at *both* radii on each flagged boundary azimuth. The inner channel
# reaches the inner arc; the outer channel reaches the outer arc.
BASKET_HEX_OVERHANG = 60.0
R_BASKET_OUTER = R_HEX_OUTER + BASKET_HEX_OVERHANG   # 330 = top-deck OD/2
BASKET_WALL_T = 3.0

# Each basket takes its full 360°/N sector. An earlier design shrunk
# this angularly (BASKET_HALF_DELTA) to clear the vertex brackets'
# radial arms that sat on top of the radial rails at z ∈ [0, 6] —
# same Z as the basket floor. With both vertex brackets flipped to
# the underside of the extrusions (z ∈ [-26, -20]), the top surface
# of every frame member is flush at z=0 and the rails/rings
# themselves sit tangent to the basket floor (no volume overlap),
# so each basket can take the whole sector.

# BASKET_FLOOR_Z / BASKET_DEPTH_Z / BASKET_RIM_Z moved to ..params —
# the rim plane is the basket↔ramp sweep interface and ramp_mechanism
# asserts against it.

# Outer-hex through-column cross-section. Must match frame's
# `EXT_SIZE` (2020 aluminum extrusion). Duplicated locally for the
# same reason the bracket sub-assemblies duplicate it — params.py
# doesn't host this constant today.
COLUMN_SIDE = 20.0
COLUMN_CLEARANCE = 1.0   # gap between notch wall and column face
CLAMP_ARM_T = 6.0        # vertex-bracket clamp plate thickness
ACCESS_CHANNEL_W = COLUMN_SIDE + 2 * (CLAMP_ARM_T + COLUMN_CLEARANCE)

# A channel box's radial end face is a flat chord, so pushing it to
# `arc_radius ∓ 1.0` does not clear a cylindrical basket arc across the
# channel's own ±ACCESS_CHANNEL_W/2 width: the arc bulges past the chord
# by the sagitta r − √(r² − (W/2)²), which is 1.116 mm at the r=130 inner
# arc — more than the 1.0 mm that used to be used. The crescent between
# chord and arc then survives the subtraction as a full-height knife-edge
# feather hanging off the channel side wall, 0 mm at |y|=16.09 growing to
# 0.116 mm at |y|=17. Overshoot each arc by its own sagitta plus
# ARC_OVERSHOOT_MARGIN instead. The outer arc's sagitta is only 0.438 mm
# so the old 1.0 mm already cleared it; deriving both keeps the two ends
# honest if ACCESS_CHANNEL_W or either radius moves.
ARC_OVERSHOOT_MARGIN = 0.5
_ARC_HALF_W = ACCESS_CHANNEL_W / 2.0


def _arc_overshoot(radius: float) -> float:
    """Radial depth a chord of half-width `_ARC_HALF_W` must be pushed
    past a `radius` arc for the whole chord to clear it."""
    return (
        radius
        - math.sqrt(radius**2 - _ARC_HALF_W**2)
        + ARC_OVERSHOOT_MARGIN
    )


INNER_ARC_OVERSHOOT = _arc_overshoot(R_BASKET_INNER)
OUTER_ARC_OVERSHOOT = _arc_overshoot(R_BASKET_OUTER)

WALL_MIN = 1.5


# --- Helpers ------------------------------------------------------------

def _radial_access_channel_at(
    azimuth_deg: float,
    radius_start: float,
    radius_end: float,
) -> Part:
    """Full-height radial slot between two radii at `azimuth_deg`.

    The slot is wide enough to swallow the 2020 column footprint and
    either vertex bracket's 6 mm clamp plate. It intentionally reaches
    a basket arc: the printed walls hand off to the clamp face without
    trapping a continuous tower column inside an enclosed floor hole.
    """
    z_margin = 10.0
    height = BASKET_DEPTH_Z + 2 * z_margin
    length = radius_end - radius_start
    radius_mid = (radius_start + radius_end) / 2.0
    vx = radius_mid * math.cos(math.radians(azimuth_deg))
    vy = radius_mid * math.sin(math.radians(azimuth_deg))
    z_center = (BASKET_FLOOR_Z + BASKET_RIM_Z) / 2.0
    return (
        Pos(vx, vy, z_center)
        * Rot(0, 0, azimuth_deg)
        * Box(length, ACCESS_CHANNEL_W, height)
    )


def _arc_prism(
    r_in: float,
    r_out: float,
    half_angle_deg: float,
    z_bot: float,
    z_top: float,
    inset: float = 0.0,
) -> Part:
    """Annular sector with cylindrical-arc inner and outer walls plus
    two radial side walls. Centered on +X.

    `inset` offsets each bounding feature perpendicular-inward by
    that amount — used to carve the hollow interior from an outer
    shell with a uniform wall thickness. Inner arc radius grows by
    `inset`, outer arc radius shrinks by `inset`, radial walls move
    inward perpendicularly.

    Implementation: build a sector trapezoid that straddles the
    annular region, intersect with the outer cylinder of radius
    `r_out - inset` (clips the outer edge to a circular arc), then
    subtract the inner cylinder of radius `r_in + inset` (clips the
    inner edge to a circular arc). The trapezoid's inner edge `x_in`
    sits well inside `r_in + inset` so the subtraction defines the
    inner boundary; `x_in` is also kept outboard of the radial-wall
    intersection point at `x = inset / sin(ha)` so the polygon stays
    non-degenerate.
    """
    ha = math.radians(half_angle_deg)
    r_far = r_out * 2.0
    # Inner edge of the polygon at x = x_in. The polygon's two inner
    # corners sit at (x_in, ±(x_in·tan_ha − dy)), so their distance
    # from the axis is x_in / cos_ha (for inset=0) or slightly less
    # (for inset > 0). For the inner-cylinder subtraction to clean up
    # the polygon's full inner edge, both corners must lie strictly
    # inside `r_in + inset`. Setting `x_in = r_in · cos_ha − 1`
    # places the corners exactly 1 mm inside r_in (always inside
    # `r_in + inset` since inset ≥ 0) and stays comfortably above the
    # radial-wall crossing at `x = inset / sin_ha` for all (ha, inset)
    # in this assembly.
    x_in = r_in * math.cos(ha) - 1.0
    x_far = r_far * math.cos(ha) - inset
    tan_ha = math.tan(ha)
    dy = inset / math.cos(ha)
    pts = [
        (x_in, -(x_in * tan_ha - dy)),
        (x_far, -(x_far * tan_ha - dy)),
        (x_far, +(x_far * tan_ha - dy)),
        (x_in, +(x_in * tan_ha - dy)),
    ]
    sketch = Polygon(*pts, align=None)
    wedge = Pos(0, 0, z_bot) * extrude(sketch, amount=z_top - z_bot)
    cyl_h = (z_top - z_bot) + 2.0
    cyl_z_center = (z_bot + z_top) / 2.0
    outer_cyl = Pos(0, 0, cyl_z_center) * Cylinder(r_out - inset, cyl_h)
    inner_cyl = Pos(0, 0, cyl_z_center) * Cylinder(r_in + inset, cyl_h)
    return (wedge & outer_cyl) - inner_cyl


# --- Part functions -----------------------------------------------------

def basket(
    *,
    half_angle_deg: float,
    notch_left: bool = False,
    notch_right: bool = False,
    notch_middle: bool = False,
) -> Part:
    """Wedge bucket with cylindrical-arc outer wall for one basket
    slot, centered on +X. Closed floor at z=BASKET_FLOOR_Z, open top
    at z=BASKET_RIM_Z.

    `half_angle_deg` = 180/N for the basket's size class (30° at N=6,
    20° at N=9, 15° at N=12, 10° at N=18).

    Notches (all expressed in the basket's local frame; world azimuth
    follows from the Rot applied at placement time):
      - notch_left   → -half_angle_deg (LEFT boundary = smaller world θ)
      - notch_right  → +half_angle_deg (RIGHT boundary = larger world θ)
      - notch_middle → 0° (basket center — used only where V_k lands
                       inside a basket, i.e. N=9 MIDDLE baskets)
    Each flagged notch subtracts two open radial access slots (see
    `_radial_access_channel_at`) from the wedge shell.
    """
    z_bot = BASKET_FLOOR_Z
    z_top = BASKET_RIM_Z
    outer = _arc_prism(
        R_BASKET_INNER, R_BASKET_OUTER, half_angle_deg, z_bot, z_top,
    )
    cavity = _arc_prism(
        R_BASKET_INNER, R_BASKET_OUTER, half_angle_deg,
        z_bot + BASKET_WALL_T, z_top + 1.0,
        inset=BASKET_WALL_T,
    )
    body = outer - cavity
    # Two columns per flagged azimuth: open the inner one to the inner
    # arc and the outer one to the outer arc. The channels overlap the
    # complete column footprints with clearance, so no enclosed floor
    # holes remain.
    for flagged, az in (
        (notch_right, +half_angle_deg),
        (notch_left, -half_angle_deg),
        (notch_middle, 0.0),
    ):
        if not flagged:
            continue
        radial_clearance = COLUMN_SIDE / 2 + COLUMN_CLEARANCE
        body = body - _radial_access_channel_at(
            az,
            R_VERT_OUTER - radial_clearance,
            R_BASKET_OUTER + OUTER_ARC_OVERSHOOT,
        )
        body = body - _radial_access_channel_at(
            az,
            R_BASKET_INNER - INNER_ARC_OVERSHOOT,
            R_VERT_INNER + radial_clearance,
        )
    return body


def col_stub() -> Part:
    """Minimal column-outer-footprint box for standalone basket checks.

    Authoritative basket-vs-column check lives in ../assembly.py
    against the real column() part; this stub only guards against
    baskets encroaching on the column's outer envelope in isolation.
    """
    z_bot = BASKET_FLOOR_Z
    z_top = BASKET_RIM_Z
    z_center = (z_bot + z_top) / 2
    return Pos(0, 0, z_center) * Box(
        COL_OUTER_SIDE, COL_OUTER_SIDE, z_top - z_bot,
    )


# --- Assembly builders ---------------------------------------------------

def build_basket_ring(
    layout,
    size_color_fn: Callable[[int], Color | None] | None = None,
    material_fn: Callable[[str], str | None] | None = None,
) -> Assembly:
    """Basket ring per ``layout`` as a sub-`Assembly` in the
    floor-local frame (floor datum at z=0). ``layout`` is a list of
    ``(center_deg, n_size, notches)`` tuples (``params.FLOOR_LAYOUT``
    shape). ``size_color_fn`` maps a basket's N-size to its color —
    the tower threads ``design_systems.basket_color_of``, a size-keyed
    luminance ramp rather than a name-keyed lookup. ``material_fn``
    follows the frame builders' leaf-name contract; floor identity
    lives in the parent's subtree path.
    """
    a = Assembly()
    for center_deg, n_size, notches in layout:
        name = f"basket_{int(round(center_deg)):03d}_{n_size:02d}"
        a = a.with_part(
            name,
            basket(
                half_angle_deg=180.0 / n_size,
                notch_left="left" in notches,
                notch_right="right" in notches,
                notch_middle="middle" in notches,
            ),
            location=Rot(0, 0, center_deg),
            color=size_color_fn(n_size) if size_color_fn is not None else None,
            material=material_fn(name) if material_fn is not None else None,
        )
    return a


# --- Standalone assembly -------------------------------------------------

assembly = build_basket_ring(FLOOR_LAYOUT).with_part("col_stub", col_stub())

basket_names = [
    f"basket_{int(round(center_deg)):03d}_{n_size:02d}"
    for center_deg, n_size, _notches in FLOOR_LAYOUT
]

for name in basket_names:
    assembly = assembly.assert_no_interference(name, "col_stub")

# Every adjacent pair around the ring, including wrap. Two baskets
# that share a boundary must both cut the notch for the column on
# that boundary (or the column would lie inside their shared wall);
# if either side forgets its notch, the two basket bodies overlap
# through the missing column envelope, which this loop catches.
for a, b in zip(basket_names, basket_names[1:] + basket_names[:1]):
    assembly = assembly.assert_no_interference(a, b)

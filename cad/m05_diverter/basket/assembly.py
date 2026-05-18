"""
M05 hex-tower — basket sub-assembly.

Wedge **buckets** with a flat inner chord, two radial side walls,
and a cylindrical-arc outer wall at R_BASKET_OUTER, sitting on a
rotating-column floor. The floor is a **mixed layout** — baskets
of four sizes (N=6 / 9 / 12 / 18, each with half-angle 180°/N)
placed around the hex per `params.FLOOR_LAYOUT`, with the ramp-
facing N=6 basket straddling +X and the remaining sections filled
by N=9 triples, N=12 pairs, and an N=18 triple + (N=9, N=18) pair
around the back.

Outer arc radius matches the top-deck acrylic disc OD/2 (330 mm),
so all baskets share the same outer cylindrical surface regardless
of N — the floor of buckets reads as one continuous cylindrical
skirt below the disc edge.

Each basket's radial side walls (and in one case, its center) are
notched around the through-columns at vertices V_k = (k+0.5)·60°.
With outer and inner hex aligned, every vertex azimuth carries two
columns — one at R_VERT_OUTER and one at R_VERT_INNER — so each
flagged notch cuts the basket twice along that azimuth (once per
radius). The notch set per basket is declared in `FLOOR_LAYOUT`:
`left`/`right` for boundary columns, `middle` for the V_1 column
that sits inside the N=9 MIDDLE basket's interior.

Floor sits on the shared stage datum (z=0); rim at z=+BASKET_DEPTH_Z.
Basket floor cantilevers past the outer-hex edge but still rests on
the hex outer extrusion top-face strip plus the radial rails, so
brick load drops through the supported zone. Open top for drop-in
delivery, closed bottom; lifts out for emptying. Rack that
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
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build123d import Box, Cylinder, Part, Polygon, Pos, Rot, extrude

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from params import (
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
# The basket's radial side walls lie on the shared (outer = inner)
# hex vertex azimuths and would otherwise pass straight through the
# through-columns at both radii — at R_VERT_OUTER the column body
# spans r ∈ [~301.77, ~321.77] along the vertex azimuth (fully
# inside the basket envelope at r=330); at R_VERT_INNER it spans
# r ∈ [~151.66, ~171.66] (just outboard of the basket's inner
# corner at R_BASKET_INNER=150). `basket()` subtracts a column-
# footprint cutout at *both* radii on each flagged boundary
# azimuth.
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

BASKET_DEPTH_Z = 130.0
BASKET_FLOOR_Z = 0.0                               # shared floor datum
BASKET_RIM_Z = BASKET_FLOOR_Z + BASKET_DEPTH_Z     # 130

# Outer-hex through-column cross-section. Must match frame's
# `EXT_SIZE` (2020 aluminum extrusion). Duplicated locally for the
# same reason the bracket sub-assemblies duplicate it — params.py
# doesn't host this constant today.
COLUMN_SIDE = 20.0
COLUMN_CLEARANCE = 1.0   # gap between notch wall and column face

WALL_MIN = 1.5


# --- Helpers ------------------------------------------------------------

def _column_cutout_at(azimuth_deg: float, radius: float) -> Part:
    """Rotated 20×20 through-cutout centered on a hex vertex at
    (`radius`, `azimuth_deg`), spanning the full basket Z range with
    margin. Removes the column body (and a thin clearance margin)
    from the basket so the radial side wall wraps around the column
    instead of passing through it. Called once per column on each
    flagged boundary azimuth: outer hex vertex at R_VERT_OUTER and
    inner hex vertex at R_VERT_INNER (both share the azimuth).

    Rotation matches frame's `Rot(0, 0, v_angle)` on `outer_vert_k`
    / `inner_vert_k`, so the cutout's faces align with the real
    column's faces.
    """
    side = COLUMN_SIDE + 2 * COLUMN_CLEARANCE
    z_margin = 10.0
    height = BASKET_DEPTH_Z + 2 * z_margin
    vx = radius * math.cos(math.radians(azimuth_deg))
    vy = radius * math.sin(math.radians(azimuth_deg))
    z_center = (BASKET_FLOOR_Z + BASKET_RIM_Z) / 2.0
    return Pos(vx, vy, z_center) * Rot(0, 0, azimuth_deg) * Box(side, side, height)


def _arc_prism(
    r_in: float,
    r_out: float,
    half_angle_deg: float,
    z_bot: float,
    z_top: float,
    inset: float = 0.0,
) -> Part:
    """Angular-sector wedge with flat inner chord, two radial side
    walls, and a cylindrical-arc outer wall at radius `r_out`.
    Centered on +X.

    `inset` offsets each bounding feature perpendicular-inward by
    that amount — used to carve the hollow interior from an outer
    shell with a uniform wall thickness. Inner chord moves outward
    by `inset`, radial walls move inward perpendicularly, outer arc
    radius shrinks by `inset`.

    Implementation: build a sector trapezoid extending well past
    `r_out` and intersect it with a coaxial cylinder of radius
    `r_out - inset`; the cylinder clips the outer edge into a
    circular arc.
    """
    ha = math.radians(half_angle_deg)
    r_far = r_out * 2.0
    x_in = r_in * math.cos(ha) + inset
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
    cyl = Pos(0, 0, cyl_z_center) * Cylinder(r_out - inset, cyl_h)
    return wedge & cyl


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
    Each flagged notch subtracts a rotated column-footprint box (see
    `_column_cutout_at`) from the wedge shell.
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
    # Two columns per flagged azimuth: one at the outer-hex vertex,
    # one at the inner-hex vertex (the hexes are rotation-aligned so
    # both sit on the same vertex azimuth).
    for flagged, az in (
        (notch_right, +half_angle_deg),
        (notch_left, -half_angle_deg),
        (notch_middle, 0.0),
    ):
        if not flagged:
            continue
        body = body - _column_cutout_at(az, R_VERT_OUTER)
        body = body - _column_cutout_at(az, R_VERT_INNER)
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


# --- Assembly -----------------------------------------------------------

assembly = Assembly().add("col_stub", col_stub())

basket_names = []
for center_deg, n_size, notches in FLOOR_LAYOUT:
    name = f"basket_{int(round(center_deg)):03d}_{n_size:02d}"
    basket_names.append(name)
    assembly = assembly.add(
        name,
        basket(
            half_angle_deg=180.0 / n_size,
            notch_left="left" in notches,
            notch_right="right" in notches,
            notch_middle="middle" in notches,
        ),
        location=Rot(0, 0, center_deg),
    )

for name in basket_names:
    assembly = assembly.assert_no_interference(name, "col_stub")

# Every adjacent pair around the ring, including wrap. Two baskets
# that share a boundary must both cut the notch for the column on
# that boundary (or the column would lie inside their shared wall);
# if either side forgets its notch, the two basket bodies overlap
# through the missing column envelope, which this loop catches.
for a, b in zip(basket_names, basket_names[1:] + basket_names[:1]):
    assembly = assembly.assert_no_interference(a, b)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Inspect the geometrically most interesting variant — N=9 MIDDLE,
    # which is the only basket with a column notch cut through its
    # interior rather than at a boundary.
    method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0)
    inspect(
        basket(half_angle_deg=180.0 / 9, notch_middle=True),
        method=method,
        out="outputs",
        name="basket_090_09",
    )

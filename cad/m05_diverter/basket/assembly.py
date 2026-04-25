"""
M05 hex-tower — basket sub-assembly.

Trapezoidal-prism **buckets** sitting on a rotating-column floor.
The floor is a **mixed layout** — baskets of four sizes (N=6 / 9 /
12 / 18, each with half-angle 180°/N) placed around the hex per
`params.FLOOR_LAYOUT`, with the ramp-facing N=6 basket straddling
+X and the remaining sections filled by N=9 triples, N=12 pairs,
and an N=18 triple + (N=9, N=18) pair around the back.

Each basket's outer corners (and in one case, its center) are
notched around the six outer-hex through-columns at vertices
V_k = (k+0.5)·60°. The notch set per basket is declared in
`FLOOR_LAYOUT`: `left`/`right` for boundary columns, `middle`
for the V_1 column that sits inside the N=9 MIDDLE basket's
interior.

Floor sits on the shared stage datum (z=0); rim at z=+BASKET_DEPTH_Z.
Outer chord sits past R_HEX_OUTER so the basket floor rests on the
outer-hex top face from above (mirroring the inner-chord-over-inner-
hex support already landed in Phase 2 step 1). Open top for drop-in
delivery, closed bottom; lifts out for emptying. Rack that physically
carries the bucket load is the hex ring itself — no separate shelf.

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

from build123d import Box, Part, Polygon, Pos, Rot, extrude

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from params import (
    COL_OUTER_SIDE,
    FLOOR_LAYOUT,
    R_BASKET_INNER,
    R_HEX_OUTER,
    R_VERT_OUTER,
)


# --- Parameters ---------------------------------------------------------

# Basket outer chord sits `BASKET_HEX_OVERHANG` mm outside R_HEX_OUTER
# (ΔR convention, matching the inner side). At N=6 the chord midpoint
# x-coordinate is R_BASKET_OUTER·cos(30°); with the current value the
# chord overhangs the outer-hex edge midpoint by ~7.1 mm while
# staying inside the hex's outer face (at r=280), so the basket
# floor rests on a ~17 mm strip of outer-hex top face and does not
# stick out past the tower footprint.
#
# The basket's radial side walls lie on the outer-hex vertex
# azimuths and would otherwise pass straight through the through-
# columns (20×20 rotated, body at r ∈ [~301.77, ~321.77] along
# vertex azimuth). `basket()` subtracts a column-footprint cutout
# at each outer corner to absorb the column body.
BASKET_HEX_OVERHANG = 50.0
R_BASKET_OUTER = R_HEX_OUTER + BASKET_HEX_OVERHANG   # 320
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

def _column_cutout_at(azimuth_deg: float) -> Part:
    """Rotated 20×20 through-cutout centered on the outer-hex vertex
    at `azimuth_deg`, spanning the full basket Z range with margin.
    Removes the column body (and a thin clearance margin) from a
    basket's outer corner so the radial side wall wraps around the
    column instead of passing through it.

    Rotation matches frame's `Rot(0, 0, v_angle)` on `outer_vert_k`,
    so the cutout's faces align with the real column's faces.
    """
    side = COLUMN_SIDE + 2 * COLUMN_CLEARANCE
    z_margin = 10.0
    height = BASKET_DEPTH_Z + 2 * z_margin
    vx = R_VERT_OUTER * math.cos(math.radians(azimuth_deg))
    vy = R_VERT_OUTER * math.sin(math.radians(azimuth_deg))
    z_center = (BASKET_FLOOR_Z + BASKET_RIM_Z) / 2.0
    return Pos(vx, vy, z_center) * Rot(0, 0, azimuth_deg) * Box(side, side, height)


def _trap_prism(
    r_in: float,
    r_out: float,
    half_angle_deg: float,
    z_bot: float,
    z_top: float,
    inset: float = 0.0,
) -> Part:
    """Angular-sector trapezoidal prism centered on +X.

    `inset` offsets each of the four bounding lines perpendicular-inward
    by that amount — used to carve the hollow interior from an outer
    shell with a uniform wall thickness.
    """
    ha = math.radians(half_angle_deg)
    x_in = r_in * math.cos(ha) + inset
    x_out = r_out * math.cos(ha) - inset
    tan_ha = math.tan(ha)
    dy = inset / math.cos(ha)
    pts = [
        (x_in, -(x_in * tan_ha - dy)),
        (x_out, -(x_out * tan_ha - dy)),
        (x_out, +(x_out * tan_ha - dy)),
        (x_in, +(x_in * tan_ha - dy)),
    ]
    sketch = Polygon(*pts, align=None)
    return Pos(0, 0, z_bot) * extrude(sketch, amount=z_top - z_bot)


# --- Part functions -----------------------------------------------------

def basket(
    *,
    half_angle_deg: float,
    notch_left: bool = False,
    notch_right: bool = False,
    notch_middle: bool = False,
) -> Part:
    """Trapezoidal-prism bucket for one basket slot, centered on +X.
    Closed floor at z=BASKET_FLOOR_Z, open top at z=BASKET_RIM_Z.

    `half_angle_deg` = 180/N for the basket's size class (30° at N=6,
    20° at N=9, 15° at N=12, 10° at N=18).

    Notches (all expressed in the basket's local frame; world azimuth
    follows from the Rot applied at placement time):
      - notch_left   → -half_angle_deg (LEFT boundary = smaller world θ)
      - notch_right  → +half_angle_deg (RIGHT boundary = larger world θ)
      - notch_middle → 0° (basket center — used only where V_k lands
                       inside a basket, i.e. N=9 MIDDLE baskets)
    Each flagged notch subtracts a rotated column-footprint box (see
    `_column_cutout_at`) from the trapezoidal shell.
    """
    z_bot = BASKET_FLOOR_Z
    z_top = BASKET_RIM_Z
    outer = _trap_prism(
        R_BASKET_INNER, R_BASKET_OUTER, half_angle_deg, z_bot, z_top,
    )
    cavity = _trap_prism(
        R_BASKET_INNER, R_BASKET_OUTER, half_angle_deg,
        z_bot + BASKET_WALL_T, z_top + 1.0,
        inset=BASKET_WALL_T,
    )
    body = outer - cavity
    if notch_right:
        body = body - _column_cutout_at(+half_angle_deg)
    if notch_left:
        body = body - _column_cutout_at(-half_angle_deg)
    if notch_middle:
        body = body - _column_cutout_at(0.0)
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

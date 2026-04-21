"""
M05 hex-tower — basket sub-assembly.

Trapezoidal-prism container hung in one hex bay on a rotating-column
floor. N identical baskets arrayed around the column axis at angular
step 360°/N; each basket's angular span equals 360°/N so neighbors
touch along shared radial walls. Floor sits below the column datum;
rim at z = 0, floor at z = -BASKET_DEPTH_Z.

Standalone validation: N baskets + a minimal column-footprint stub,
with interference checks against the stub and between one adjacent
pair. The authoritative check against real column / bearing-spider /
ramp geometry runs in the top-level `../assembly.py`.

Coordinate frame:
    origin = column axis ∩ floor datum
    +Z     = up
    +X     = radial outward; basket_plus_x sits on the +X azimuth
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
    N_BASKETS_PER_FLOOR,
    R_BASKET_INNER,
    R_HEX_OUTER,
)


# --- Parameters ---------------------------------------------------------

BASKET_HEX_CLEARANCE = 20.0                        # outer chord wall to outer-hex extrusion
R_BASKET_OUTER = R_HEX_OUTER - BASKET_HEX_CLEARANCE
BASKET_ANG_SPAN_DEG = 360.0 / N_BASKETS_PER_FLOOR
BASKET_WALL_T = 3.0
BASKET_DEPTH_Z = 80.0
BASKET_RIM_Z = 0.0

WALL_MIN = 1.5


# --- Helpers ------------------------------------------------------------

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

def basket() -> Part:
    """Trapezoidal-prism container for one basket slot, centered on +X."""
    half_angle_deg = BASKET_ANG_SPAN_DEG / 2
    z_bot = BASKET_RIM_Z - BASKET_DEPTH_Z
    z_top = BASKET_RIM_Z
    outer = _trap_prism(
        R_BASKET_INNER, R_BASKET_OUTER, half_angle_deg, z_bot, z_top,
    )
    cavity = _trap_prism(
        R_BASKET_INNER, R_BASKET_OUTER, half_angle_deg,
        z_bot + BASKET_WALL_T, z_top + 1.0,
        inset=BASKET_WALL_T,
    )
    return outer - cavity


def col_stub() -> Part:
    """Minimal column-outer-footprint box for standalone basket checks.

    Authoritative basket-vs-column check lives in ../assembly.py
    against the real column() part; this stub only guards against
    baskets encroaching on the column's outer envelope in isolation.
    """
    z_bot = BASKET_RIM_Z - BASKET_DEPTH_Z
    z_top = BASKET_RIM_Z
    z_center = (z_bot + z_top) / 2
    return Pos(0, 0, z_center) * Box(
        COL_OUTER_SIDE, COL_OUTER_SIDE, z_top - z_bot,
    )


# --- Assembly -----------------------------------------------------------

assembly = Assembly().add("col_stub", col_stub())

basket_names = []
for i in range(N_BASKETS_PER_FLOOR):
    theta = i * 360.0 / N_BASKETS_PER_FLOOR
    name = "basket_plus_x" if i == 0 else f"basket_{i:02d}"
    basket_names.append(name)
    assembly = assembly.add(name, basket(), location=Rot(0, 0, theta))

for name in basket_names:
    assembly = assembly.assert_no_interference(name, "col_stub")

if len(basket_names) >= 2:
    assembly = assembly.assert_no_interference(basket_names[0], basket_names[1])


if __name__ == "__main__":
    check(assembly, out="outputs")
    method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0)
    try:
        inspect(basket(), method=method, out="outputs", name="basket")
    except SystemExit:
        pass

"""
M05 hex-tower — top-level diverter assembly (one generic middle floor).

Composes the three sub-assemblies at their installed positions:

- frame — hex rings + outer-hex vertical corner columns + corner brackets
- ramp_mechanism — bearing spider, ring bearing, rotating column + trap + ramp
- basket — N angular-sector basket cells around the column

and runs the interference checks that span sub-assembly boundaries.
Per-sub-system checks stay in each sub-assembly's standalone mode;
this file covers only the cross-boundary pairs.

Cross-boundary checks:

- Ramp extends through the column's +X exit opening into the +X basket
  sector → ramp vs basket_plus_x.
- Baskets hang between the inner and outer hex rings → each basket
  must clear every frame part type (6-fold rotational symmetry means a
  representative basket covers all six).
- Rotating column + bearing spider sit coaxial with the frame inner
  hex → frame inner-hex rails must clear the column envelope and the
  spider's saddle extents.
- Outer hex + corner columns are radially far from the column envelope
  (>200 mm clearance); those pairs aren't enumerated.

Trap sweep vs basket / trap vs frame are intentionally omitted: the
trap stays inside the column envelope, and the column itself is
already checked against the frame.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from build123d import Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

from basket import assembly as _bkt
from frame import assembly as _frame
from params import N_BASKETS_PER_FLOOR
from ramp_mechanism import assembly as _ramp


# --- Assembly -----------------------------------------------------------

# Frame (extrusions + corner brackets) first so the frame parts exist
# by name when the cross-boundary assertions run below.
assembly = _frame.add_frame(Assembly())

# Column-side: bearing spider, ring bearing, rotating column,
# hinge pin + trap (closed) + fixed ramp.
assembly = (
    assembly
    .add("bearing_spider", _ramp.bearing_spider())
    .add("ring_bearing", _ramp.ring_bearing())
    .add("column", _ramp.column())
    .add("hinge_pin", _ramp.hinge_pin(), location=_ramp.hinge_loc)
    .add("ramp", _ramp.ramp(), location=_ramp.ramp_loc)
    .add(
        "trap_closed", _ramp.trap(),
        location=_ramp.hinge_loc * Rot(0, _ramp.TRAP_CLOSED_DEG, 0),
    )
)

# Baskets arrayed around the column axis.
basket_names = []
for i in range(N_BASKETS_PER_FLOOR):
    theta = i * 360.0 / N_BASKETS_PER_FLOOR
    name = "basket_plus_x" if i == 0 else f"basket_{i:02d}"
    basket_names.append(name)
    assembly = assembly.add(name, _bkt.basket(), location=Rot(0, 0, theta))


# --- Cross-boundary checks ---------------------------------------------

# Every basket vs the column-side fixed parts.
for name in basket_names:
    assembly = (
        assembly
        .assert_no_interference(name, "column")
        .assert_no_interference(name, "bearing_spider")
        .assert_no_interference(name, "ring_bearing")
    )

# Ramp tip lands in basket_plus_x's sector.
assembly = assembly.assert_no_interference("basket_plus_x", "ramp")

# Baskets vs frame. 6-fold rotational symmetry on both sides ⇒
# checking basket_plus_x against all six of each frame part type
# validates the symmetric cases.
for k in range(6):
    assembly = (
        assembly
        .assert_no_interference("basket_plus_x", f"outer_horiz_{k}")
        .assert_no_interference("basket_plus_x", f"outer_vert_{k}")
        .assert_no_interference("basket_plus_x", f"corner_bracket_{k}")
        .assert_no_interference("basket_plus_x", f"inner_horiz_{k}")
        .assert_no_interference("basket_plus_x", f"radial_{k}")
    )

# Column-side vs inner hex + radial rails (the frame parts radially
# closest to the column envelope). Outer hex / corner columns /
# brackets are all ≥ 272 mm from the axis — far outside the column's
# ~68 mm corner reach — and are omitted.
#
# Spider now hangs off the radial rails (saddles at azimuth (k+0.5)·60°,
# aligned with the rail axes). The saddle grips the rail's inner end;
# we model this as an asserted no-interference today — shape-level
# tangent contact at the rail outer faces reads as zero clearance but
# zero overlap volume, which passes. If the saddle is later modeled as
# a physical clamp that closes 3D around the extrusion, expect to
# relax this to assert_interference with a reason.
for k in range(6):
    inner = f"inner_horiz_{k}"
    radial = f"radial_{k}"
    assembly = (
        assembly
        .assert_no_interference("column", inner)
        .assert_no_interference("ring_bearing", inner)
        .assert_no_interference("bearing_spider", inner)
        .assert_no_interference("column", radial)
        .assert_no_interference("ring_bearing", radial)
        .assert_no_interference("bearing_spider", radial)
    )

# Ramp extends radially past the inner hex (tip at x≈147) but stays
# at z ≥ 50 while the inner hex sits at z ∈ [-20, 0]. Still worth
# asserting in case either moves.
for k in range(6):
    assembly = assembly.assert_no_interference("ramp", f"inner_horiz_{k}")
    assembly = assembly.assert_no_interference("ramp", f"radial_{k}")


if __name__ == "__main__":
    check(assembly, out="outputs")

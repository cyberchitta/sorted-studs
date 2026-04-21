"""
M05 hex-tower — top-level diverter assembly.

Composes the column-side sub-assembly (ramp_mechanism) with the basket
sub-assembly at their installed locations and runs interference checks
that span sub-assembly boundaries. Per-sub-system parts are validated
inside each sub-assembly's own standalone mode; this file doesn't
duplicate those checks, only the cross-boundary ones.

Key cross-boundary check: the ramp extends out through the column's
+X exit opening into the basket sector, so ramp-vs-basket_plus_x must
pass with real geometry (not the minimal col_stub used in basket's
standalone mode). Baskets as a whole must also clear the column and
bearing-spider outer envelope.

Trap sweep vs basket is intentionally omitted: the trap stays inside
the column envelope and the baskets sit entirely outside it, so any
such intersection would already have been caught by trap-vs-column or
basket-vs-column in the respective sub-assemblies.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from build123d import Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

from basket import assembly as _bkt
from params import N_BASKETS_PER_FLOOR
from ramp_mechanism import assembly as _ramp


# --- Assembly -----------------------------------------------------------

assembly = (
    Assembly()
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

basket_names = []
for i in range(N_BASKETS_PER_FLOOR):
    theta = i * 360.0 / N_BASKETS_PER_FLOOR
    name = "basket_plus_x" if i == 0 else f"basket_{i:02d}"
    basket_names.append(name)
    assembly = assembly.add(name, _bkt.basket(), location=Rot(0, 0, theta))

# Cross-boundary checks: every basket vs column-side fixed parts.
for name in basket_names:
    assembly = (
        assembly
        .assert_no_interference(name, "column")
        .assert_no_interference(name, "bearing_spider")
        .assert_no_interference(name, "ring_bearing")
    )

# Ramp tip is in the +X basket's sector — the single check that this
# whole top-level exists to run.
assembly = assembly.assert_no_interference("basket_plus_x", "ramp")


if __name__ == "__main__":
    check(assembly, out="outputs")

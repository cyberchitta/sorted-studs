"""
M05 hex-tower — single-floor cross-boundary assembly (one generic middle floor).

Fast-iteration variant of the top-level: one middle floor at world z=0,
with the floor-below trap-open as a stand-in stub. The full 4-floor
tower lives in `assembly.py`; use this file when iterating on a single
floor's geometry without paying the multi-floor check cost.

Rides the same sub-module builders as the tower, with every subtree
at identity (floor-local == world here):

- `rails` / `columns` — frame horizontals + through-columns
- `bearing` — bearing spider + ring bearing
- `rotating` — column + hinge pin + ramp + trap-closed (no trap-open:
  that envelope only exists to check against an upper neighbour)
- `baskets` — the full mixed `FLOOR_LAYOUT`
- `lower_trap_open` — root-level stand-in for the floor below's trap
  at max-open, mapped into this floor's frame via -FLOOR_H

and runs the interference checks that span sub-assembly boundaries.
Per-sub-system checks stay in each sub-assembly's standalone mode;
this file covers only the cross-boundary pairs.

Cross-boundary checks:

- Ramp extends through the column's +X exit opening into the +X basket
  sector → ramp vs `baskets.basket_000_06` (the N=6 basket straddling +X).
- Baskets hang between the inner and outer hex rings → every basket
  must clear every frame part type. The mixed floor (see
  `params.FLOOR_LAYOUT`) breaks rotational symmetry on the basket
  side, so all 11 baskets × 6 frame copies per part type are
  enumerated.
- Rotating column + bearing spider sit coaxial with the frame inner
  hex → frame inner-hex rails must clear the column envelope and the
  spider's saddle extents.
- Outer hex + corner columns are radially far from the column envelope
  (>200 mm clearance); those pairs aren't enumerated.

Trap sweep vs basket / trap vs frame are intentionally omitted: the
trap stays inside the column envelope, and the column itself is
already checked against the frame.
"""


from build123d import Location, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

from .basket import assembly as _bkt
from .design_systems import (
    DESIGN_SYSTEM,
    basename_of,
    basket_color_of,
    color_of,
)
from .materials import material_of
from .frame import assembly as _frame
from .params import FLOOR_H, FLOOR_LAYOUT
from .ramp_mechanism import assembly as _ramp


# --- Assembly -----------------------------------------------------------

assembly = (
    Assembly()
    .with_subassembly(
        "rails",
        _frame.build_floor_horizontals(
            color_fn=color_of, material_fn=material_of,
        ),
    )
    .with_subassembly(
        "columns",
        _frame.build_through_columns(
            color_fn=color_of, material_fn=material_of,
        ),
    )
    .with_subassembly(
        "bearing",
        _ramp.build_column_side_static(
            "middle", color_fn=color_of, material_fn=material_of,
        ),
    )
    .with_subassembly(
        "rotating",
        _ramp.build_column_side_rotating(
            "middle", with_trap_open=False,
            color_fn=color_of, material_fn=material_of,
        ),
    )
    .with_subassembly(
        "baskets",
        _bkt.build_basket_ring(
            FLOOR_LAYOUT,
            size_color_fn=basket_color_of, material_fn=material_of,
        ),
    )
)

# Stand-in for the floor-below's trap at max-open (60°), mapped into
# this floor's frame via -FLOOR_H. Lets the single-floor model show the
# full steady-state stack-up before the multi-floor assembly is built.
assembly = assembly.with_part(
    "lower_trap_open", _ramp.trap(),
    location=Location(
        (_ramp.TRAP_HINGE_X, 0, _ramp.TRAP_HINGE_Z - FLOOR_H)
    ) * Rot(0, _ramp.TRAP_OPEN_DEG, 0),
    color=color_of("lower_trap_open"),
    material=material_of("lower_trap_open"),
)

basket_names = [
    f"baskets.basket_{int(round(center_deg)):03d}_{n_size:02d}"
    for center_deg, n_size, _notches in FLOOR_LAYOUT
]


# --- Design-system coverage --------------------------------------------
#
# Every named part this single-floor view adds must appear in the
# active design system's tag dict (``None`` allowed for hardware —
# kept at material's intrinsic color). Attributes key on the basename
# (leaf segment of the qualified path); identity stays the full path.
# A part added without a tag fails here instead of silently rendering
# off-palette. Swapping ``DESIGN_SYSTEM_NAME`` (in
# ``cad/design_systems.py``) to a palette missing a tag also fails
# here, so the swap surface is loud.
_untagged = sorted(
    {
        p.name for p in assembly.placed_parts
        if basename_of(p.name) not in DESIGN_SYSTEM.tag_of
    }
)
assert not _untagged, (
    f"{len(_untagged)} m05 single-floor part(s) missing a tag for "
    f"design system '{DESIGN_SYSTEM.name}' in design_systems.py: "
    f"{_untagged}"
)


# --- Cross-boundary checks ---------------------------------------------

# Every basket vs the column-side fixed parts. lower_trap_open is a
# stand-in for the floor below, not a physical part on this floor —
# but it's modeled so we can assert basket walls and frame members
# still clear it.
for name in basket_names:
    assembly = (
        assembly
        .assert_no_interference(name, "rotating.column")
        .assert_no_interference(name, "bearing.bearing_spider")
        .assert_no_interference(name, "bearing.ring_bearing")
        .assert_no_interference(name, "lower_trap_open")
    )

# Ramp tip lands in the N=6 basket that straddles +X.
assembly = assembly.assert_no_interference("baskets.basket_000_06", "rotating.ramp")

# Sweep clearance over the basket rim plane (all azimuths, no sweep —
# Z-invariant datum plane) propagates from
# `ramp_mechanism.build_column_side_rotating` as
# `rotating.ramp_sweep_over_basket_rim`; the mirrored parametric
# assert this file carried is gone.

# Every basket vs every frame copy of each part type. The frame has
# 6-fold rotational symmetry but the mixed basket layout doesn't, so
# every basket is checked against all six frame copies. This is also
# the authoritative test of the notch cutouts against the real
# through-columns at both radii (outer_vert_k / inner_vert_k) — if a
# notch is missing or misaligned, basket-vs-*_vert interference
# fails here.
for name in basket_names:
    for k in range(6):
        assembly = (
            assembly
            .assert_no_interference(name, f"rails.outer_horiz_{k}")
            .assert_no_interference(name, f"columns.outer_vert_{k}")
            .assert_no_interference(name, f"columns.inner_vert_{k}")
            .assert_no_interference(name, f"rails.outer_vertex_bracket_{k}")
            .assert_no_interference(name, f"rails.inner_horiz_{k}")
            .assert_no_interference(name, f"rails.inner_vertex_bracket_{k}")
            .assert_no_interference(name, f"rails.radial_inner_{k}")
            .assert_no_interference(name, f"rails.radial_outer_{k}")
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
    inner = f"rails.inner_horiz_{k}"
    radial_in = f"rails.radial_inner_{k}"
    radial_out = f"rails.radial_outer_{k}"
    assembly = (
        assembly
        .assert_no_interference("rotating.column", inner)
        .assert_no_interference("bearing.ring_bearing", inner)
        .assert_no_interference("bearing.bearing_spider", inner)
        .assert_no_interference("rotating.column", radial_in)
        .assert_no_interference("rotating.column", radial_out)
        .assert_no_interference("bearing.ring_bearing", radial_in)
        .assert_no_interference("bearing.ring_bearing", radial_out)
        # Spider saddle clamps the inboard rail segment's inner end
        # (r ∈ [90, 110]); the outer segment is well past the spider
        # hub. Tangent-contact on the saddle reads as zero clearance
        # / zero overlap volume — passes today; will tighten when
        # the saddle is modeled as a 3D clamp.
        .assert_no_interference("bearing.bearing_spider", radial_in)
        .assert_no_interference("bearing.bearing_spider", radial_out)
        # Lower-floor trap-open intrusion vs the frame members radially
        # closest to the axis. Inner hex + rails sit at z ∈ [-20, 0]
        # (top face at z=0); the intrusion sweeps up through the axis
        # region and must clear them.
        .assert_no_interference("lower_trap_open", inner)
        .assert_no_interference("lower_trap_open", radial_in)
        .assert_no_interference("lower_trap_open", radial_out)
    )

# Ramp extends radially past the inner hex (tip at r≈146.4) but
# stays at z ≥ 50 while the inner hex sits at z ∈ [-20, 0]. Inner
# through-columns at r=R_VERT_INNER≈161.66 (inner face at 151.66)
# clear the tip by ≈5.3 mm radially at any column rotation; the
# nearest pair (V_0 / V_5) sits 30° off the +X delivery azimuth, so
# the ramp tip is far in plan when the ramp is aimed at a basket
# center. Still worth asserting in case any of these radii move.
for k in range(6):
    assembly = assembly.assert_no_interference("rotating.ramp", f"rails.inner_horiz_{k}")
    assembly = assembly.assert_no_interference("rotating.ramp", f"rails.radial_inner_{k}")
    assembly = assembly.assert_no_interference("rotating.ramp", f"rails.radial_outer_{k}")
    assembly = assembly.assert_no_interference("rotating.ramp", f"columns.inner_vert_{k}")


if __name__ == "__main__":
    check(assembly, out="outputs")

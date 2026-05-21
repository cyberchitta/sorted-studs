"""
M05 hex-tower — single-floor cross-boundary assembly (one generic middle floor).

Fast-iteration variant of the top-level: one middle floor at world z=0,
with the floor-below trap-open as a stand-in stub. The full 4-floor
tower lives in `assembly.py`; use this file when iterating on a single
floor's geometry without paying the multi-floor check cost.

Composes the three sub-assemblies at their installed positions:

- frame — hex rings + outer-hex vertical corner columns + vertex brackets
- ramp_mechanism — bearing spider, ring bearing, rotating column + trap + ramp
- basket — N angular-sector basket cells around the column

and runs the interference checks that span sub-assembly boundaries.
Per-sub-system checks stay in each sub-assembly's standalone mode;
this file covers only the cross-boundary pairs.

Cross-boundary checks:

- Ramp extends through the column's +X exit opening into the +X basket
  sector → ramp vs `basket_000_06` (the N=6 basket straddling +X).
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

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from build123d import Location, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

from basket import assembly as _bkt
from design_systems import DESIGN_SYSTEM, basename_of, color_of
from materials import material_of
from frame import assembly as _frame
from params import FLOOR_H, FLOOR_LAYOUT
from ramp_mechanism import assembly as _ramp


# --- Assembly -----------------------------------------------------------

# Frame (extrusions + corner brackets) first so the frame parts exist
# by name when the cross-boundary assertions run below.
assembly = _frame.add_frame(
    Assembly(), color_fn=color_of, material_fn=material_of,
)


def _add(a: Assembly, name: str, part, **kwargs) -> Assembly:
    """Wrap `.with_part(...)` so every column-side / basket call site picks up
    the active design system's color + material without repeating the
    boilerplate."""
    return a.with_part(
        name, part, color=color_of(name), material=material_of(name), **kwargs,
    )


# Column-side: bearing spider, ring bearing, rotating column,
# hinge pin + trap (closed) + fixed ramp.
assembly = _add(assembly, "bearing_spider", _ramp.bearing_spider())
assembly = _add(assembly, "ring_bearing", _ramp.ring_bearing())
assembly = _add(assembly, "column", _ramp.column())
assembly = _add(assembly, "hinge_pin", _ramp.hinge_pin(), location=_ramp.hinge_pin_loc)
assembly = _add(assembly, "ramp", _ramp.ramp(), location=_ramp.ramp_loc)
assembly = _add(
    assembly, "trap_closed", _ramp.trap(),
    location=_ramp.hinge_loc * Rot(0, _ramp.TRAP_CLOSED_DEG, 0),
)
# Stand-in for the floor-below's trap at max-open (60°), mapped into
# this floor's frame via -FLOOR_H. Lets the single-floor model show the
# full steady-state stack-up before the multi-floor assembly is built.
assembly = _add(
    assembly, "lower_trap_open", _ramp.trap(),
    location=Location(
        (_ramp.TRAP_HINGE_X, 0, _ramp.TRAP_HINGE_Z - FLOOR_H)
    ) * Rot(0, _ramp.TRAP_OPEN_DEG, 0),
)

# Mixed-layout baskets (see params.FLOOR_LAYOUT).
basket_names = []
for center_deg, n_size, notches in FLOOR_LAYOUT:
    name = f"basket_{int(round(center_deg)):03d}_{n_size:02d}"
    basket_names.append(name)
    assembly = _add(
        assembly, name,
        _bkt.basket(
            half_angle_deg=180.0 / n_size,
            notch_left="left" in notches,
            notch_right="right" in notches,
            notch_middle="middle" in notches,
        ),
        location=Rot(0, 0, center_deg),
    )


# --- Design-system coverage --------------------------------------------
#
# Every named part this single-floor view adds must appear in the
# active design system's tag dict (``None`` allowed for hardware —
# kept at material's intrinsic color). A part added without a tag
# fails here instead of silently rendering off-palette. Swapping
# ``DESIGN_SYSTEM_NAME`` (in ``cad/design_systems.py``) to a palette
# missing a tag also fails here, so the swap surface is loud.
_untagged = sorted(
    {p.name for p in assembly.parts if basename_of(p.name) not in DESIGN_SYSTEM.tag_of}
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
        .assert_no_interference(name, "column")
        .assert_no_interference(name, "bearing_spider")
        .assert_no_interference(name, "ring_bearing")
        .assert_no_interference(name, "lower_trap_open")
    )

# Ramp tip lands in the N=6 basket that straddles +X.
assembly = assembly.assert_no_interference("basket_000_06", "ramp")

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
            .assert_no_interference(name, f"outer_horiz_{k}")
            .assert_no_interference(name, f"outer_vert_{k}")
            .assert_no_interference(name, f"inner_vert_{k}")
            .assert_no_interference(name, f"outer_vertex_bracket_{k}")
            .assert_no_interference(name, f"inner_horiz_{k}")
            .assert_no_interference(name, f"inner_vertex_bracket_{k}")
            .assert_no_interference(name, f"radial_inner_{k}")
            .assert_no_interference(name, f"radial_outer_{k}")
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
    radial_in = f"radial_inner_{k}"
    radial_out = f"radial_outer_{k}"
    assembly = (
        assembly
        .assert_no_interference("column", inner)
        .assert_no_interference("ring_bearing", inner)
        .assert_no_interference("bearing_spider", inner)
        .assert_no_interference("column", radial_in)
        .assert_no_interference("column", radial_out)
        .assert_no_interference("ring_bearing", radial_in)
        .assert_no_interference("ring_bearing", radial_out)
        # Spider saddle clamps the inboard rail segment's inner end
        # (r ∈ [90, 110]); the outer segment is well past the spider
        # hub. Tangent-contact on the saddle reads as zero clearance
        # / zero overlap volume — passes today; will tighten when
        # the saddle is modeled as a 3D clamp.
        .assert_no_interference("bearing_spider", radial_in)
        .assert_no_interference("bearing_spider", radial_out)
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
    assembly = assembly.assert_no_interference("ramp", f"inner_horiz_{k}")
    assembly = assembly.assert_no_interference("ramp", f"radial_inner_{k}")
    assembly = assembly.assert_no_interference("ramp", f"radial_outer_{k}")
    assembly = assembly.assert_no_interference("ramp", f"inner_vert_{k}")


if __name__ == "__main__":
    check(assembly, out="outputs")

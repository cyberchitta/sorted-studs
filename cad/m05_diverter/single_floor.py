"""
M05 hex-tower — single-floor cross-boundary assembly (one generic middle floor).

Fast-iteration variant of the top-level: one middle floor at world z=0,
with the floor-below trap servo as a stand-in stub. The full 4-floor
tower lives in `assembly.py`; use this file when iterating on a single
floor's geometry without paying the multi-floor check cost.

Rides the same sub-module builders as the tower, with every subtree
at identity (floor-local == world here):

- `rails` / `columns` — frame horizontals + through-columns
- `bearing` — bearing spider + ring bearing
- `rotating` — column + hinge pin + ramp + trap (hanging) + trap servo
- `baskets` — the layout of the floor named by `floor` (default 0,
  the base floor's reference layout). The tower's four floors differ
  on the basket side only, so sweeping `floor` over 0..3 covers every
  layout against the same column-side fixture —
  `floor_layout_sweep.py`.
- `lower_trap_servo` — root-level stand-in for the floor below's trap
  servo, mapped into this floor's frame via -FLOOR_H

and runs the interference checks that span sub-assembly boundaries.
Per-sub-system checks stay in each sub-assembly's standalone mode;
this file covers only the cross-boundary pairs.

Cross-boundary checks:

- Ramp extends through the column's +X exit opening into the +X basket
  sector → ramp vs `baskets.basket_000_06` (the N=6 basket straddling +X).
- Baskets hang between the inner and outer hex rings → every basket
  must clear every frame part type. The mixed floor (see
  `params.FLOOR_LAYOUTS`) breaks rotational symmetry on the basket
  side, so all 12 baskets × 6 frame copies per part type are
  enumerated.
- Rotating column + bearing spider sit coaxial with the frame inner
  hex → frame inner-hex rails must clear the column envelope and the
  spider's saddle extents.
- Outer hex + corner columns are radially far from the column envelope
  (>200 mm clearance); those pairs aren't enumerated.

Trap sweep vs basket / trap vs frame are intentionally omitted: the
trap stays inside the column envelope, and the column itself is
already checked against the frame. The trap servo does leave the
column (outside its +Y wall, r ≤ ≈ 91), so it is checked against the
baskets and the inner frame members below.
"""


from functools import cache


from build123d import Axis

from cad_khana.mechanism.assembly import Assembly, RevoluteJoint
from cad_khana.mechanism.motion import Motion

from ..design_systems import DEFAULT_SCHEMA, paint
from .basket import assembly as _bkt
from .design_systems import color_for, design_system
from .materials import material_of
from .frame import assembly as _frame
from .params import FLOOR_LAYOUTS
from .ramp_mechanism import assembly as _ramp


# --- Assembly -----------------------------------------------------------
# Cached because the degenerate ``assembly`` memo below and any
# composing parent (``cad/assembly.py``, ``cad/render.py``) each ask
# for the same schema, and the build is seconds of OCCT work. Replaces
# import-time memoization: the memo line is now free, and a parent
# calling the builder gets the same object rather than rebuilding.
# Safe to share — ``Assembly`` is a frozen dataclass and every
# ``with_*`` returns a copy.
#
# Call it with the schema **explicit** everywhere, including the memo
# below: ``@cache`` keys on the call signature, so ``build_x()`` and
# ``build_x(DEFAULT_SCHEMA)`` are two entries that build identical
# geometry twice. That mistake is invisible — same output, double the
# work — so ``cache_info().misses`` is the only thing that shows it.
@cache
def build_single_floor(
    schema: str = DEFAULT_SCHEMA, floor: int = 0,
) -> Assembly:
    """One generic middle floor carrying floor ``floor``'s basket
    layout, plus a stand-in for the floor below, painted under
    ``schema``.

    ``floor`` selects the basket layout only; the column side stays the
    ``middle`` role at identity, which is the conservative fixture (the
    base floor has no floor below it, and gets the lower-servo intrusion
    checked anyway).

    The cross-boundary check matrix this file exists for — every basket
    against every frame copy, the column side against the inner hex and
    radials, and the lower floor's trap-open intrusion — is declared
    here; ``paint`` runs last and carries the coverage check the
    module-level assert used to do.
    """
    assembly = (
        Assembly()
        .with_subassembly(
            "rails",
            _frame.build_floor_horizontals(
                material_fn=material_of,
            ),
        )
        .with_subassembly(
            "columns",
            _frame.build_through_columns(
                material_fn=material_of,
            ),
        )
        .with_subassembly(
            "bearing",
            _ramp.build_column_side_static(
                "middle", material_fn=material_of,
            ),
        )
        .with_subassembly(
            "rotating",
            _ramp.build_column_side_rotating(
                "middle", material_fn=material_of,
            ),
            # Same DOF the tower drives as ``rotating_stack`` — one
            # revolute about world Z. Declared here so the motion below
            # has a joint to sweep; at angle 0 the geometry is identical
            # to the flat form this replaced.
            joint=RevoluteJoint(axis=Axis.Z, angle_deg=0.0),
        )
        .with_subassembly(
            "baskets",
            _bkt.build_basket_ring(
                FLOOR_LAYOUTS[floor],
                material_fn=material_of,
            ),
        )
    )

    # Stand-in for the floor-below's trap servo, mapped into this floor's
    # frame via -FLOOR_H — the one lower-floor rotating part that rises
    # past the joint plane outside the column. Lets the single-floor model
    # show the full steady-state stack-up before the multi-floor assembly
    # is built.
    assembly = assembly.with_part(
        "lower_trap_servo", _ramp.trap_servo(),
        location=_ramp.lower_servo_loc,
        material=material_of("lower_trap_servo"),
    )

    # --- Cross-boundary checks ---------------------------------------------

    # Every basket vs the column-side fixed parts. lower_trap_servo is a
    # stand-in for the floor below, not a physical part on this floor —
    # but it's modeled so we can assert basket walls and frame members
    # still clear it.
    assembly = assembly.assert_no_interference_between("baskets", [
        "rotating.column",
        "bearing.bearing_spider",
        "bearing.ring_bearing",
        "rotating.trap_servo",
        "lower_trap_servo",
    ])

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
    assembly = (
        assembly
        .assert_no_interference_between("baskets", "rails")
        .assert_no_interference_between("baskets", "columns")
    )

    # Column-side vs inner hex + radial rails (the frame parts radially
    # closest to the column envelope). Outer hex / corner columns /
    # brackets are all ≥ 272 mm from the axis — far outside the column's
    # ~68 mm corner reach — and are omitted.
    #
    # Spider hangs off the radial rails (saddles at azimuth (k+0.5)·60°,
    # on the inboard segment's inner end, r ∈ [100, 120]). It is held clear
    # of them only — nothing yet asserts it rests on them. That each basket
    # rests on its sector's two hex edges is the tower's
    # `_build_floor_static`, for every floor's layout.
    #
    # Lower-floor servo vs the frame members radially closest to
    # the axis. Inner hex + rails sit at z ∈ [-20, 0] (top face at
    # z=0); the servo sweeps the ring r ≈ 55..91 at z ≈ 13..26 and
    # must clear them. This floor's own servo, same band a floor higher.
    inner_rails = [
        f"rails.{kind}_{k}"
        for k in range(6)
        for kind in ("inner_horiz", "radial_inner", "radial_outer")
    ]
    assembly = assembly.assert_no_interference_between([
        "rotating.column",
        "bearing.ring_bearing",
        "bearing.bearing_spider",
        "lower_trap_servo",
        "rotating.trap_servo",
    ], inner_rails)

    # Ramp extends radially past the inner hex (tip corners at r≈146.7) but
    # stays at z ≥ 50 while the inner hex sits at z ∈ [-20, 0]. Inner
    # through-columns at r=R_VERT_INNER≈161.66 (inner face at 151.66)
    # are held off by a measured gap, not just no-overlap: the ramp's
    # plate and walls are cut to the length that puts their outer top
    # corners on RAMP_REACH_R, so the gap bottoms out at
    # RAMP_COLUMN_CLEARANCE when a corner points at a column. The wall
    # corners cut these columns at 14–20° + 60°·k while the no-overlap
    # form stood green at azimuth 0.
    assembly = assembly.assert_no_interference_between(
        ["rotating.ramp"], inner_rails,
    )
    for k in range(6):
        assembly = assembly.assert_distance(
            "rotating.ramp", f"columns.inner_vert_{k}",
            min_mm=_ramp.RAMP_COLUMN_CLEARANCE,
            name=f"ramp_inner_column_gap_{k}",
        )

    # Every claim above is held over the stack turn, not just at the
    # as-built azimuth. Nothing composes this file (it is a leaf root,
    # not a unit the tower or `cad/assembly.py` builds from), so the
    # motion costs its ~20 s here and nowhere else — which is why it
    # can live in the factory rather than on a separate check target.
    # `step=2` bounds how far the sampled ramp/column gap can read above
    # its true minimum: a 1° miss on a corner at r≈146.7 moves it
    # ≈0.02 mm radially toward the face.
    assembly = assembly.with_motion(
        Motion.over_joint("stack_turn", "rotating", 0, 358, step=2)
    )

    return paint(assembly, design_system(schema), color_for=color_for)


# Degenerate memoized master — the base floor's layout. Note the
# privileged-stem rule: this file is not called ``assembly.py``, so
# ``khana check`` writes to ``outputs/single_floor/``, not
# ``outputs/``. The other three layouts are checked by
# ``floor_layout_sweep.py``; the CLI cannot address them, because
# ``floor`` is an argument.
assembly = build_single_floor(DEFAULT_SCHEMA, 0)

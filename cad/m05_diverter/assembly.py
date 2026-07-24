"""
M05 hex-tower — top-level 4-floor diverter assembly.

Composes four floors with roles `[base, middle, middle, top]` stacked
at world z = `[0, FLOOR_H, 2·FLOOR_H, 3·FLOOR_H]`. Bricks enter above
the top floor and fall down through per-floor traps that divert them
into the floor's baskets (or release them through to the floor below).

The tree is rigid-body-major: everything stationary hangs off the
root, and the one degree of freedom — the column stack that revolves
as a single rigid body — is the `rotating_stack` subtree with a
revolute joint about world Z. Floors are instanced from shared
builders (role differences are builder parameters) and placed by one
`Pos(0, 0, idx·FLOOR_H)` transform each:

- `columns` — the twelve through-columns spanning all floors as
  continuous 2020s (`frame.build_through_columns`).
- `f{idx}` — one static floor: `rails` (hex rings, radials, vertex
  brackets; `frame.build_floor_horizontals`), `baskets`
  (`basket.build_basket_ring` per the floor's layout), and `bearing`
  (spider + ring bearing; `ramp_mechanism.build_column_side_static`).
- `rotating_stack.f{idx}` — the floor's revolving parts: column +
  hinge pin + ramp + trap(s)
  (`ramp_mechanism.build_column_side_rotating`). Roles differ only at
  the column bottom: base has a bottom collar (motor + ring-gear
  interface); middle/top have a bottom face flange that mates with
  the column below at the joint plane (z_world = N·220 + 273). Each
  column carries a rotating collar at z_local 220..225 that the
  bearing of the floor *above* grips.
- `drive_train` — off-axis ring gear / pinion / motor / motor mount
  below the base floor's rail datum (`base_floor.build_drive_train`).
  The ring gear bolts to the base column's bottom collar; no central
  drive shaft — Lego falls through the column axis.
- `top_acrylic` — clear disc capping the tower (root part).

Part identity is the dotted tree path
(`rotating_stack.f0.f0_base__column`); leaf names still carry the
`f{idx}_{role}__` floor prefix pending the naming migration, and
attribute maps (colors, materials, detail targets) key on the
basename via `design_systems.basename_of`.

Cross-floor checks (the value the tower adds beyond `single_floor.py`):

- Upper floor's bearing+spider engages lower floor's column rotating
  collar (z_local 220..225 of lower = z_local 0..5 of upper). Grease
  film clearance — `assert_no_interference` guards the radial gap.
- Lower column top (z = 273 in lower local) vs upper column bottom
  (z = 53 in upper local) tangent at the joint plane. The two flanges
  meet face-to-face; assert_no_interference allows that contact.
- Lower floor's open-trap envelope (max-open free end at z ≈ 46 mm
  above the upper floor's bracket) vs upper floor's column-bottom
  stack — the trap-open zone must thread cleanly between the bearing
  (z ≤ 5) and this floor's column-bottom (z ≥ 53).

Intra-floor pairs (basket-vs-frame, ramp-vs-basket, column-side self-
clearance) are validated by `single_floor.py` and the standalone
sub-assemblies; the geometry is z-translated only, so per-floor
enumeration here would be redundant.

The composition is factored into `build_tower(floor_plan, ...)` so an
exhibit variant (`exhibit_assembly.py`) can reuse every per-floor
helper with a reduced plan + basket layout + no drive train / top
deck. The production assembly is `build_tower(FLOOR_PLAN)`.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from build123d import Axis, Color, Cylinder, Pos

from cad_khana.mechanism.assembly import Assembly, RevoluteJoint
from cad_khana.mechanism.check import check

from base_floor import assembly as _base
from basket import assembly as _bkt
from design_systems import (
    DESIGN_SYSTEM,
    basename_of,
    basket_color_of,
    color_of,
)
from materials import material_of
from frame import assembly as _frame
from params import FLOOR_H, FLOOR_LAYOUT
from ramp_mechanism import assembly as _ramp


# --- Ramp sweep clearance (cross-sub-assembly) ---------------------------
#
# The ramp rotates with the column, sweeping over every basket's radial
# side walls (which stand at BASKET_RIM_Z). Its lowest swept point — the
# tip end's bottom corner — must clear the rim plane at all azimuths.
# Parametric, because the geometric ramp-vs-basket check in
# `single_floor.py` only covers the one basket the static model aims at.
RAMP_SWEEP_CLEARANCE = 5.0
assert _ramp.RAMP_TIP_BOTTOM_Z >= _bkt.BASKET_RIM_Z + RAMP_SWEEP_CLEARANCE, (
    f"Ramp tip bottom (z = {_ramp.RAMP_TIP_BOTTOM_Z:.1f}) dips within "
    f"{RAMP_SWEEP_CLEARANCE} mm of the basket rim plane "
    f"(z = {_bkt.BASKET_RIM_Z:.1f}); the rotating ramp would strike "
    f"basket side walls. Rebalance RAMP_DEG / RAMP_L / "
    f"RAMP_DROP_BELOW_HINGE / TRAP_HINGE_Z."
)


# --- Top deck ----------------------------------------------------------
#
# Clear-acrylic disc capping the tower. Hopper / rotary-feed and
# ceiling-sleeve mounting are deferred — see NEXT_STEPS "Top deck —
# open detail".
TOP_DECK_OD = 660.0       # circumscribes column outer corners (~322 mm) + ~8 mm clearance
TOP_DECK_T = 10.0
TOP_DECK_APERTURE_D = 150.0  # clears top column's rotating collar (OD 138 at z 880..885)


# --- Floor plan --------------------------------------------------------

# (idx, role) — base at idx 0 (world z=0, the bottom). Bricks enter
# above the top floor (idx 3, world z=3·FLOOR_H) and fall in -Z.
FLOOR_PLAN = [
    (0, "base"),
    (1, "middle"),
    (2, "middle"),
    (3, "top"),
]

# Through-column top z for the production plan. One extra FLOOR_H
# above the top floor for the brick-entry zone.
TOP_VERT_Z = 4 * FLOOR_H


def _floor_prefix(idx: int, role: str) -> str:
    return f"f{idx}_{role}__"


# --- Per-floor builders ------------------------------------------------

def _build_floor_static(role: str, prefix: str, basket_layout) -> Assembly:
    """One floor's static side as a sub-`Assembly` in the floor-local
    frame (rail-top datum at z=0): `rails` (hex rings + radials +
    vertex brackets), `baskets` (per the supplied layout, colored by
    the size-keyed luminance ramp — see ``basket_color_of``), and
    `bearing` (spider + ring bearing). The caller places the whole
    floor at `Pos(0, 0, idx·FLOOR_H)`."""
    return (
        Assembly()
        .with_subassembly(
            "rails",
            _frame.build_floor_horizontals(
                prefix=prefix, color_fn=color_of, material_fn=material_of,
            ),
        )
        .with_subassembly(
            "baskets",
            _bkt.build_basket_ring(
                basket_layout, prefix=prefix,
                size_color_fn=basket_color_of, material_fn=material_of,
            ),
        )
        .with_subassembly(
            "bearing",
            _ramp.build_column_side_static(
                role, prefix=prefix,
                color_fn=color_of, material_fn=material_of,
            ),
        )
    )


# Top-deck shape-truth color — faint blue-white at α=0.25 to render as
# clear PMMA in cad-khana's flat-shaded GLB path (mirrors
# m03_scanner/turret_platform/assembly.py:ACRYLIC_INSERT_COLOR). The
# chitra-cad photo-real path uses ``acrylic_clear`` from its material
# catalog (transmission=1.0, ior=1.49) and effectively ignores this
# color (alpha is stripped at render time and the RGB matches the
# material's base_color).
_TOP_DECK_COLOR = Color(0.95, 0.97, 0.98, 0.25)


def _add_top_deck(a: Assembly, *, top_z: float) -> Assembly:
    """Clear-acrylic cap disc; bottom face tangent to through-column tops."""
    top_deck = (
        Cylinder(TOP_DECK_OD / 2, TOP_DECK_T)
        - Cylinder(TOP_DECK_APERTURE_D / 2, TOP_DECK_T * 2)
    )
    return a.with_part(
        "top_acrylic", top_deck,
        location=Pos(0, 0, top_z + TOP_DECK_T / 2),
        color=_TOP_DECK_COLOR,
        material=material_of("top_acrylic"),
    )


# --- Cross-check helpers -----------------------------------------------
#
# Assertions address qualified tree paths from the tower root. Each
# helper takes `(idx, prefix)` floor descriptors so the path and the
# (still prefix-carrying) leaf name stay in sync.

def _rot(idx: int, pre: str, name: str) -> str:
    """Path of a rotating part on floor `idx` (inside `rotating_stack`)."""
    return f"rotating_stack.f{idx}.{pre}{name}"


def _brg(idx: int, pre: str, name: str) -> str:
    """Path of a floor's bearing-side static part (spider / ring)."""
    return f"f{idx}.bearing.{pre}{name}"


def _rail(idx: int, pre: str, name: str) -> str:
    """Path of a floor's frame-horizontal part (rails subtree)."""
    return f"f{idx}.rails.{pre}{name}"


def _add_floor_pair_checks(
    a: Assembly, lower: tuple[int, str], upper: tuple[int, str],
) -> Assembly:
    """Adjacent-floor cross-checks: upper floor's column-bottom stack
    must clear the lower floor's open-trap envelope and column-top."""
    l_idx, l_pre = lower
    u_idx, u_pre = upper
    upper_stack = [
        _rot(u_idx, u_pre, "column"),
        _brg(u_idx, u_pre, "ring_bearing"),
        _brg(u_idx, u_pre, "bearing_spider"),
    ]
    for u in upper_stack:
        a = (
            a
            .assert_no_interference(_rot(l_idx, l_pre, "trap_open"), u)
            .assert_no_interference(_rot(l_idx, l_pre, "column"), u)
        )
    return a


def _add_top_deck_checks(a: Assembly, top: tuple[int, str]) -> Assembly:
    """Disc rests on the through-column tops (tangent contact at
    z = top_z) and clears the top floor's central column body +
    rotating collar through its central aperture. Inner-hex
    through-columns sit at r ≈ 162, well inside the disc OD/2=330 and
    well outside the aperture R=75."""
    t_idx, t_pre = top
    for k in range(6):
        a = (
            a
            .assert_no_interference("top_acrylic", f"columns.outer_vert_{k}")
            .assert_no_interference("top_acrylic", f"columns.inner_vert_{k}")
        )
    return a.assert_no_interference("top_acrylic", _rot(t_idx, t_pre, "column"))


def _add_drive_train_checks(a: Assembly, base: tuple[int, str]) -> Assembly:
    """Off-axis drive train (ring gear / pinion / motor / mount)
    clearance vs the base floor's column-side stack, ramp_mechanism, and
    frame radials. The pinion↔ring-gear mesh is intentional contact —
    validated numerically in base_floor — so that pair is excluded."""
    b_idx, b_pre = base
    ring = f"drive_train.{b_pre}ring_gear"
    pin = f"drive_train.{b_pre}pinion"
    mtr = f"drive_train.{b_pre}motor"
    mount = f"drive_train.{b_pre}motor_mount"

    # Ring gear bolts up to the base column's bottom collar (tangent at
    # z = 0) and stacks just under the column-side bearing (z ∈ [0, 5]).
    # No overlap with the bearing / spider — pure z stacking.
    a = (
        a
        .assert_no_interference(ring, _rot(b_idx, b_pre, "column"))
        .assert_no_interference(ring, _brg(b_idx, b_pre, "ring_bearing"))
        .assert_no_interference(ring, _brg(b_idx, b_pre, "bearing_spider"))
    )

    # Pinion sits at z ∈ [-11, -1] on the off-axis drive axis, threaded
    # between the Ø138 collar and the rail-inner ring. It must clear the
    # radial rails (the tight packaging constraint — guarded numerically
    # against the tip circle in base_floor) and stay below the bearing.
    for k in range(6):
        a = (
            a
            .assert_no_interference(pin, _rail(b_idx, b_pre, f"radial_inner_{k}"))
            .assert_no_interference(pin, _rail(b_idx, b_pre, f"radial_outer_{k}"))
        )
    for other in (
        _rot(b_idx, b_pre, "column"),
        _brg(b_idx, b_pre, "ring_bearing"),
        _brg(b_idx, b_pre, "bearing_spider"),
    ):
        a = a.assert_no_interference(pin, other)

    # Motor body hangs at z ∈ [-65, -25], below every other modeled part
    # and inside the rail-inner ring — clears the whole base floor.
    for other in (
        _rot(b_idx, b_pre, "column"),
        _brg(b_idx, b_pre, "ring_bearing"),
        _brg(b_idx, b_pre, "bearing_spider"),
        _rot(b_idx, b_pre, "ramp"),
        _rot(b_idx, b_pre, "trap_closed"),
        _rot(b_idx, b_pre, "trap_open"),
    ):
        a = a.assert_no_interference(mtr, other)
    for k in range(6):
        a = (
            a
            .assert_no_interference(mtr, _rail(b_idx, b_pre, f"radial_inner_{k}"))
            .assert_no_interference(mtr, _rail(b_idx, b_pre, f"radial_outer_{k}"))
            .assert_no_interference(mtr, _rail(b_idx, b_pre, f"inner_horiz_{k}"))
        )

    # Motor mount is a radial arm at z ∈ [-25, -20]; it bolts up into one
    # rail (tangent at z = -20) and clears the others + the column-side
    # stack (all at z ≥ 0).
    for k in range(6):
        a = (
            a
            .assert_no_interference(mount, _rail(b_idx, b_pre, f"radial_inner_{k}"))
            .assert_no_interference(mount, _rail(b_idx, b_pre, f"radial_outer_{k}"))
            .assert_no_interference(mount, _rail(b_idx, b_pre, f"inner_horiz_{k}"))
        )
    for other in (
        _rot(b_idx, b_pre, "column"),
        _brg(b_idx, b_pre, "ring_bearing"),
        _brg(b_idx, b_pre, "bearing_spider"),
    ):
        a = a.assert_no_interference(mount, other)

    return a


def _assert_design_system_coverage(a: Assembly) -> None:
    """Every named part (root + any sub-assembly) must appear in the
    active design system's tag dict (``None`` allowed for hardware kept
    at material's intrinsic color). A part added without a tag fails
    here instead of silently rendering off-palette."""
    untagged = sorted(
        {
            p.name for p in a.placed_parts
            if basename_of(p.name) not in DESIGN_SYSTEM.tag_of
        }
    )
    assert not untagged, (
        f"{len(untagged)} m05 part(s) missing a tag for design system "
        f"'{DESIGN_SYSTEM.name}' in design_systems.py: {untagged}"
    )


# --- Public composition ------------------------------------------------

def build_tower(
    floor_plan: list[tuple[int, str]],
    *,
    basket_layout=FLOOR_LAYOUT,
    with_top_deck: bool = True,
    with_drive_train: bool = True,
) -> tuple[Assembly, list[str]]:
    """Compose a hex-tower diverter assembly with cross-floor +
    drive-train + top-deck checks, parameterised on which floors,
    baskets, and capping features are present.

    Returns ``(assembly, prefixes)`` so callers can hang custom
    assertions off the per-floor prefixes if needed.

    `floor_plan` is a list of ``(idx, role)`` tuples in stacking order
    (lowest first). The exhibit variant passes a reduced plan; the
    production tower passes ``FLOOR_PLAN`` and gets the default
    ``FLOOR_LAYOUT`` of 11 baskets per floor.

    `basket_layout` is either a single layout list shared across every
    floor (production / single-tier exhibits) or a
    ``dict[floor_idx, layout]`` keyed by floor index for per-floor
    overrides (the article exhibit, which stages different sizes on
    different floors).

    `with_top_deck` and `with_drive_train` strip those features for
    the exhibit. Drive-train assertions and top-deck assertions are
    skipped accordingly — they reference parts that wouldn't exist.
    """
    def _layout_for(idx: int):
        if isinstance(basket_layout, dict):
            return basket_layout[idx]
        return basket_layout

    top_z = (max(idx for idx, _ in floor_plan) + 1) * FLOOR_H

    a = Assembly().with_subassembly(
        "columns",
        _frame.build_through_columns(
            z_bot=-_frame.VERT_EXT_BELOW, z_top=top_z,
            color_fn=color_of, material_fn=material_of,
        ),
    )

    n = len(floor_plan)
    rotating = Assembly()
    prefixes: list[str] = []
    for i, (idx, role) in enumerate(floor_plan):
        pre = _floor_prefix(idx, role)
        fz_loc = Pos(0, 0, idx * FLOOR_H)
        a = a.with_subassembly(
            f"f{idx}",
            _build_floor_static(role, pre, _layout_for(idx)),
            location=fz_loc,
        )
        rotating = rotating.with_subassembly(
            f"f{idx}",
            _ramp.build_column_side_rotating(
                role, pre,
                # Trap-open envelope only needs to exist on floors that
                # have an upper neighbour to check against.
                with_trap_open=(i < n - 1),
                color_fn=color_of, material_fn=material_of,
            ),
            location=fz_loc,
        )
        prefixes.append(pre)

    if with_top_deck:
        a = _add_top_deck(a, top_z=top_z)

    # The rotating stack — every floor's column / hinge pin / ramp /
    # trap parts — is one rigid body driven by a single revolute joint
    # about world Z. Animation factories update the joint angle via
    # ``with_joint_angle("rotating_stack", theta)``; static callers (the
    # production tower, ``khana check``) leave it at 0, which matches
    # the previous flat geometry exactly.
    a = a.with_subassembly(
        "rotating_stack",
        rotating,
        joint=RevoluteJoint(axis=Axis.Z, angle_deg=0.0),
    )

    if with_drive_train:
        base_idx, base_pre = next(
            (idx, p)
            for (idx, role), p in zip(floor_plan, prefixes)
            if role == "base"
        )
        a = a.with_subassembly(
            "drive_train",
            _base.build_drive_train(
                prefix=base_pre, color_fn=color_of, material_fn=material_of,
            ),
            location=Pos(0, 0, base_idx * FLOOR_H),
        )

    _assert_design_system_coverage(a)

    # Adjacent-floor pairs only. Non-adjacent floors are >1·FLOOR_H
    # apart and the trap-open envelope reaches at most ~70 mm above
    # its hinge, so floor i+2 is comfortably out of range. Cross-subassembly
    # references resolve through ``placed_parts`` at evaluation time.
    floors = [(idx, pre) for (idx, _), pre in zip(floor_plan, prefixes)]
    for i in range(n - 1):
        a = _add_floor_pair_checks(a, floors[i], floors[i + 1])

    if with_top_deck:
        a = _add_top_deck_checks(a, floors[-1])

    if with_drive_train:
        a = _add_drive_train_checks(a, (base_idx, base_pre))

    return a, prefixes


# --- Production assembly -----------------------------------------------

assembly, _ = build_tower(FLOOR_PLAN)


if __name__ == "__main__":
    check(assembly, out="outputs")

"""
M05 hex-tower — top-level 4-floor diverter assembly.

Composes four floors with roles `[base, middle, middle, top]` stacked
at world z = `[0, FLOOR_H, 2·FLOOR_H, 3·FLOOR_H]`. Bricks enter above
the top floor and fall down through per-floor traps that divert them
into the floor's baskets (or release them through to the floor below).

Each floor reuses the same sub-assemblies as `single_floor.py`:

- frame — through-columns span all floors as one continuous 2020;
  hex rings, brackets, and radial rails repeat per floor (suffixed by
  floor index).
- ramp_mechanism — top-bearing architecture: bearing+spider geometry
  is uniform across roles (short ring at z_local 0..5). Roles differ
  only at the column bottom: base has a bottom collar (motor + ring-
  gear interface); middle/top have a bottom face flange that mates
  with the column below at the joint plane (z_world = N·220 + 273).
  Each column carries a rotating collar at z_local 220..225 that the
  bearing of the floor *above* grips.
- basket — `FLOOR_LAYOUT` (mixed N=6/9/12/18) replicated per floor.
- base_floor — off-axis drive train (ring gear / pinion / motor /
  motor mount) below the rail datum on the base floor only. The ring
  gear bolts to the base column's bottom collar; an off-axis motor
  drives it through a pinion. No central drive shaft — Lego falls
  through the column axis.

Cross-floor checks (the value the tower adds beyond `single_floor.py`):

- Upper floor's bearing+spider engages lower floor's column rotating
  collar (z_local 220..225 of lower = z_local 0..5 of upper). Grease
  film clearance — `assert_no_interference` guards the radial gap.
- Lower column top (z = 273 in lower local) vs upper column bottom
  (z = 53 in upper local) tangent at the joint plane. The two flanges
  meet face-to-face; assert_no_interference allows that contact.
- Lower floor's open-trap envelope (max-open free end at z ≈ 36 mm
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

from build123d import Axis, Color, Cylinder, Pos, Rot

from cad_khana.mechanism.assembly import Assembly, RevoluteJoint
from cad_khana.mechanism.check import check

from base_floor import assembly as _base
from basket import assembly as _bkt
from design_systems import DESIGN_SYSTEM, basename_of, basket_color_of, color_of
from materials import material_of
from frame import assembly as _frame
from params import FLOOR_H, FLOOR_LAYOUT
from ramp_mechanism import assembly as _ramp


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


# --- Per-part add helpers ----------------------------------------------

def _add(a: Assembly, name: str, part, **kwargs) -> Assembly:
    """Wrap ``.with_part(...)`` so every column-side / basket / base-floor
    call site picks up the active design system's color + material
    without repeating the boilerplate. ``color_of`` / ``material_of``
    strip the floor prefix internally before lookup."""
    return a.with_part(
        name, part, color=color_of(name), material=material_of(name), **kwargs,
    )


def _add_column_side_static(
    a: Assembly, idx: int, role: str, prefix: str,
) -> Assembly:
    """Static column-side parts: bearing spider + ring bearing. These
    stay with the frame regardless of the rotating-stack DOF."""
    fz_loc = Pos(0, 0, idx * FLOOR_H)
    a = _add(a, f"{prefix}bearing_spider", _ramp.bearing_spider(role),
             location=fz_loc)
    a = _add(a, f"{prefix}ring_bearing", _ramp.ring_bearing(role),
             location=fz_loc)
    return a


def _add_column_side_rotating(
    rotating: Assembly, idx: int, role: str, prefix: str, *,
    with_trap_open: bool,
) -> Assembly:
    """Rotating column-side parts: column + hinge pin + ramp +
    trap-closed (+ trap-open if this floor has an upper neighbour).
    Added to the rotating sub-assembly; the revolute joint in
    ``build_tower`` carries the per-frame azimuth."""
    fz_loc = Pos(0, 0, idx * FLOOR_H)
    rotating = _add(rotating, f"{prefix}column", _ramp.column(role),
                    location=fz_loc)
    rotating = _add(rotating, f"{prefix}hinge_pin", _ramp.hinge_pin(),
                    location=fz_loc * _ramp.hinge_pin_loc)
    rotating = _add(rotating, f"{prefix}ramp", _ramp.ramp(),
                    location=fz_loc * _ramp.ramp_loc)
    rotating = _add(rotating, f"{prefix}trap_closed", _ramp.trap(),
                    location=fz_loc * _ramp.hinge_loc * Rot(0, _ramp.TRAP_CLOSED_DEG, 0))
    if with_trap_open:
        rotating = _add(rotating, f"{prefix}trap_open", _ramp.trap(),
                        location=fz_loc * _ramp.hinge_loc * Rot(0, _ramp.TRAP_OPEN_DEG, 0))
    return rotating


def _add_baskets(
    a: Assembly, idx: int, prefix: str, basket_layout,
) -> Assembly:
    """Baskets per the supplied layout at floor `idx`. Color is a
    size-keyed luminance shift off the palette's CONSUMABLE base
    (see ``basket_color_of``), so basket sizes are immediately
    distinguishable in renders. Bypasses ``_add`` to pass the
    per-instance color explicitly."""
    fz_loc = Pos(0, 0, idx * FLOOR_H)
    for center_deg, n_size, notches in basket_layout:
        bname = f"{prefix}basket_{int(round(center_deg)):03d}_{n_size:02d}"
        a = a.with_part(
            bname,
            _bkt.basket(
                half_angle_deg=180.0 / n_size,
                notch_left="left" in notches,
                notch_right="right" in notches,
                notch_middle="middle" in notches,
            ),
            location=fz_loc * Rot(0, 0, center_deg),
            color=basket_color_of(n_size),
            material=material_of(bname),
        )
    return a


def _add_drive_train(a: Assembly, idx: int, prefix: str) -> Assembly:
    """Off-axis motor + pinion + ring gear + motor mount below the base
    floor's rail datum. The ring gear bolts to the base column's bottom
    collar; the pinion on the off-axis motor shaft drives it. No central
    drive shaft — bricks fall through the column axis."""
    fz_loc = Pos(0, 0, idx * FLOOR_H)
    a = _add(a, f"{prefix}ring_gear", _base.ring_gear(), location=fz_loc)
    a = _add(a, f"{prefix}pinion", _base.pinion(), location=fz_loc)
    a = _add(a, f"{prefix}motor", _base.motor(), location=fz_loc)
    a = _add(a, f"{prefix}motor_mount", _base.motor_mount(), location=fz_loc)
    return a


def _add_floor(
    a: Assembly, rotating: Assembly, idx: int, role: str, *,
    basket_layout,
    with_trap_open: bool,
    with_drive_train: bool,
) -> tuple[str, Assembly, Assembly]:
    """One full floor: frame horizontals + static column-side parts +
    baskets + (drive train if requested) into ``a``; column / hinge pin
    / ramp / trap parts into ``rotating``. Returns
    ``(prefix, a, rotating)``."""
    pre = _floor_prefix(idx, role)
    a = _frame.add_floor_horizontals(
        a, z=idx * FLOOR_H, prefix=pre,
        color_fn=color_of, material_fn=material_of,
    )
    a = _add_column_side_static(a, idx, role, pre)
    rotating = _add_column_side_rotating(
        rotating, idx, role, pre, with_trap_open=with_trap_open,
    )
    a = _add_baskets(a, idx, pre, basket_layout)
    if with_drive_train:
        a = _add_drive_train(a, idx, pre)
    return pre, a, rotating


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

def _add_floor_pair_checks(
    a: Assembly, lower_pre: str, upper_pre: str,
) -> Assembly:
    """Adjacent-floor cross-checks: upper floor's column-bottom stack
    must clear the lower floor's open-trap envelope and column-top."""
    upper_stack = [
        f"{upper_pre}column",
        f"{upper_pre}ring_bearing",
        f"{upper_pre}bearing_spider",
    ]
    for u in upper_stack:
        a = (
            a
            .assert_no_interference(f"{lower_pre}trap_open", u)
            .assert_no_interference(f"{lower_pre}column", u)
        )
    return a


def _add_top_deck_checks(a: Assembly, top_pre: str) -> Assembly:
    """Disc rests on the through-column tops (tangent contact at
    z = top_z) and clears the top floor's central column body +
    rotating collar through its central aperture. Inner-hex
    through-columns sit at r ≈ 162, well inside the disc OD/2=330 and
    well outside the aperture R=75."""
    for k in range(6):
        a = (
            a
            .assert_no_interference("top_acrylic", f"outer_vert_{k}")
            .assert_no_interference("top_acrylic", f"inner_vert_{k}")
        )
    return a.assert_no_interference("top_acrylic", f"{top_pre}column")


def _add_drive_train_checks(a: Assembly, base_pre: str) -> Assembly:
    """Off-axis drive train (ring gear / pinion / motor / mount)
    clearance vs the base floor's column-side stack, ramp_mechanism, and
    frame radials. The pinion↔ring-gear mesh is intentional contact —
    validated numerically in base_floor — so that pair is excluded."""
    ring = f"{base_pre}ring_gear"
    pin = f"{base_pre}pinion"
    mtr = f"{base_pre}motor"
    mount = f"{base_pre}motor_mount"

    # Ring gear bolts up to the base column's bottom collar (tangent at
    # z = 0) and stacks just under the column-side bearing (z ∈ [0, 5]).
    # No overlap with the bearing / spider — pure z stacking.
    a = (
        a
        .assert_no_interference(ring, f"{base_pre}column")
        .assert_no_interference(ring, f"{base_pre}ring_bearing")
        .assert_no_interference(ring, f"{base_pre}bearing_spider")
    )

    # Pinion sits at z ∈ [-11, -1] on the off-axis drive axis, threaded
    # between the Ø138 collar and the rail-inner ring. It must clear the
    # radial rails (the tight packaging constraint — guarded numerically
    # against the tip circle in base_floor) and stay below the bearing.
    for k in range(6):
        a = (
            a
            .assert_no_interference(pin, f"{base_pre}radial_inner_{k}")
            .assert_no_interference(pin, f"{base_pre}radial_outer_{k}")
        )
    for other in (
        f"{base_pre}column",
        f"{base_pre}ring_bearing",
        f"{base_pre}bearing_spider",
    ):
        a = a.assert_no_interference(pin, other)

    # Motor body hangs at z ∈ [-65, -25], below every other modeled part
    # and inside the rail-inner ring — clears the whole base floor.
    for other in (
        f"{base_pre}column",
        f"{base_pre}ring_bearing",
        f"{base_pre}bearing_spider",
        f"{base_pre}ramp",
        f"{base_pre}trap_closed",
        f"{base_pre}trap_open",
    ):
        a = a.assert_no_interference(mtr, other)
    for k in range(6):
        a = (
            a
            .assert_no_interference(mtr, f"{base_pre}radial_inner_{k}")
            .assert_no_interference(mtr, f"{base_pre}radial_outer_{k}")
            .assert_no_interference(mtr, f"{base_pre}inner_horiz_{k}")
        )

    # Motor mount is a radial arm at z ∈ [-25, -20]; it bolts up into one
    # rail (tangent at z = -20) and clears the others + the column-side
    # stack (all at z ≥ 0).
    for k in range(6):
        a = (
            a
            .assert_no_interference(mount, f"{base_pre}radial_inner_{k}")
            .assert_no_interference(mount, f"{base_pre}radial_outer_{k}")
            .assert_no_interference(mount, f"{base_pre}inner_horiz_{k}")
        )
    for other in (
        f"{base_pre}column",
        f"{base_pre}ring_bearing",
        f"{base_pre}bearing_spider",
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

    a = Assembly()
    rotating = Assembly()
    a = _frame.add_through_columns(
        a, z_bot=-_frame.VERT_EXT_BELOW, z_top=top_z,
        color_fn=color_of, material_fn=material_of,
    )

    n = len(floor_plan)
    prefixes: list[str] = []
    for i, (idx, role) in enumerate(floor_plan):
        pre, a, rotating = _add_floor(
            a, rotating, idx, role,
            basket_layout=_layout_for(idx),
            # Trap-open envelope only needs to exist on floors that
            # have an upper neighbour to check against.
            with_trap_open=(i < n - 1),
            with_drive_train=(role == "base" and with_drive_train),
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

    _assert_design_system_coverage(a)

    # Adjacent-floor pairs only. Non-adjacent floors are >1·FLOOR_H
    # apart and the trap-open envelope reaches at most ~70 mm above
    # its hinge, so floor i+2 is comfortably out of range. Cross-subassembly
    # references resolve through ``placed_parts`` at evaluation time.
    for i in range(n - 1):
        a = _add_floor_pair_checks(a, prefixes[i], prefixes[i + 1])

    if with_top_deck:
        a = _add_top_deck_checks(a, prefixes[-1])

    if with_drive_train:
        base_pre = next(
            p for (idx, role), p in zip(floor_plan, prefixes) if role == "base"
        )
        a = _add_drive_train_checks(a, base_pre)

    return a, prefixes


# --- Production assembly -----------------------------------------------

assembly, _ = build_tower(FLOOR_PLAN)


if __name__ == "__main__":
    check(assembly, out="outputs")

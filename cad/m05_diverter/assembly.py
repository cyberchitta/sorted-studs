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
  ramp + trap servo + the jointed `trap` subtree (plate, hinge pin,
  coupler, horn — hanging, at rest)
  (`ramp_mechanism.build_column_side_rotating`). Roles differ only at
  the column bottom: base has a bottom collar (motor + ring-gear
  interface); middle/top have a bottom face flange that mates with
  the column below at the joint plane (z_world = N·220 + 273). Each
  column carries a rotating collar at z_local 220..225 that the
  bearing of the floor *above* grips.
- `drive_train` — off-axis ring gear / pinion / motor / motor mount
  below the base floor's rail datum (`base_floor.build_drive_train`).
  The ring gear bolts to the base column's bottom collar; no central
  drive shaft — Lego falls through the column axis. It declares the
  fall path out of the base column as keep-outs.
- `catch` — the bottom catch: a tray under the motor on a three-post
  cradle hung from the base floor's radial rails
  (`base_floor.build_bottom_catch`). Whatever falls past every trap —
  the fault case — and parts wider than any floor's ramp end here.
- `top_acrylic` — clear disc capping the tower (root part). At the
  two through-deck post vertices (`params.THROUGH_DECK_POSTS`) it is
  notched from the rim so the column and the bracket's vertical arm
  pass through.
- `top_cap` — the two outer-vertex brackets under the deck at the
  post vertices: the same printed part the floors use, in the same
  relationship to the deck as to a floor's rails (hub + arms under
  the plate, vertical arm rising past it to clamp the column above).
  The other four outer vertices and all six inner ones keep the deck
  resting on the column top; their top-cap bracket variant is still
  open (`_notes/state/m05-diverter.md` §Top deck).

Part identity is the dotted tree path
(`rotating_stack.f0.column`); floor identity lives in the `f{idx}`
subtree segment and leaf names are plain. Attribute maps (colors,
materials, detail targets) key on the leaf via
`design_systems.basename_of`.

Cross-floor checks (the value the tower adds beyond `single_floor.py`):

- Upper floor's bearing+spider engages lower floor's column rotating
  collar (z_local 220..225 of lower = z_local 0..5 of upper). Grease
  film clearance — `assert_no_interference` guards the radial gap.
- Lower column top (z = 273 in lower local) vs upper column bottom
  (z = 53 in upper local) tangent at the joint plane. The two flanges
  meet face-to-face; assert_no_interference allows that contact.
- Lower floor's trap servo (outside the column's +Y wall, at
  z ≈ 13..26 above the upper floor's bracket) vs upper floor's
  column-bottom stack — it must pass above the bearing and spider
  and below this floor's column-bottom (z ≥ 53).

Intra-floor pairs (basket-vs-frame, ramp-vs-basket, column-side self-
clearance) are validated by `single_floor.py` and the standalone
sub-assemblies; the geometry is z-translated only, so per-floor
enumeration here would be redundant.

The composition is factored into `build_tower(floor_plan, ...)` so an
exhibit variant (`exhibit_assembly.py`) can reuse every per-floor
helper with a reduced plan + basket layout + no drive train / top
deck. The production assembly is `build_tower(FLOOR_PLAN)`.
"""

from functools import cache

import math
from pathlib import Path


from build123d import Axis, Box, Color, Cylinder, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly, RevoluteJoint

from .base_floor import assembly as _base
from .basket import assembly as _bkt
from ..design_systems import DEFAULT_SCHEMA, paint
from .design_systems import (
    basename_of,
    color_for,
    design_system,
)
from .materials import material_of
from .frame import assembly as _frame
from .outer_vertex_bracket.assembly import (
    BRACKET_Z_TOP,
    EXT_SIZE,
    PLATE_T as OUTER_BRACKET_PLATE_T,
    bracket as outer_bracket,
)
from .aim import ramp_exit_w_for
from .params import FLOOR_H, FLOOR_LAYOUTS, R_VERT_OUTER, THROUGH_DECK_POSTS, basket_name
from .ramp_mechanism import assembly as _ramp


# Ramp sweep clearance over the basket rim plane is declared once in
# `ramp_mechanism.build_column_side_rotating` (`ramp_sweep_over_basket_rim`,
# a Z-invariant datum-plane assert_distance) and propagates here
# qualified per floor — no mirrored parametric assert needed.


# --- Top deck ----------------------------------------------------------
#
# Clear-acrylic disc capping the tower. Hopper / rotary-feed and
# ceiling-sleeve mounting are deferred — see NEXT_STEPS "Top deck —
# open detail".
TOP_DECK_OD = 660.0       # circumscribes column outer corners (~322 mm) + ~8 mm clearance
TOP_DECK_T = 10.0
TOP_DECK_APERTURE_D = 150.0  # clears top column's rotating collar (OD 138 at z 880..885)

# Through-deck posts: the deck is notched at each post vertex so the
# 2020 column and the bracket's vertical arm (on the column's outward
# face, 6 mm plate) pass through with this much clearance per side.
# The arm sits at r ≈ 322..328 under a Ø660 deck (rim at 330), so a
# closed hole would leave a ~1 mm sliver of acrylic — the cut runs
# out through the rim instead. Decided (owner 2026-09-15): the OD
# stays 660 and the two notches stay open; no thin outer strip.
TOP_DECK_POST_CLEAR = 1.0
_NOTCH_RUNOUT = 40.0         # past the rim; anything ≥ 330 - 312 = 18 opens it


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


# --- Per-floor builders ------------------------------------------------

def _build_floor_static(role: str, basket_layout) -> Assembly:
    """One floor's static side as a sub-`Assembly` in the floor-local
    frame (rail-top datum at z=0): `rails` (hex rings + radials +
    vertex brackets), `baskets` (per the supplied layout, colored by
    the size-keyed luminance ramp — see ``basket_color_of``), and
    `bearing` (spider + ring bearing). The caller places the whole
    floor at `Pos(0, 0, idx·FLOOR_H)`; floor identity is the caller's
    `f{idx}` subtree name.

    The baskets-at-rest claims against this floor's rails and bearing
    are declared here, where the layout is known, so every floor of the
    tower is checked in its own layout — `single_floor.py` sees only
    the reference one. Columns are tower-global; the baskets' claims
    against them are `check_basket_removal.py` (rest is its first
    step).

    Rest is asserted as contact, not only as no-overlap — a basket
    floating above its rails passes every no-interference claim. Each
    basket stands on the inner and outer hex edges of its own sector
    (its floor spans both, `basket/assembly.py`)."""
    a = (
        Assembly()
        .with_subassembly(
            "rails",
            _frame.build_floor_horizontals(
                material_fn=material_of,
            ),
        )
        .with_subassembly(
            "baskets",
            _bkt.build_basket_ring(
                basket_layout,
                material_fn=material_of,
            ),
        )
        .with_subassembly(
            "bearing",
            _ramp.build_column_side_static(
                role, material_fn=material_of,
            ),
        )
        .assert_no_interference_between("baskets", "rails")
        .assert_no_interference_between("baskets", "bearing")
    )
    for center_deg, n_size, _notches in basket_layout:
        name = f"baskets.{basket_name(center_deg, n_size)}"
        k = _frame.edge_under(center_deg)
        a = (
            a
            .assert_tangent_contact(name, f"rails.inner_horiz_{k}")
            .assert_tangent_contact(name, f"rails.outer_horiz_{k}")
        )
    # The spider's rest on its rails — each saddle's top plate on a
    # radial rail's top face (SADDLE_CEIL_Z) — is NOT asserted: nothing
    # in cad-khana 0.14 can see it. A tangent contact passes on the
    # saddle's side walls, which touch the rail's sides whether or not
    # the plate is down (0.0 with the spider lifted 7 mm), and
    # `assert_distance(along="-Z")` compares whole-body extents, so the
    # side walls reaching z = -7 read -7 resting or floating (tried
    # 2026-09-27; field note in cad-khana).
    return a


# Top-deck shape-truth color — faint blue-white at α=0.25 to render as
# clear PMMA in cad-khana's flat-shaded GLB path (mirrors
# m03_scanner/turret_platform/assembly.py:ACRYLIC_INSERT_COLOR). The
# chitra-cad photo-real path uses ``acrylic_clear`` from its material
# catalog (transmission=1.0, ior=1.49) and effectively ignores this
# color (alpha is stripped at render time and the RGB matches the
# material's base_color).
_TOP_DECK_COLOR = Color(0.95, 0.97, 0.98, 0.25)


def _post_vertex_pose(k: int, z: float):
    """Outer vertex k's bracket frame at height z: local +X inward,
    as `frame.build_floor_horizontals` places every vertex bracket."""
    v_angle = (k + 0.5) * 60.0
    return Pos(
        R_VERT_OUTER * math.cos(math.radians(v_angle)),
        R_VERT_OUTER * math.sin(math.radians(v_angle)),
        z,
    ) * Rot(0, 0, v_angle + 180.0)


def _post_notch() -> Part:
    """Deck cut-out at a post vertex, in the bracket's local frame
    (centred on the column axis, +X inward, -X outward to the rim):
    clears the column's inward face by TOP_DECK_POST_CLEAR, the
    bracket's vertical arm on the outward face by the same, and runs
    out through the rim."""
    x_in = EXT_SIZE / 2 + TOP_DECK_POST_CLEAR
    x_out = -(EXT_SIZE / 2 + OUTER_BRACKET_PLATE_T + TOP_DECK_POST_CLEAR + _NOTCH_RUNOUT)
    w = EXT_SIZE + 2 * TOP_DECK_POST_CLEAR
    return Pos((x_in + x_out) / 2, 0, 0) * Box(x_in - x_out, w, TOP_DECK_T * 2)


def _add_top_deck(
    a: Assembly, *, top_z: float, posts: dict[int, float],
) -> Assembly:
    """Clear-acrylic cap disc; bottom face tangent to through-column
    tops, notched at each through-deck post vertex."""
    top_deck = (
        Cylinder(TOP_DECK_OD / 2, TOP_DECK_T)
        - Cylinder(TOP_DECK_APERTURE_D / 2, TOP_DECK_T * 2)
    )
    # The disc is placed at z = top_z + T/2, so cut in deck-local z.
    for k in posts:
        top_deck = top_deck - _post_vertex_pose(k, 0.0) * _post_notch()
    return a.with_part(
        "top_acrylic", top_deck,
        location=Pos(0, 0, top_z + TOP_DECK_T / 2),
        color=_TOP_DECK_COLOR,
        material=material_of("top_acrylic"),
    )


def _add_top_cap(
    a: Assembly, *, top_z: float, posts: dict[int, float],
) -> Assembly:
    """The through-form vertex bracket under the deck at each post
    vertex, as subtree `top_cap`: hub top face tangent to the deck's
    underside (the rails' bottom-face relationship, `BRACKET_Z_TOP`),
    vertical arm rising through the notch to clamp the column above
    the deck. Leaf names match the floors' so the tag maps apply."""
    cap = Assembly()
    for k in posts:
        name = f"outer_vertex_bracket_{k}"
        cap = cap.with_part(
            name,
            outer_bracket(),
            location=_post_vertex_pose(k, top_z - BRACKET_Z_TOP),
            material=material_of(name),
        )
    return a.with_subassembly("top_cap", cap)


# --- Cross-check helpers -----------------------------------------------
#
# Assertions address qualified tree paths from the tower root; floors
# are identified by index (the `f{idx}` subtree name).

def _rot(idx: int, name: str) -> str:
    """Path of a rotating part on floor `idx` (inside `rotating_stack`)."""
    return f"rotating_stack.f{idx}.{name}"


def _brg(idx: int, name: str) -> str:
    """Path of a floor's bearing-side static part (spider / ring)."""
    return f"f{idx}.bearing.{name}"


def _rail(idx: int, name: str) -> str:
    """Path of a floor's frame-horizontal part (rails subtree)."""
    return f"f{idx}.rails.{name}"


def _add_floor_pair_checks(a: Assembly, lower: int, upper: int) -> Assembly:
    """Adjacent-floor cross-checks: upper floor's column-bottom stack
    must clear the lower floor's trap servo and column-top."""
    upper_stack = [
        _rot(upper, "column"),
        _brg(upper, "ring_bearing"),
        _brg(upper, "bearing_spider"),
    ]
    for u in upper_stack:
        a = (
            a
            .assert_no_interference(_rot(lower, "trap_servo"), u)
            .assert_no_interference(_rot(lower, "column"), u)
        )
    # The two columns mate face-to-face at the joint plane under
    # gravity preload — the bas-relief bumps transfer torque, nothing
    # bolts the stack together, so a gap here means the tower isn't
    # standing on itself. `assert_no_interference` above is blind to
    # that; this is the claim. (Declared per adjacent pair here, where
    # the real neighbours meet; `ramp_mechanism`'s standalone declares
    # the same rest against its `lower_column` stand-in.)
    return a.assert_tangent_contact(
        _rot(lower, "column"), _rot(upper, "column")
    )


def _add_top_deck_checks(
    a: Assembly, top: int, posts: dict[int, float],
) -> Assembly:
    """Disc rests on the through-column tops (tangent contact at
    z = top_z) and clears the top floor's central column body +
    rotating collar through its central aperture. Inner-hex
    through-columns sit at r ≈ 162, well inside the disc OD/2=330 and
    well outside the aperture R=75.

    At a through-deck post vertex the column does not end at the
    deck, so the rest moves to the bracket under it: the notch clears
    the column by TOP_DECK_POST_CLEAR (measured, so a drifted notch
    or a rotated column reads as a number), the deck sits on the
    bracket's hub + arms, and the bracket's vertical arm threads the
    notch without touching it."""
    for k in range(6):
        col = f"columns.outer_vert_{k}"
        a = a.assert_no_interference("top_acrylic", col)
        if k in posts:
            brk = f"top_cap.outer_vertex_bracket_{k}"
            a = (
                a
                # Constructed at exactly TOP_DECK_POST_CLEAR; the bound
                # is the same constant (cad/CLAUDE.md §6, corollary 2).
                .assert_distance(
                    "top_acrylic", col,
                    min_mm=TOP_DECK_POST_CLEAR,
                    name=f"deck_notch_clears_outer_vert_{k}",
                )
                .assert_tangent_contact("top_acrylic", brk)
                .assert_no_interference(brk, col)
                .assert_no_interference(brk, f"columns.inner_vert_{k}")
            )
        else:
            # …and it must actually *sit* on the rest: the disc is
            # unsupported between columns, so one short column is a
            # disc resting on one point fewer. A gap passes
            # no-interference.
            a = a.assert_tangent_contact("top_acrylic", col)
        a = (
            a
            .assert_no_interference("top_acrylic", f"columns.inner_vert_{k}")
            .assert_tangent_contact("top_acrylic", f"columns.inner_vert_{k}")
        )
    # The top floor's trap servo rides above the deck plane, radially
    # past the aperture (r up to ≈ 91 vs 75) — it must clear the disc.
    return (
        a
        .assert_no_interference("top_acrylic", _rot(top, "column"))
        .assert_no_interference("top_acrylic", _rot(top, "trap_servo"))
    )


def _add_base_bearing_checks(a: Assembly, base: int) -> Assembly:
    """The base column stands on the turntable, which stands on the
    seat: two tangent rests that together are the rotating stack's
    axial load path. Seat-on-turntable is declared once in
    `ramp_mechanism.build_column_side_static` and propagates; the
    column-on-turntable rest spans the static and rotating subtrees,
    so it is declared here (as the standalone declares it at its own
    root)."""
    return a.assert_tangent_contact(
        _brg(base, "ring_bearing"), _rot(base, "column"),
        name="column_on_turntable",
    )


def _add_drive_train_checks(a: Assembly, base: int) -> Assembly:
    """Off-axis drive train (ring gear / pinion / motor / mount)
    clearance vs the base floor's column-side stack, ramp_mechanism, and
    frame radials. The pinion↔ring-gear mesh is intentional contact —
    validated numerically in base_floor — so that pair is excluded."""
    ring = "drive_train.ring_gear"
    pin = "drive_train.pinion"
    mtr = "drive_train.motor"
    mount = "drive_train.motor_mount"

    # Ring gear bolts up to the underside of the base column's bottom
    # flange (tangent at z = BASE_COL_Z_BOT) and its standoff drops
    # through the turntable's bore and the seat's bore to the mesh
    # below the rails. No overlap with either — the seat bore is the
    # tight one, so its radial gap is measured, not just non-negative.
    a = (
        a
        .assert_no_interference(ring, _rot(base, "column"))
        .assert_no_interference(ring, _brg(base, "ring_bearing"))
        .assert_no_interference(ring, _brg(base, "bearing_spider"))
        .assert_distance(
            ring, _brg(base, "bearing_spider"), min_mm=2.0,
            name="ring_gear_standoff_seat_bore_gap",
        )
        # The ring's standoff top face seats on the flange's underside
        # — that seat is the whole bolted interface, and a ring
        # hanging 3 mm below it would satisfy no-interference just as
        # well.
        .assert_tangent_contact(ring, _rot(base, "column"))
    )

    # Pinion sits between PINION_Z_BOT and PINION_Z_TOP on the off-axis drive
    # axis, midway between two radial rails (rails at (k+0.5)·60° ⇒ k
    # from MOTOR_AZIMUTH_DEG ± 30°). It must clear the rails and stay
    # below the bearing. Its tip circle vs each flanking rail is measured
    # via grow_a_mm while the modeled body stays a pitch cylinder — the
    # modeled-body no-interference checks below can't see the tooth reach.
    mount_rail_ks = [
        round((_base.MOTOR_AZIMUTH_DEG + off) / 60.0 - 0.5) % 6
        for off in _base.MOUNT_RAIL_OFFSETS_DEG
    ]
    for k in mount_rail_ks:
        a = a.assert_distance(
            pin, _rail(base, f"radial_inner_{k}"),
            min_mm=1.0, grow_a_mm=_base.ADDENDUM,
            name=f"pinion_tip_rail_{k}_gap",
        )
    for k in range(6):
        a = (
            a
            .assert_no_interference(pin, _rail(base, f"radial_inner_{k}"))
            .assert_no_interference(pin, _rail(base, f"radial_outer_{k}"))
        )
    for other in (
        _rot(base, "column"),
        _brg(base, "ring_bearing"),
        _brg(base, "bearing_spider"),
    ):
        a = a.assert_no_interference(pin, other)

    # Motor body hangs at z ∈ [-65, -25], below every other modeled part
    # and inside the rail-inner ring — clears the whole base floor. The
    # trap is named as its subtree, so every part of it is covered.
    a = (
        a
        .assert_no_interference_between([mtr], [
            _rot(base, "column"),
            _brg(base, "ring_bearing"),
            _brg(base, "bearing_spider"),
            _rot(base, "ramp"),
            _rot(base, "trap_servo"),
        ])
        .assert_no_interference_between([mtr], _rot(base, "trap"))
    )
    for k in range(6):
        a = (
            a
            .assert_no_interference(mtr, _rail(base, f"radial_inner_{k}"))
            .assert_no_interference(mtr, _rail(base, f"radial_outer_{k}"))
            .assert_no_interference(mtr, _rail(base, f"inner_horiz_{k}"))
        )

    # Motor mount is a plate at z ∈ [-25, -20]; it bolts up into the two
    # inner radials flanking the drive azimuth, whose bottom faces it
    # meets at z = -20, and clears the others + the column-side stack
    # (all at z ≥ 0). Its pads (r ≤ 117) stay in those rails' free inner
    # span, short of the inner-vertex brackets' inboard arms, whose gap
    # is measured.
    for k in mount_rail_ks:
        a = (
            a
            .assert_tangent_contact(mount, _rail(base, f"radial_inner_{k}"))
            .assert_distance(
                mount, _rail(base, f"inner_vertex_bracket_{k}"), min_mm=1.0,
                name=f"motor_mount_clears_inner_bracket_{k}",
            )
        )
    for k in range(6):
        a = (
            a
            .assert_no_interference(mount, _rail(base, f"radial_inner_{k}"))
            .assert_no_interference(mount, _rail(base, f"radial_outer_{k}"))
            .assert_no_interference(mount, _rail(base, f"inner_horiz_{k}"))
        )
    for other in (
        _rot(base, "column"),
        _brg(base, "ring_bearing"),
        _brg(base, "bearing_spider"),
    ):
        a = a.assert_no_interference(mount, other)

    return a


def _add_catch_checks(a: Assembly, base: int) -> Assembly:
    """The fall path out of the base column and the bottom catch, against
    the frame. The keep-outs are declared in the base_floor builders
    (`drive_train.fall_path`: the vertical fall at every column azimuth;
    `catch.pull_path`: the tray drawn out toward −Y, standing on the
    shelf); the parts they must clear are
    composed here. The ring gear turns with the column, so it is not
    held against the fall path — its hole is the column's
    (`check_base_fall_path.py`). The drive train is held by its own
    claim in `build_drive_train`."""
    tray = "catch.catch_tray"
    cradle = "catch.catch_cradle"
    frame = [f"f{base}", "columns"]
    a = (
        a
        .assert_clear_of(
            frame + ["catch"], "drive_train.fall_path",
            min_mm=_base.FALL_CLEAR, name="base_fall_path",
        )
        .assert_clear_of(
            frame + ["drive_train", cradle], "catch.pull_path",
            min_mm=_base.CATCH_CURB_GAP, name="catch_pull_path",
        )
        .assert_no_interference_between("catch", frame + ["drive_train"])
    )
    # Each post's top face bolts up to its rail's bottom face, between
    # the spider's saddle (on top) and the inner-vertex bracket's
    # inboard arm (under), whose gap is measured.
    for az in _base.CATCH_POST_AZIMUTHS_DEG:
        k = round(az / 60.0 - 0.5) % 6
        a = (
            a
            .assert_tangent_contact(cradle, _rail(base, f"radial_inner_{k}"))
            .assert_distance(
                cradle, _rail(base, f"inner_vertex_bracket_{k}"), min_mm=1.0,
                name=f"catch_post_{k}_clears_inner_bracket",
            )
        )
    return a


# --- Public composition ------------------------------------------------

def build_tower(
    floor_plan: list[tuple[int, str]],
    *,
    schema: str = DEFAULT_SCHEMA,
    basket_layout=FLOOR_LAYOUTS,
    with_top_deck: bool = True,
    with_drive_train: bool = True,
    through_posts: dict[int, float] | None = None,
) -> Assembly:
    """Compose a hex-tower diverter assembly with cross-floor +
    drive-train + top-deck checks, parameterised on which floors,
    baskets, and capping features are present.

    `floor_plan` is a list of ``(idx, role)`` tuples in stacking order
    (lowest first). The exhibit variant passes a reduced plan; the
    production tower passes ``FLOOR_PLAN`` and gets the default
    ``FLOOR_LAYOUTS``, a different basket layout on every floor.

    `basket_layout` is either a single layout list shared across every
    floor or a ``dict[floor_idx, layout]`` keyed by floor index —
    production (`FLOOR_LAYOUTS`) and the article exhibit, which stages
    a few named baskets per floor.

    `with_top_deck` and `with_drive_train` strip those features for
    the exhibit (`with_drive_train` takes the bottom catch with it — its
    fall path is the drive train's). Drive-train assertions and top-deck assertions are
    skipped accordingly — they reference parts that wouldn't exist.

    `through_posts` maps outer-vertex index → height above the deck
    top for columns that continue through the deck as the m02
    gantry's posts (`params.THROUGH_DECK_POSTS` in production; the
    exhibits pass none). Requires the top deck.
    """
    posts = dict(through_posts or {})
    if posts and not with_top_deck:
        raise ValueError("through_posts need the top deck to pass through")
    def _layout_for(idx: int):
        if isinstance(basket_layout, dict):
            return basket_layout[idx]
        return basket_layout

    top_z = (max(idx for idx, _ in floor_plan) + 1) * FLOOR_H
    deck_top_z = top_z + TOP_DECK_T

    a = Assembly().with_subassembly(
        "columns",
        _frame.build_through_columns(
            z_bot=-_frame.VERT_EXT_BELOW, z_top=top_z,
            material_fn=material_of,
            outer_z_top={k: deck_top_z + h for k, h in posts.items()},
        ),
    )

    n = len(floor_plan)
    rotating = Assembly()
    for idx, role in floor_plan:
        fz_loc = Pos(0, 0, idx * FLOOR_H)
        a = a.with_subassembly(
            f"f{idx}",
            _build_floor_static(role, _layout_for(idx)),
            location=fz_loc,
        )
        rotating = rotating.with_subassembly(
            f"f{idx}",
            _ramp.build_column_side_rotating(
                role, exit_w=ramp_exit_w_for(_layout_for(idx)),
                material_fn=material_of,
            ),
            location=fz_loc,
        )

    if with_top_deck:
        a = _add_top_deck(a, top_z=top_z, posts=posts)
        if posts:
            a = _add_top_cap(a, top_z=top_z, posts=posts)

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
        base_idx = next(idx for idx, role in floor_plan if role == "base")
        a = a.with_subassembly(
            "drive_train",
            _base.build_drive_train(
                material_fn=material_of,
            ),
            location=Pos(0, 0, base_idx * FLOOR_H),
        )
        a = a.with_subassembly(
            "catch",
            _base.build_bottom_catch(material_fn=material_of),
            location=Pos(0, 0, base_idx * FLOOR_H),
        )

    # Adjacent-floor pairs only. Non-adjacent floors are >1·FLOOR_H
    # apart and the servo — the highest rotating part outside the
    # column — tops out ~27 mm above the joint plane, so floor i+2 is
    # comfortably out of range. Cross-subassembly
    # references resolve through ``placed_parts`` at evaluation time.
    floors = [idx for idx, _ in floor_plan]
    for i in range(n - 1):
        a = _add_floor_pair_checks(a, floors[i], floors[i + 1])

    if with_top_deck:
        a = _add_top_deck_checks(a, floors[-1], posts)

    base_idx = next((idx for idx, role in floor_plan if role == "base"), None)
    if base_idx is not None:
        a = _add_base_bearing_checks(a, base_idx)
    if with_drive_train:
        a = _add_drive_train_checks(a, base_idx)
        a = _add_catch_checks(a, base_idx)

    # Paint last, once the tree is complete: ``paint`` also carries the
    # coverage check that ``_assert_design_system_coverage`` used to do
    # mid-build, and it can only see parts already composed. ``color_for``
    # applies the basket size ramp alongside the plain tag reads.
    return paint(a, design_system(schema), color_for=color_for)


# --- Production assembly -----------------------------------------------

# Exported interface anchors (m05-local; = world while m05 sits at
# world identity). ``deck_top``: hub-axis point on the top deck's top
# face — the plane the m03 turret floor rests on. ``outer_vert_0_top``:
# vertex-0 outer through-column axis at the deck-top plane (vertex
# azimuths are ``(k + 0.5)·60°`` per ``frame/assembly.py``, so k=0 is
# 30°) — the face the m02 chain's col1 extension butts against.
_DECK_TOP_Z = TOP_VERT_Z + TOP_DECK_T


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
def build_m05(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The production hex tower with its exported interface anchors,
    painted under ``schema`` — the m05 unit as the top level composes it.

    ``build_tower`` is the parameterised form (used by the exhibit
    variants); this is the production floor plan plus the two anchors
    peers derive their placement from.
    """
    a = (
        build_tower(FLOOR_PLAN, schema=schema, through_posts=THROUGH_DECK_POSTS)
        .with_anchor("deck_top", Pos(0, 0, _DECK_TOP_Z))
        .with_anchor(
            "outer_vert_0_top",
            Pos(
                R_VERT_OUTER * math.cos(math.radians(30.0)),
                R_VERT_OUTER * math.sin(math.radians(30.0)),
                _DECK_TOP_Z,
            ),
        )
    )
    # Post tops: the m02 gantry derives its post-top joints from these
    # (`outer_vert_4_top` at deck + 449.5, `outer_vert_5_top` at deck +
    # 160.5 — `THROUGH_DECK_POSTS`) instead of mirroring the heights.
    for k, h in THROUGH_DECK_POSTS.items():
        a = a.with_anchor(
            f"outer_vert_{k}_top",
            _post_vertex_pose(k, _DECK_TOP_Z + h),
        )
    return a


# Degenerate memoized master, so ``khana check <file>`` resolves
# without a ``:factory``.
assembly = build_m05(DEFAULT_SCHEMA)

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
  (`ramp_mechanism.build_column_side_rotating`). Roles differ at the
  column's ends: base has a bottom collar (motor + ring-gear
  interface); middle/top have a bottom face flange that mates with
  the column below at the joint plane (z_world = N·220 + 273); top has
  the feed flare in place of the top joint flange. Each
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
  cut — a closed hole, or a notch from the rim when the deck is too
  small to leave a strip outside the hole — so the column and the
  bracket's vertical arm pass through. OD is `params.TOP_DECK_OD`.
- `top_cap` — the two outer-vertex brackets under the deck at the
  post vertices: the same printed part the floors use, in the same
  relationship to the deck as to a floor's rails (hub + arms under
  the plate, vertical arm rising past it to clamp the column above).
  The other four outer vertices and all six inner ones keep the deck
  resting on the column top; their top-cap bracket variant is still
  open (`_notes/state/m05-diverter.md` §Top deck).
- `ceiling_sleeve` — the fifth bearing slot: a printed sleeve ring set
  in the deck's central aperture round the top column's rotating
  collar, with a lip under the deck (root part; the
  CEILING_SLEEVE_* parameters). Three M3 screws come down through the
  deck into heat-set inserts in the lip (`sleeve_{i}_screw` /
  `_washer` / `_insert`, root parts; the DECK_* fastener family), and
  their heads and washers are the keep-out `deck_fasteners` the top
  level holds clear of what stands on the deck.

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
from .electronics import assembly as _elec
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
from .params import (
    CABLE_DROP_D, CABLE_DROP_R, CABLE_DROP_Z_BOT, CABLE_LOOP_H,
    CABLE_LOOP_R_OUT, CABLE_LOOP_Z_BOT, COLLAR_CLEARANCE, FLOOR_H,
    FLOOR_LAYOUTS, R_VERT_OUTER, THROUGH_DECK_POSTS, TOP_DECK_OD, basket_name,
)
from .ramp_mechanism import assembly as _ramp


# Ramp sweep clearance over the basket rim plane is declared once in
# `ramp_mechanism.build_column_side_rotating` (`ramp_sweep_over_basket_rim`,
# a Z-invariant datum-plane assert_distance) and propagates here
# qualified per floor — no mirrored parametric assert needed.


# --- Top deck ----------------------------------------------------------
#
# Clear-acrylic disc capping the tower. The top feed is m03's DUMP
# into the flare on the top column (`ramp_mechanism` FLARE_*; the m03
# side is `cad/check_dump_feed.py`). The ceiling sleeve is below —
# see `_notes/state/m05-diverter.md` §"Top deck — open detail".
# The OD is `params.TOP_DECK_OD` (a knob the top level tunes).
TOP_DECK_T = 10.0
TOP_DECK_APERTURE_D = 150.0  # clears top column's rotating collar (OD 138 at z 880..885)

# Through-deck posts: the deck is cut at each post vertex so the 2020
# column and the bracket's vertical arm (on the column's outward face,
# 6 mm plate) pass through with this much clearance per side. The cut
# is a closed hole when the acrylic left outside it is at least a sheet
# thickness wide (`_post_hole_runs_out`); thinner than that the strip
# is a crack waiting to happen, so the cut runs out through the rim as
# a notch instead. At Ø660 (rim 330) the arm's outer face sits at
# r ≈ 328.8 and the strip would be ~1 mm: a notch (owner 2026-09-15).
# At the grown OD for the top feed it is a closed hole (owner
# 2026-10-06).
TOP_DECK_POST_CLEAR = 1.0
# Floor on the gap between the deck and the flare turning over it. The
# top column's trap servo stands in that gap; this is not a claim that
# a hand reaches it.
TOP_DECK_FLARE_CLEAR = 20.0
_NOTCH_PAST_RIM = 10.0       # how far a run-out notch carries past the rim

# --- Ceiling sleeve ----------------------------------------------------
#
# The fifth bearing slot. The top column's rotating collar (world z
# 880..885) turns inside the deck's thickness, so its sleeve is set in
# the deck's aperture instead of a spider: a printed ring whose barrel
# fills the aperture round the collar, bore `ramp_mechanism`'s
# RING_BEARING_ID (the other floors' sleeve bore: COLLAR_CLEARANCE
# radial, on grease), and whose lip is *under* the deck — the top
# column's trap coupler turns 0.85 above the deck top, so nothing may
# stand proud of it. The barrel is as tall as the collar (BEARING_T, as
# every other sleeve), which leaves the upper half of the aperture
# empty under the coupler.
#
# Retention (owner 2026-10-17): screws down through the deck into
# heat-set inserts in the lip (the deck fasteners below). The sleeve
# goes in from below until the lip seats on the deck's underside, and
# the screws hold it there. CEILING_SLEEVE_FIT_GAP is the modelled
# radial gap in the aperture: 0 is line-to-line (a push fit) — it
# locates the sleeve; it no longer has to hold it up.
CEILING_SLEEVE_FIT_GAP = 0.0
# The screw heads stand on the deck top, where the top column's trap
# servo turns over them 4.8 above the deck; its reach is r ≈ 95, so the
# screw circle sits outside it (held at CEILING_SLEEVE_TURN_CLEAR by
# `sleeve_screw_*_under_*`). Three screws, kept off +Y, where m03's
# floor cross bar stands on the deck from y 115.
CEILING_SLEEVE_SCREW_R = 106.0
CEILING_SLEEVE_SCREW_AZ = (30.0, 150.0, 270.0)
# Printed wall left round an insert's pilot hole out to the lip's rim.
CEILING_SLEEVE_INSERT_WALL = 5.0
# The lip carries the inserts: at least as thick as an insert is long
# (`ceiling_sleeve` holds it), and wide enough to take the screw circle
# with its insert and that wall (CEILING_SLEEVE_LIP_W, derived after the
# deck fasteners below).
CEILING_SLEEVE_LIP_T = 6.0
# Print orientation, in the part's own frame (lip down, as installed):
# the lip's underside is the bed and the barrel steps in above it.
CEILING_SLEEVE_UP_AXIS = (0.0, 0.0, 1.0)
# Floor on the gap to what turns over the sleeve — the top column's
# trap servo, horn and coupler, and the flare (as FLARE_DRIVE_CLEARANCE).
CEILING_SLEEVE_TURN_CLEAR = 5.0
# Floor on the gap to the frame under the deck (inner column tops, the
# top-cap brackets): room for the lip to grow, not an access claim.
CEILING_SLEEVE_FRAME_CLEAR = 10.0

# --- Deck fasteners ----------------------------------------------------
#
# Bought, one family for every screw through the acrylic: an M3
# socket-head cap screw (ISO 4762) on a wide plain washer (ISO 7093,
# "DIN 9021" — spreads the clamp on acrylic, which crazes under a bare
# head), down through a clearance hole in the deck (acrylic does not
# tap) into an M3 heat-set insert in the printed part below. Cheap
# envelopes: head and washer as cylinders, the shank at the thread's
# major Ø (an outer envelope of the thread), the insert a plain sleeve
# at its pilot-hole Ø with the thread's major Ø as its bore. A
# substitute is a change here: the deck holes (DECK_HOLE_D), the
# inserts' pilot holes (DECK_INSERT_D), the lip's thickness and width,
# the screw-length check in `_add_deck_fastener` and the edge-distance
# floor all derive from these.
DECK_SCREW_D = 3.0          # M3 thread major Ø — the shank envelope
DECK_SCREW_HEAD_D = 5.5     # ISO 4762 M3 dk
DECK_SCREW_HEAD_H = 3.0     # ISO 4762 M3 k
DECK_SCREW_L = 16.0         # under-head length, a stock size
DECK_WASHER_OD = 9.0        # ISO 7093 M3 d2
DECK_WASHER_ID = 3.2        # ISO 7093 M3 d1
DECK_WASHER_T = 0.8         # ISO 7093 M3 h
DECK_INSERT_D = 4.0         # pilot-hole Ø of a short M3 heat-set insert
DECK_INSERT_L = 5.7         # its length
DECK_HOLE_D = 3.4           # ISO 273 medium clearance for M3, in the acrylic
# Edge distance of a hole in the acrylic, to the rim, the aperture or
# another hole: a hole near an edge is where cast acrylic cracks. The
# common rule is 2× the hole's Ø (6.8 here); the deck's post holes
# already keep no strip thinner than the sheet (`_post_hole_runs_out`),
# and the stricter of the two governs.
DECK_HOLE_EDGE_MIN = max(2 * DECK_HOLE_D, TOP_DECK_T)

CEILING_SLEEVE_LIP_W = (
    CEILING_SLEEVE_SCREW_R + DECK_INSERT_D / 2 + CEILING_SLEEVE_INSERT_WALL
    - TOP_DECK_APERTURE_D / 2
)  # 38: the lip's reach past the aperture, radial

# The `off_deck` keep-out (see `_add_top_deck`): a band on the deck-top
# plane from the rim outward, this tall and this wide.
_OFF_DECK_BAND_H = 1.0
_OFF_DECK_BAND_W = 200.0


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
# cad_khana.render photo-real path uses ``acrylic_clear`` from its material
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


_POST_HOLE_X_IN = EXT_SIZE / 2 + TOP_DECK_POST_CLEAR
_POST_HOLE_X_OUT = -(EXT_SIZE / 2 + OUTER_BRACKET_PLATE_T + TOP_DECK_POST_CLEAR)
_POST_HOLE_W = EXT_SIZE + 2 * TOP_DECK_POST_CLEAR


def _post_hole_runs_out(deck_od: float) -> bool:
    """True when the acrylic left between a post hole's outer corners
    and the rim would be narrower than a sheet thickness — the hole
    then runs out through the rim as a notch."""
    corner_r = math.hypot(R_VERT_OUTER - _POST_HOLE_X_OUT, _POST_HOLE_W / 2)
    return deck_od / 2 - corner_r < TOP_DECK_T


def _post_notch(deck_od: float) -> Part:
    """Deck cut-out at a post vertex, in the bracket's local frame
    (centred on the column axis, +X inward, -X outward to the rim):
    clears the column's inward face and sides by TOP_DECK_POST_CLEAR
    and the bracket's vertical arm on the outward face by the same. A
    closed hole, or a notch out through the rim when the strip outside
    it would be thinner than the sheet (`_post_hole_runs_out`)."""
    x_out = _POST_HOLE_X_OUT
    if _post_hole_runs_out(deck_od):
        x_out = -(deck_od / 2 - R_VERT_OUTER + _NOTCH_PAST_RIM)
    x_in = _POST_HOLE_X_IN
    return Pos((x_in + x_out) / 2, 0, 0) * Box(x_in - x_out, _POST_HOLE_W, TOP_DECK_T * 2)


def _circle_points(r: float, azimuths_deg) -> list[tuple[float, float]]:
    """(x, y) of each azimuth (degrees from +X) on a circle of radius r."""
    return [
        (r * math.cos(math.radians(az)), r * math.sin(math.radians(az)))
        for az in azimuths_deg
    ]


def _fastener_holes(fastener_xy, hole_d: float = DECK_HOLE_D) -> list[Part]:
    """The deck's clearance holes, in deck-local coordinates (z = 0 the
    sheet's mid-plane), one per deck fastener at (x, y)."""
    return [Pos(x, y, 0) * Cylinder(hole_d / 2, TOP_DECK_T * 2) for x, y in fastener_xy]


def deck_holes(deck_od: float, posts: dict[int, float], fastener_xy) -> dict[str, Part]:
    """Every hole cut in the deck sheet but its central aperture, by
    name, in deck-local coordinates: the post holes (`post_hole_{k}`)
    and the fastener clearance holes (`fastener_hole_{i}`, in the order
    of `fastener_xy`)."""
    holes = {
        f"post_hole_{k}": _post_vertex_pose(k, 0.0) * _post_notch(deck_od) for k in posts
    }
    for i, h in enumerate(_fastener_holes(fastener_xy)):
        holes[f"fastener_hole_{i}"] = h
    return holes


def hole_edge_distance(deck: Part, hole: Part, deck_od: float) -> float:
    """How much acrylic stands between one hole and the nearest other
    edge of the sheet — its rim, the aperture or another hole. `deck`
    is the cut sheet in deck-local coordinates and `hole` the solid that
    cut this hole: the sheet with this one hole filled back in is
    subtracted from a disc past the rim, which leaves every *other* void,
    and the hole's distance to that is its edge distance. Measured on
    the cut geometry, so it holds for any hole shape."""
    slab = Cylinder(deck_od / 2, TOP_DECK_T)
    others = Cylinder(deck_od / 2 + 10.0, TOP_DECK_T) - (deck + (slab & hole))
    return (slab & hole).distance_to(others)


def _add_top_deck(
    a: Assembly, *, top_z: float, posts: dict[int, float], deck_od: float,
    fastener_xy=(),
) -> Assembly:
    """Clear-acrylic cap disc; bottom face tangent to through-column
    tops, cut at each through-deck post vertex and with a clearance
    hole (DECK_HOLE_D) at each deck fastener's (x, y) in `fastener_xy`.

    Also declares the keep-out ``off_deck``: the band just above the
    deck's top face from its rim outward. It is not a part — a
    composing parent holds whatever stands on the deck clear of it
    (`cad/assembly.py` `on_deck_*`), so a footprint's distance to it
    is its margin inside the rim."""
    top_deck = (
        Cylinder(deck_od / 2, TOP_DECK_T)
        - Cylinder(TOP_DECK_APERTURE_D / 2, TOP_DECK_T * 2)
    )
    # The disc is placed at z = top_z + T/2, so cut in deck-local z.
    for hole in deck_holes(deck_od, posts, fastener_xy).values():
        top_deck = top_deck - hole
    deck_top_z = top_z + TOP_DECK_T
    # A 1 mm band standing on the deck-top plane: a part resting on the
    # deck meets it only with its bottom millimetre, so the distance is
    # the footprint's radial gap to the rim. The outer radius only has
    # to reach past anything that could stand there.
    off_deck = Pos(0, 0, deck_top_z + _OFF_DECK_BAND_H / 2) * (
        Cylinder(deck_od / 2 + _OFF_DECK_BAND_W, _OFF_DECK_BAND_H)
        - Cylinder(deck_od / 2, _OFF_DECK_BAND_H * 2)
    )
    return (
        a.with_part(
            "top_acrylic", top_deck,
            location=Pos(0, 0, top_z + TOP_DECK_T / 2),
            color=_TOP_DECK_COLOR,
            material=material_of("top_acrylic"),
        )
        .with_keepout("off_deck", off_deck)
    )


def ceiling_sleeve(
    aperture_d: float = TOP_DECK_APERTURE_D,
    bore_d: float = _ramp.RING_BEARING_ID,
    barrel_h: float = _ramp.BEARING_T,
    lip_t: float = CEILING_SLEEVE_LIP_T,
    lip_w: float = CEILING_SLEEVE_LIP_W,
    fit_gap: float = CEILING_SLEEVE_FIT_GAP,
    screw_r: float = CEILING_SLEEVE_SCREW_R,
    screw_az: tuple[float, ...] = CEILING_SLEEVE_SCREW_AZ,
    insert_d: float = DECK_INSERT_D,
    insert_l: float = DECK_INSERT_L,
) -> Part:
    """The ceiling sleeve, in its own frame: axis on Z, z = 0 the plane
    of the deck's underside (the lip's top face). The barrel rises
    `barrel_h` into the aperture; the lip hangs `lip_t` below and
    reaches `lip_w` past the aperture. Printed as drawn, lip down
    (`CEILING_SLEEVE_UP_AXIS`).

    The lip carries a heat-set insert's pilot hole, through the lip, at
    each `screw_az` on the `screw_r` circle — the screws come down
    through the deck into them (`_add_deck_fastener`)."""
    lip_r = aperture_d / 2 + lip_w
    assert lip_t >= insert_l, (
        f"sleeve lip {lip_t} thick is shorter than its {insert_l} inserts"
    )
    assert screw_r - insert_d / 2 - aperture_d / 2 >= _ramp.WALL_MIN, (
        f"sleeve insert hole at r {screw_r} breaks into the barrel's aperture"
    )
    assert lip_r - (screw_r + insert_d / 2) >= _ramp.WALL_MIN, (
        f"sleeve insert hole at r {screw_r} leaves under WALL_MIN to the lip's rim r {lip_r}"
    )
    body = _ceiling_sleeve_body(aperture_d, bore_d, barrel_h, lip_t, lip_w, fit_gap)
    for x, y in _circle_points(screw_r, screw_az):
        body = body - Pos(x, y, -lip_t / 2) * Cylinder(insert_d / 2, 2 * lip_t)
    return body


def _ceiling_sleeve_body(
    aperture_d: float, bore_d: float, barrel_h: float,
    lip_t: float, lip_w: float, fit_gap: float,
) -> Part:
    """The sleeve's barrel and lip without the insert holes — see
    `ceiling_sleeve`. `ramp_mechanism.ring_bearing` is not reused: its
    OD is the spider hub's (Ø159.8, wider than the aperture) and it has
    no lip."""
    barrel_r = aperture_d / 2 - fit_gap
    assert fit_gap >= 0.0, f"sleeve fit gap {fit_gap} < 0: model a press fit as a contact claim"
    assert barrel_r - bore_d / 2 >= _ramp.WALL_MIN, (
        f"sleeve barrel wall {barrel_r - bore_d / 2:.2f} under WALL_MIN "
        f"{_ramp.WALL_MIN}: aperture Ø{aperture_d} is too tight round bore Ø{bore_d:.2f}"
    )
    assert 0.0 < barrel_h <= TOP_DECK_T, (
        f"sleeve barrel {barrel_h} tall stands proud of the {TOP_DECK_T} deck"
    )
    assert lip_t >= _ramp.WALL_MIN and lip_w > 0.0, (
        f"sleeve lip {lip_t} thick / {lip_w} wide: no lip to seat under the deck"
    )
    barrel = Pos(0, 0, barrel_h / 2) * Cylinder(barrel_r, barrel_h)
    lip = Pos(0, 0, -lip_t / 2) * Cylinder(aperture_d / 2 + lip_w, lip_t)
    bore = Cylinder(bore_d / 2, 4 * (barrel_h + lip_t))
    return (barrel + lip) - bore


def _add_ceiling_sleeve(a: Assembly, *, top_z: float) -> Assembly:
    """The sleeve in the deck's aperture, lip against the deck's
    underside at `top_z`, with its two features named as regions (in
    the part's own frame): `barrel`, everything above the lip's top
    face, and `lip`, everything below it."""
    name = "ceiling_sleeve"
    slab_r = TOP_DECK_APERTURE_D + CEILING_SLEEVE_LIP_W
    return (
        a.with_part(
            name, ceiling_sleeve(),
            location=Pos(0, 0, top_z),
            material=material_of(name),
        )
        .with_region(
            name, "barrel",
            Pos(0, 0, TOP_DECK_T / 2) * Cylinder(slab_r, TOP_DECK_T),
        )
        .with_region(
            name, "lip",
            Pos(0, 0, -CEILING_SLEEVE_LIP_T / 2) * Cylinder(slab_r, CEILING_SLEEVE_LIP_T),
        )
    )


def sleeve_screw_xy() -> list[tuple[float, float]]:
    """(x, y) of the ceiling sleeve's screws, in the tower frame."""
    return _circle_points(CEILING_SLEEVE_SCREW_R, CEILING_SLEEVE_SCREW_AZ)


def deck_screw(
    d: float = DECK_SCREW_D, head_d: float = DECK_SCREW_HEAD_D,
    head_h: float = DECK_SCREW_HEAD_H, length: float = DECK_SCREW_L,
    washer_t: float = DECK_WASHER_T,
) -> Part:
    """Socket-head cap screw envelope, in its seat's frame: z = 0 the
    face the washer stands on, the head on top of the washer, the shank
    `length` down from the head's underside."""
    head = Pos(0, 0, washer_t + head_h / 2) * Cylinder(head_d / 2, head_h)
    shank = Pos(0, 0, washer_t - length / 2) * Cylinder(d / 2, length)
    return head + shank


def deck_washer(
    od: float = DECK_WASHER_OD, id_: float = DECK_WASHER_ID, t: float = DECK_WASHER_T,
) -> Part:
    """Plain washer, standing on z = 0."""
    return Pos(0, 0, t / 2) * (Cylinder(od / 2, t) - Cylinder(id_ / 2, 2 * t))


def deck_insert(
    d: float = DECK_INSERT_D, bore_d: float = DECK_SCREW_D, length: float = DECK_INSERT_L,
) -> Part:
    """Heat-set insert envelope, hanging from z = 0 (its top, flush with
    the printed face it is set into): OD at the pilot-hole Ø, bore at
    the screw's major Ø."""
    return Pos(0, 0, -length / 2) * (Cylinder(d / 2, length) - Cylinder(bore_d / 2, 2 * length))


def _add_deck_fastener(
    a: Assembly, name: str, x: float, y: float, *, top_z: float,
) -> Assembly:
    """One deck fastener at (x, y): washer and screw on the deck's top
    face, the insert's top flush with the deck's underside at `top_z`
    (the face of the printed part it is set into). Parts
    `{name}_screw`, `{name}_washer`, `{name}_insert`."""
    deck_top_z = top_z + TOP_DECK_T
    # The tip must reach into the insert and stop inside it.
    reach = DECK_SCREW_L - DECK_WASHER_T - TOP_DECK_T
    assert 0.0 < reach <= DECK_INSERT_L, (
        f"a {DECK_SCREW_L} screw reaches {reach:.2f} below the deck: "
        f"outside its {DECK_INSERT_L} insert"
    )
    return (
        a.with_part(
            f"{name}_washer", deck_washer(), location=Pos(x, y, deck_top_z),
            material=material_of(f"{name}_washer"),
        )
        .with_part(
            f"{name}_screw", deck_screw(), location=Pos(x, y, deck_top_z),
            material=material_of(f"{name}_screw"),
        )
        .with_part(
            f"{name}_insert", deck_insert(), location=Pos(x, y, top_z),
            material=material_of(f"{name}_insert"),
        )
    )


def _add_deck_fasteners(a: Assembly, *, top_z: float) -> Assembly:
    """The ceiling sleeve's screws (`sleeve_{i}_*`), and the keep-out
    ``deck_fasteners``: every head and washer standing on the deck top,
    which a composing parent holds clear of what stands on the deck
    (`cad/assembly.py`). Not a part — the same solids as the heads, so
    a stander's distance to it is its distance to the nearest head."""
    deck_top_z = top_z + TOP_DECK_T
    tops = []
    for i, (x, y) in enumerate(sleeve_screw_xy()):
        a = _add_deck_fastener(a, f"sleeve_{i}", x, y, top_z=top_z)
        tops.append(Pos(x, y, deck_top_z) * (deck_washer() + (deck_screw() & Pos(0, 0, 50) * Cylinder(50, 100))))
    keepout = tops[0]
    for t in tops[1:]:
        keepout = keepout + t
    return a.with_keepout("deck_fasteners", keepout)


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
    a: Assembly, top: int, posts: dict[int, float], *, flare: bool,
) -> Assembly:
    """Disc rests on the through-column tops (tangent contact at
    z = top_z) and clears the top floor's central column body +
    rotating collar through its central aperture. Inner-hex
    through-columns sit at r ≈ 162, well inside the disc rim (≥ 330) and
    well outside the aperture R=75.

    At a through-deck post vertex the column does not end at the
    deck, so the rest moves to the bracket under it: the hole (or
    notch) clears the column by TOP_DECK_POST_CLEAR (measured, so a drifted cut
    or a rotated column reads as a number), the deck sits on the
    bracket's hub + arms, and the bracket's vertical arm threads the
    cut without touching it."""
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
    a = (
        a
        .assert_no_interference("top_acrylic", _rot(top, "column"))
        .assert_no_interference("top_acrylic", _rot(top, "trap_servo"))
    )
    if flare:
        # The flare turns over the deck; round, so one reading holds at
        # every azimuth (and `check_stack_turn.py` turns it anyway).
        a = a.assert_distance(
            "top_acrylic", _rot(top, "column") + "@flare",
            min_mm=TOP_DECK_FLARE_CLEAR, name="flare_over_deck",
        )
    return a


def _add_ceiling_sleeve_checks(
    a: Assembly, top: int, posts: dict[int, float], *, flare: bool,
) -> Assembly:
    """The ceiling sleeve's seat in the deck, its running fit round the
    top column's collar, and its clearances — to everything that turns
    with the stack (held over the turn by `check_stack_turn.py`) and to
    the frame under the deck.

    The trap's own joint is not driven here, and need not be for these
    claims: the coupler and the horn are discs round the hinge pin, the
    axis the trap turns them about, so their readings are the same at
    every trap angle (true while both stay round — a horn with an arm
    would need the trap swung)."""
    s = "ceiling_sleeve"
    col = _rot(top, "column")
    a = (
        a
        .assert_solid_count(
            s,
            detail="above 1 the lip has come off the barrel — they join "
                   "on the deck-underside plane; check the bore is not "
                   "wider than the barrel",
        )
        # Seat: the lip's top face on the deck's underside (the stop the
        # sleeve is pushed up to), and the barrel in the aperture at the
        # modelled fit gap — constructed from the same constant as the
        # bound (cad/CLAUDE.md §6, corollary 2).
        .assert_no_interference(s, "top_acrylic")
        .assert_tangent_contact(
            f"{s}@lip", "top_acrylic", name="ceiling_sleeve_lip_under_deck",
        )
        .assert_distance(
            f"{s}@barrel", "top_acrylic",
            min_mm=CEILING_SLEEVE_FIT_GAP, max_mm=CEILING_SLEEVE_FIT_GAP,
            name="ceiling_sleeve_in_aperture",
        )
        # Running fit: the bore stands COLLAR_CLEARANCE off the collar,
        # the gap every other floor's sleeve runs at. Bounded both
        # ways — a sleeve 5 mm off the collar locates nothing.
        .assert_distance(
            s, col, min_mm=COLLAR_CLEARANCE, max_mm=COLLAR_CLEARANCE,
            name="ceiling_sleeve_collar_gap",
        )
        .assert_no_interference_between([s], f"rotating_stack.f{top}")
    )
    for part in ("trap_servo", "trap.trap_coupler", "trap.servo_horn"):
        a = a.assert_distance(
            s, _rot(top, part), min_mm=CEILING_SLEEVE_TURN_CLEAR,
            name=f"ceiling_sleeve_under_{part.split('.')[-1]}",
        )
    if flare:
        a = a.assert_distance(
            s, col + "@flare", min_mm=CEILING_SLEEVE_TURN_CLEAR,
            name="ceiling_sleeve_under_flare",
        )
    # Under the deck: the column tops the deck rests on and the top-cap
    # brackets. (The top floor's own rails and bearing are a floor
    # height below.)
    frame = ["columns"] + (["top_cap"] if posts else [])
    a = a.assert_no_interference_between([s], frame)
    for k in range(6):
        a = a.assert_distance(
            s, f"columns.inner_vert_{k}", min_mm=CEILING_SLEEVE_FRAME_CLEAR,
            name=f"ceiling_sleeve_clears_inner_vert_{k}",
        )
    for k in posts:
        a = a.assert_distance(
            s, f"top_cap.outer_vertex_bracket_{k}", min_mm=CEILING_SLEEVE_FRAME_CLEAR,
            name=f"ceiling_sleeve_clears_vertex_bracket_{k}",
        )
    return a


def _add_deck_hole_checks(
    a: Assembly, *, deck_od: float, posts: dict[int, float], fastener_xy,
) -> Assembly:
    """Every hole in the acrylic — post holes and fastener clearance
    holes — keeps DECK_HOLE_EDGE_MIN of sheet to the rim, the aperture
    and every other hole. Measured on the deck body as cut
    (`hole_edge_distance`) and recorded per hole; the deck is static,
    so the build pose is every pose."""
    deck = a.part("top_acrylic").part
    for name, hole in deck_holes(deck_od, posts, fastener_xy).items():
        a = a.assert_scalar(
            f"deck_{name}_edge_distance", hole_edge_distance(deck, hole, deck_od),
            ge=DECK_HOLE_EDGE_MIN,
            detail="mm of acrylic from this hole to the nearest other edge of the sheet",
        )
    return a


def _add_sleeve_fastener_checks(a: Assembly, top: int, *, flare: bool) -> Assembly:
    """Each ceiling-sleeve screw: washer on the deck, head on the washer,
    shank through the deck's clearance hole without touching it, coaxial
    with its insert, the insert seated in the lip's pilot hole — and the
    head and washer clear of what turns over them by
    CEILING_SLEEVE_TURN_CLEAR (held over the turn by
    `check_stack_turn.py`; the screws are static, the stack turns)."""
    col = _rot(top, "column")
    turning = ["trap_servo", "trap.trap_coupler", "trap.servo_horn"]
    for i in range(len(CEILING_SLEEVE_SCREW_AZ)):
        n = f"sleeve_{i}"
        scr, wsh, ins = f"{n}_screw", f"{n}_washer", f"{n}_insert"
        a = (
            a
            .assert_tangent_contact(wsh, "top_acrylic", name=f"{n}_washer_on_deck")
            .assert_tangent_contact(scr, wsh, name=f"{n}_head_on_washer")
            # Through the clearance hole, centred in it: constructed at
            # exactly the hole's radial clearance, bounded both ways.
            .assert_distance(
                scr, "top_acrylic",
                min_mm=(DECK_HOLE_D - DECK_SCREW_D) / 2,
                max_mm=(DECK_HOLE_D - DECK_SCREW_D) / 2,
                name=f"{n}_screw_through_deck",
            )
            # Coaxial with its insert: the shank fills the insert's bore,
            # so any offset is an overlap and any gap a miss.
            .assert_tangent_contact(scr, ins, name=f"{n}_screw_in_insert")
            .assert_no_interference(scr, ins)
            .assert_tangent_contact(ins, "ceiling_sleeve", name=f"{n}_insert_in_lip")
            .assert_no_interference(ins, "ceiling_sleeve")
            .assert_no_interference(scr, "ceiling_sleeve")
            .assert_no_interference_between([scr, wsh, ins], f"rotating_stack.f{top}")
        )
        for part in turning:
            for p in (scr, wsh):
                a = a.assert_distance(
                    p, _rot(top, part), min_mm=CEILING_SLEEVE_TURN_CLEAR,
                    name=f"{p}_under_{part.split('.')[-1]}",
                )
        if flare:
            for p in (scr, wsh):
                a = a.assert_distance(
                    p, col, min_mm=CEILING_SLEEVE_TURN_CLEAR,
                    name=f"{p}_under_column",
                )
    return a


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


# The servo wires (params.HARNESS_*, CABLE_*). Static parts that turn
# nothing are held this far off the bundle and the loop: the bundle must
# not rub as the stack turns, and the loop's cable flexes inside its
# envelope.
HARNESS_STATIC_CLEAR = 2.0
CABLE_LOOP_CLEAR = 1.0
# A claim selects the parts whose bounding box enters the claim's region
# grown by this much. A part that ever comes within it of the region is
# in that box at every azimuth (a turning point stays on its circle, and
# the box holds the whole disc), so the selection is a superset of every
# part that could read under the margins above.
_SELECT_MARGIN = 15.0


def _parts_near(
    a: Assembly, r: float, z_lo: float, z_hi: float, *, rotating: bool,
) -> list[str]:
    """Part paths — static, or inside `rotating_stack` — whose bounding
    box enters the box round the upright cylinder of radius `r` over
    z_lo..z_hi, grown by _SELECT_MARGIN, in this assembly's frame."""
    r += _SELECT_MARGIN
    z_lo, z_hi = z_lo - _SELECT_MARGIN, z_hi + _SELECT_MARGIN
    out = []
    for p in a.placed_parts:
        if p.name.startswith("rotating_stack.") != rotating:
            continue
        bb = (p.location * p.part).bounding_box()
        if bb.max.Z < z_lo or bb.min.Z > z_hi:
            continue
        if bb.min.X > r or bb.max.X < -r or bb.min.Y > r or bb.max.Y < -r:
            continue
        out.append(p.name)
    return sorted(out)


def _add_harness_checks(
    a: Assembly, floor_plan: list[tuple[int, str]], *, with_drive_train: bool,
) -> Assembly:
    """The servo-wire bundle on each floor's column, held clear of every
    static part (it turns with the stack — `check_stack_turn.py` holds
    these over the turn); and, with the drive train, the base's loop and
    static drop held clear of everything but the bundle's own lead-in.
    The keep-outs are declared where their geometry is known
    (`ramp_mechanism.build_column_side_rotating`'s `harness_path`,
    `base_floor.build_drive_train`'s `cable_loop` / `cable_drop`)."""
    for idx, role in floor_plan:
        bb = _ramp.harness(role).bounding_box()
        reach = math.hypot(
            max(abs(bb.min.X), abs(bb.max.X)), max(abs(bb.min.Y), abs(bb.max.Y)),
        )
        z0 = idx * FLOOR_H
        a = a.assert_clear_of(
            _parts_near(a, reach, z0 + bb.min.Z, z0 + bb.max.Z, rotating=False),
            f"rotating_stack.f{idx}.harness_path",
            min_mm=HARNESS_STATIC_CLEAR, name=f"harness_f{idx}_clear_of_static",
        )
    if not with_drive_train:
        return a
    base = next(idx for idx, role in floor_plan if role == "base")
    z0 = base * FLOOR_H
    lead_in = [_rot(base, "harness")]
    for kp, r, z_lo, z_hi in (
        ("cable_loop", CABLE_LOOP_R_OUT, CABLE_LOOP_Z_BOT, CABLE_LOOP_Z_BOT + CABLE_LOOP_H),
        ("cable_drop", CABLE_DROP_R + CABLE_DROP_D / 2, CABLE_DROP_Z_BOT, CABLE_LOOP_Z_BOT),
    ):
        for rotating, label in ((False, "static"), (True, "stack")):
            sel = [
                p for p in _parts_near(a, r, z0 + z_lo, z0 + z_hi, rotating=rotating)
                if p not in lead_in
            ]
            a = a.assert_clear_of(
                sel, f"drive_train.{kp}",
                min_mm=CABLE_LOOP_CLEAR, name=f"{kp}_clear_of_{label}",
            )
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


def _add_electronics_checks(a: Assembly, base: int) -> Assembly:
    """The electronics plates under the base floor, against everything
    static near them. Each plate's own rests and reach slot are declared
    in `electronics.build_electronics`; here: the hangers bolt up to
    their sector's rails, nothing of the electronics meets the frame,
    the drive train or the catch, the catch still draws out and the
    cable drop still runs, each reach slot is open to the frame, and no
    plate hangs below the catch (the clearance a stand already
    leaves)."""
    static = [f"f{base}", "columns", "drive_train", "catch"]
    a = a.assert_no_interference_between("electronics", static)
    for side, face in (("logic", _elec.LOGIC_FACE_DEG), ("power", _elec.POWER_FACE_DEG)):
        k = _elec.face_rail_index(face)
        a = (
            a
            .assert_tangent_contact(f"electronics.{side}.hanger_in", _rail(base, f"inner_horiz_{k}"))
            .assert_tangent_contact(f"electronics.{side}.hanger_out_pos", _rail(base, f"outer_horiz_{k}"))
            .assert_tangent_contact(f"electronics.{side}.hanger_out_neg", _rail(base, f"outer_horiz_{k}"))
            .assert_clear_of(
                static + [f"electronics.{other}" for other in ("logic", "power") if other != side],
                f"electronics.{side}.service_reach", name=f"electronics_{side}_reach_open",
            )
        )
    a = (
        a
        .assert_clear_of("electronics", "catch.pull_path", name="electronics_clear_of_catch_pull")
        .assert_clear_of("electronics", "drive_train.cable_drop", min_mm=5.0,
                         name="electronics_clear_of_cable_drop")
    )
    placed = {pp.name: pp for pp in a.placed_parts}
    def _min_z(name: str) -> float:
        pp = placed[name]
        return (pp.location * pp.part).bounding_box().min.Z
    floor = _min_z("catch.catch_cradle")
    for side in ("logic", "power"):
        a = a.assert_scalar(
            f"electronics_{side}_above_catch_bottom_mm",
            _min_z(f"electronics.{side}.board_plate") - floor, ge=0.0,
            detail="plate underside above the catch cradle's: the stand clearance does not deepen",
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
    top_deck_od: float = TOP_DECK_OD,
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

    `top_deck_od` is the deck disc's OD (`params.TOP_DECK_OD`).
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
        a = _add_top_deck(
            a, top_z=top_z, posts=posts, deck_od=top_deck_od,
            fastener_xy=sleeve_screw_xy(),
        )
        a = _add_ceiling_sleeve(a, top_z=top_z)
        a = _add_deck_fasteners(a, top_z=top_z)
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
        a = a.with_subassembly(
            "electronics",
            _elec.build_electronics(material_fn=material_of),
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
        # Only a top-role column carries the flare (and its region).
        a = _add_top_deck_checks(
            a, floors[-1], posts, flare=floor_plan[-1][1] == "top",
        )
        a = _add_ceiling_sleeve_checks(
            a, floors[-1], posts, flare=floor_plan[-1][1] == "top",
        )
        a = _add_sleeve_fastener_checks(
            a, floors[-1], flare=floor_plan[-1][1] == "top",
        )
        a = _add_deck_hole_checks(
            a, deck_od=top_deck_od, posts=posts, fastener_xy=sleeve_screw_xy(),
        )

    base_idx = next((idx for idx, role in floor_plan if role == "base"), None)
    if base_idx is not None:
        a = _add_base_bearing_checks(a, base_idx)
    if with_drive_train:
        a = _add_drive_train_checks(a, base_idx)
        a = _add_catch_checks(a, base_idx)
        a = _add_electronics_checks(a, base_idx)
    a = _add_harness_checks(a, floor_plan, with_drive_train=with_drive_train)

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
def build_m05(schema: str = DEFAULT_SCHEMA, top_deck_od: float = TOP_DECK_OD) -> Assembly:
    """The production hex tower with its exported interface anchors,
    painted under ``schema`` — the m05 unit as the top level composes it.

    ``build_tower`` is the parameterised form (used by the exhibit
    variants); this is the production floor plan plus the two anchors
    peers derive their placement from.

    ``top_deck_od`` defaults to ``params.TOP_DECK_OD``; a composing
    parent may pass its own to size the deck for what stands on it.
    """
    a = (
        build_tower(
            FLOOR_PLAN, schema=schema, through_posts=THROUGH_DECK_POSTS,
            top_deck_od=top_deck_od,
        )
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

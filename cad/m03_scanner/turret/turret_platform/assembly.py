"""
M03 scanner — rev-3 turret platform cartridge (slice d of the 4-station
turret).

Single platform "cartridge": PMMA acrylic plate + 2 bonded hinge
brackets near the inner (hub-side) edge + 1 bonded perimeter ring on
the top face + 1 bonded kicker tab on the underside (slice f4). Four
identical instances ride the slice (c) spider in operation; this slice
ships ONE cartridge in its IMAGE-station pose (world centroid at
origin, top face at y = 0). Slice (f) integrate replicates it at the
other three stations via 90° rotations around the hub axis.

Members (5 total — 1 acrylic + 4 printed bonded pieces):
  * acrylic — 180 × 180 × 4 mm PMMA, top face at y = 0, centred on
    origin in X / Z. Same stock + thickness as the rev-3
    platform_acrylic; reuses PLATFORM_ACRYLIC_SIZE / PLATFORM_ACRYLIC_T
    so a revision in one place propagates everywhere.
  * bracket_pos_x, bracket_neg_x — two printed hinge brackets bonded
    to the acrylic underside, straddling the slice (c) IMAGE pin's
    bushing block in X. The IMAGE pin axis runs along world ±X at
    (y = -22.5, z = -90). Slice (c)'s bushing block occupies world
    X ∈ [-10, +10] (block_w = 20 along the pin axis); the pin
    protrudes 5 mm past each face to world X = ±15. The two brackets
    sit at world X = ±13 (centerline), each engaging one pin-tip
    protrusion zone. A single printed body fits both via X-mirror
    symmetry.
  * ring — printed square frame bonded to the acrylic top. Outer
    perimeter matches the acrylic (180 × 180); inner opening 172 ×
    172 to keep the Ø172 below-mirror sightline cone (per design
    brief 4.2) clear. Frame width 4 mm, height 5 mm above the
    acrylic top. Edge containment only — the brick can wander within
    the 172 × 172 zone; centring is deferred to either banking via
    slice (e)'s cam profile, a transparent silicone insert, or both.
  * kicker_tab — printed block bonded to the acrylic underside,
    positioned to sit above the slice (f2) bench-mounted lifter pad
    when the cartridge is at the DUMP station. The slice (f2) lever
    cantilevers from the +Z side of its bench mount, which at DUMP
    maps to cartridge +X via Ry(+90); the lifter pad therefore lands
    at cartridge X ∈ [+16, +28], Z ∈ [-15, +15] (cartridge-local at
    DUMP). The tab footprint sits inside that with margin
    (cartridge X ∈ [+18, +26], Z ∈ [-10, +10]) so the lifter has
    overhang on all sides for misalignment. Tab bottom face at
    cartridge Y = -43 (TAB_LIFTER_CLEARANCE_Y = 2 mm above the
    lifter top at REST y = -45); at the cam peak the lifter rises
    and pushes the tab up, tilting the platform outer-up around its
    hinge axis so the brick slides off into the diverter. The tab
    is necessarily inside the Ø172 below-mirror sightline cone
    (kicker geometry forces it), accepted because it's on every
    platform and the classifier learns to ignore the fixed feature.
    Outboard of the hinge brackets in cartridge X (brackets ±13,
    tab +22) and Z (brackets at z ≈ -90, tab at z = 0).

Coordinate frame = cartridge-local IMAGE pose:
    origin = brick centroid on platform top face
    +X     = along pin axis
    +Y     = up
    +Z     = radial outward from hub

`add_turret_platform(a, station)` translates and rotates this
cartridge to each station in turret-local: IMAGE at (0, 0, +R) with
no rotation, LOAD at (+R, 0, 0) with Ry(+90), DUMP at (-R, 0, 0)
with Ry(-90), RETURN at (0, 0, -R) with Ry(180). Same shape, four
poses.

Bond joint: each printed piece sits BOND_GAP (0.3 mm) away from the
acrylic face it bonds to — same idiom as platform_acrylic. The gap is
filled by structural-acrylic epoxy in reality and keeps the modelled
cross-boundary interference check at clean zero.

Bracket-local frame (one bracket; cartridge placement applies
translation + X-mirror for the neg_x instance):
    origin = bottom-face centre of the bracket block
    +X     = along pin axis            (= cartridge +X)
    +Y     = up toward acrylic underside
    +Z     = perpendicular to pin in horizontal plane
             (= cartridge ±Z, hub-side / outward-side)

Cantilever note: each bracket extends in Z slightly past the acrylic
inner edge (acrylic spans cartridge z ∈ [-90, +90]; bracket centerline
at z = -90 with z-extent 16 mm → bracket spans z ∈ [-98, -82]). The
half from z = -98 to z = -90 sits in open air past the acrylic edge —
the unbonded cantilever zone where the pin bore lives. The half from
z = -90 to z = -82 sits under the acrylic in the bond zone (8 × 5 =
40 mm² per bracket; small but two brackets share the load and the
bond is in shear, not peel).

Print orientation per piece:
  * brackets — bottom face on build plate (+Y up). Horizontal pin
    bore (Ø5.4) bridges fine in FDM; threshold raised to 90° (same
    rationale as platform_hinge_bearing).
  * ring — bottom face on build plate (+Y up). Square frame, no
    bores or overhangs; default 45° threshold suffices.
  * kicker_tab — bottom face on build plate (+Y up). Solid box, no
    bores or overhangs; default 45° threshold suffices.
"""

import sys
from pathlib import Path

from build123d import Box, Cylinder, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from turret.params import (  # noqa: E402
    PLATFORM_ACRYLIC_SIZE,
    PLATFORM_ACRYLIC_T,
    TURRET_ARM_R,
    TURRET_DUMP_KICKER_LIFTER_TOP_Y,
    TURRET_HINGE_PIN_D,
    TURRET_HINGE_PIN_RADIAL,
    TURRET_HINGE_PIN_Y,
    WALL_MIN,
)


# --- Cartridge-local IMAGE-pose references -----------------------------

# In cartridge-local, the IMAGE pin axis runs along ±X. Y matches
# turret-local Y (vertical). Z is "radial outward from hub" at IMAGE
# pose, so the hub-side of the acrylic sits at -Z.
PIN_AXIS_Y_LOCAL = TURRET_HINGE_PIN_Y                          # = -22.5
# Cartridge centroid at the brick centroid (origin); the pin sits at
# the hub-side edge of the acrylic, RADIAL short of the hub.
# Cartridge-local hub Z = -ARM_R when placed at IMAGE (centroid at
# turret +R), so cartridge-local pin Z = -ARM_R + RADIAL.
PIN_AXIS_Z_LOCAL = TURRET_HINGE_PIN_RADIAL - TURRET_ARM_R      # = -90

# Modelled gap between any printed piece and the acrylic face it
# bonds to. Filled by structural-acrylic epoxy in real assembly.
BOND_GAP = 0.3


# --- Hinge bracket dimensions ------------------------------------------

# Bracket-local axes:
#   +X = along pin axis        (= cartridge +X)
#   +Y = up                    (= cartridge / world +Y)
#   +Z = perpendicular to pin  (= cartridge ±Z)
BRACKET_BLOCK_X = 5.0          # spans one pin-tip protrusion zone
BRACKET_BLOCK_Y = 23.0         # bottom up to bond-side top face
BRACKET_BLOCK_Z = 16.0         # spans across the pin axis

# Pin bore diameter — Ø5.4 = 0.4 mm diametric clearance over the M5
# stock pin from slice (c).
PIN_BORE_D = TURRET_HINGE_PIN_D + 0.4

# Bracket bottom Y in cartridge-local. Acrylic underside at
# y = -PLATFORM_ACRYLIC_T, minus BOND_GAP for the bond layer, minus
# block height to land the bottom face.
BRACKET_BOTTOM_Y_LOCAL = -PLATFORM_ACRYLIC_T - BOND_GAP - BRACKET_BLOCK_Y

# Pin axis Y in bracket-local frame. Cartridge pin Y minus bracket
# bottom Y.
_PIN_Y_BRACKET_LOCAL = PIN_AXIS_Y_LOCAL - BRACKET_BOTTOM_Y_LOCAL

# Slice (c) bushing block half-width along the pin axis (mirrors
# slice (c)'s BUSHING_BLOCK_W = 20).
_BUSHING_HALF_X = 10.0
_PIN_AIR_GAP = 0.5             # bracket-to-bushing-block side clearance

# Each bracket centred at cartridge X = ±(bushing half + air gap +
# bracket half) — its inboard face sits AIR_GAP outboard of the
# bushing block, and the bracket spans the rest of the pin tip
# (pin tip at cartridge X = ±15 = bushing 10 + protrusion 5).
BRACKET_X_CENTRE_OFFSET = (
    _BUSHING_HALF_X + _PIN_AIR_GAP + BRACKET_BLOCK_X / 2          # = 13
)
BRACKET_POS_X_LOCAL = +BRACKET_X_CENTRE_OFFSET
BRACKET_NEG_X_LOCAL = -BRACKET_X_CENTRE_OFFSET

# Bracket centerline Z in cartridge-local = pin axis Z (so the
# horizontal bore along ±X passes through the bracket centerline).
BRACKET_Z_LOCAL = PIN_AXIS_Z_LOCAL                                # = -90


# --- Perimeter ring dimensions -----------------------------------------

RING_OUTER = PLATFORM_ACRYLIC_SIZE              # = 180 (matches acrylic)
RING_INNER = 172.0                              # ≥ Ø172 transparent zone
RING_HEIGHT = 5.0                               # above acrylic top face
_RING_FRAME_T = (RING_OUTER - RING_INNER) / 2   # = 4

# Ring bottom face Y in cartridge-local: BOND_GAP above acrylic top (y = 0).
RING_BOTTOM_Y_LOCAL = BOND_GAP


# --- Kicker tab dimensions (slice f4) ----------------------------------

# Tab cartridge X offset. The slice (f2) lever cantilevers from the +Z
# side of its bench mount, putting the lifter pad at turret Z ∈ [+16,
# +28] = LEVER_Z_INSIDE / LEVER_Z_OUTSIDE. At DUMP the cartridge → turret
# rotation Ry(-90) maps cartridge +X → turret +Z, so the lifter lives at
# cartridge X ∈ [+16, +28]. Tab centerline at +22 puts the tab inside
# the lifter footprint with 4 mm margin on each X side after sizing.
TAB_X_CENTER = 22.0
TAB_X_LEN = 8.0                # narrower than 12 mm lifter pad in X
TAB_Z_LEN = 20.0               # narrower than 30 mm lifter pad in Z

# Tab Y span: top BOND_GAP below acrylic underside; bottom
# TAB_LIFTER_CLEARANCE_Y above the slice (f2) lifter pad's REST top.
# The lifter rises by the cam stroke and contacts the tab to tilt the
# platform; at REST the tab clears the lifter by this gap.
TAB_LIFTER_CLEARANCE_Y = 2.0
TAB_BOTTOM_Y_LOCAL = TURRET_DUMP_KICKER_LIFTER_TOP_Y + TAB_LIFTER_CLEARANCE_Y
TAB_TOP_Y_LOCAL = -PLATFORM_ACRYLIC_T - BOND_GAP
TAB_HEIGHT = TAB_TOP_Y_LOCAL - TAB_BOTTOM_Y_LOCAL


# --- Sanity asserts ----------------------------------------------------

# Pin bore wall to bracket Y (top + bottom) faces.
assert _PIN_Y_BRACKET_LOCAL - PIN_BORE_D / 2 >= WALL_MIN, (
    f"pin-bore to bracket bottom-Y wall "
    f"{_PIN_Y_BRACKET_LOCAL - PIN_BORE_D / 2:.2f} mm < WALL_MIN ({WALL_MIN}); "
    f"raise BRACKET_BLOCK_Y or move TURRET_HINGE_PIN_Y down"
)
assert (BRACKET_BLOCK_Y - _PIN_Y_BRACKET_LOCAL) - PIN_BORE_D / 2 >= WALL_MIN, (
    f"pin-bore to bracket top-Y wall "
    f"{(BRACKET_BLOCK_Y - _PIN_Y_BRACKET_LOCAL) - PIN_BORE_D / 2:.2f} mm < "
    f"WALL_MIN ({WALL_MIN})"
)

# Pin bore wall to bracket Z (front + back) faces. Bore centred on
# bracket Z centerline so both walls equal.
_pin_z_wall = BRACKET_BLOCK_Z / 2 - PIN_BORE_D / 2
assert _pin_z_wall >= WALL_MIN, (
    f"pin-bore to bracket Z wall {_pin_z_wall:.2f} mm < WALL_MIN "
    f"({WALL_MIN}); raise BRACKET_BLOCK_Z"
)

# Bracket inboard face (toward bushing block) clears the bushing block.
_inboard_face_x = abs(BRACKET_POS_X_LOCAL) - BRACKET_BLOCK_X / 2
assert _inboard_face_x - _BUSHING_HALF_X >= _PIN_AIR_GAP - 1e-9, (
    f"bracket inboard X face {_inboard_face_x:.2f} mm closer than "
    f"AIR_GAP ({_PIN_AIR_GAP}) to bushing block face "
    f"({_BUSHING_HALF_X}); shift the bracket outboard"
)

# Bracket outboard face inside acrylic X footprint.
assert (
    abs(BRACKET_POS_X_LOCAL) + BRACKET_BLOCK_X / 2
    <= PLATFORM_ACRYLIC_SIZE / 2
), "bracket outboard X face overhangs acrylic edge"

# Bracket inboard Z face (the face facing the acrylic interior, +Z
# direction in cartridge-local) sits inside the acrylic footprint so
# a real bond zone exists.
_bracket_inboard_z = BRACKET_Z_LOCAL + BRACKET_BLOCK_Z / 2
_acrylic_inner_edge_z = -PLATFORM_ACRYLIC_SIZE / 2                # = -90
_bond_zone_z_overlap = _bracket_inboard_z - _acrylic_inner_edge_z
assert _bond_zone_z_overlap > 0, (
    f"bracket bond-zone overlap with acrylic in Z is "
    f"{_bond_zone_z_overlap:.2f} mm ≤ 0; bracket sits entirely "
    f"past the acrylic edge — no bond surface"
)

# Ring frame width ≥ WALL_MIN.
assert _RING_FRAME_T >= WALL_MIN, (
    f"ring frame width {_RING_FRAME_T:.2f} mm < WALL_MIN ({WALL_MIN}); "
    f"shrink RING_INNER or grow RING_OUTER"
)

# Ring inner opening ≥ Ø172 transparent zone.
assert RING_INNER >= 172.0, (
    f"ring inner opening {RING_INNER} < 172 mm; would clip the "
    f"below-mirror sightline cone at the platform plane"
)

# --- Kicker tab asserts ------------------------------------------------

# Tab walls ≥ WALL_MIN (the box is 8 × ~38.7 × 20).
assert TAB_X_LEN >= WALL_MIN, (
    f"tab X len {TAB_X_LEN} < WALL_MIN ({WALL_MIN})"
)
assert TAB_Z_LEN >= WALL_MIN, (
    f"tab Z len {TAB_Z_LEN} < WALL_MIN ({WALL_MIN})"
)

# Tab footprint inside acrylic X / Z extents.
assert TAB_X_CENTER + TAB_X_LEN / 2 <= PLATFORM_ACRYLIC_SIZE / 2, (
    f"tab outer X face {TAB_X_CENTER + TAB_X_LEN / 2} overhangs "
    f"acrylic edge {PLATFORM_ACRYLIC_SIZE / 2}"
)
assert TAB_Z_LEN / 2 <= PLATFORM_ACRYLIC_SIZE / 2, (
    f"tab Z half-extent {TAB_Z_LEN / 2} overhangs acrylic edge "
    f"{PLATFORM_ACRYLIC_SIZE / 2}"
)

# Tab outboard of hinge brackets in cartridge X. POS bracket spans X ∈
# [BRACKET_POS_X - block/2, BRACKET_POS_X + block/2]; tab inboard X
# face must sit AIR_GAP outboard of the bracket outer face.
_TAB_INBOARD_X_FACE = TAB_X_CENTER - TAB_X_LEN / 2
_BRACKET_POS_OUTER_X = BRACKET_POS_X_LOCAL + BRACKET_BLOCK_X / 2
_TAB_BRACKET_X_GAP = _TAB_INBOARD_X_FACE - _BRACKET_POS_OUTER_X
assert _TAB_BRACKET_X_GAP >= _PIN_AIR_GAP - 1e-9, (
    f"tab inboard X face {_TAB_INBOARD_X_FACE} too close to bracket "
    f"outer X face {_BRACKET_POS_OUTER_X} (gap "
    f"{_TAB_BRACKET_X_GAP} < AIR_GAP {_PIN_AIR_GAP})"
)

# Tab Z span sits outboard of the bracket Z extent (brackets centered
# at z = -90, span z ∈ [-98, -82]). Tab Z ∈ [-Z_LEN/2, +Z_LEN/2].
_BRACKET_OUTER_Z = BRACKET_Z_LOCAL + BRACKET_BLOCK_Z / 2
assert -TAB_Z_LEN / 2 > _BRACKET_OUTER_Z, (
    f"tab inboard Z face {-TAB_Z_LEN / 2} ≤ bracket outer Z face "
    f"{_BRACKET_OUTER_Z}; tab too close to brackets in Z"
)

# Tab has positive height.
assert TAB_HEIGHT > 0, (
    f"tab height {TAB_HEIGHT} ≤ 0; check TAB_BOTTOM_Y vs acrylic "
    f"underside"
)

# Tab bottom Y strictly above lifter pad top at REST (no rest contact).
assert TAB_BOTTOM_Y_LOCAL > TURRET_DUMP_KICKER_LIFTER_TOP_Y, (
    f"tab bottom {TAB_BOTTOM_Y_LOCAL} ≤ lifter top at REST "
    f"{TURRET_DUMP_KICKER_LIFTER_TOP_Y}; would contact at rest"
)


# --- Part functions ----------------------------------------------------

def acrylic_plate(
    size: float = PLATFORM_ACRYLIC_SIZE,
    thickness: float = PLATFORM_ACRYLIC_T,
):
    """180 × 4 × 180 mm PMMA plate centred on origin in X / Z, top
    face at y = 0."""
    return Pos(0, -thickness / 2, 0) * Box(size, thickness, size)


def bracket(
    block_x: float = BRACKET_BLOCK_X,
    block_y: float = BRACKET_BLOCK_Y,
    block_z: float = BRACKET_BLOCK_Z,
    pin_y_local: float = _PIN_Y_BRACKET_LOCAL,
    pin_bore_d: float = PIN_BORE_D,
):
    """One printed hinge bracket: rectangular block with a horizontal
    pin bore along bracket-local +X. Body symmetric across YZ plane
    (X → -X), so the same printed body fits both world instances."""
    body = Pos(0, block_y / 2, 0) * Box(block_x, block_y, block_z)

    # Horizontal pin bore (axis along ±X). Cylinder default axis +Z;
    # rotate 90° about Y so axis lies along ±X.
    body -= (
        Pos(0, pin_y_local, 0)
        * Rot(0, 90, 0)
        * Cylinder(pin_bore_d / 2, block_x * 3)
    )

    return body


def ring(
    outer: float = RING_OUTER,
    inner: float = RING_INNER,
    height: float = RING_HEIGHT,
):
    """Square perimeter frame bonded to acrylic top. Outer perimeter
    `outer`, inner opening `inner`, vertical height `height`. Bottom
    face at the placement Y; centred on origin in X / Z."""
    outer_block = Pos(0, height / 2, 0) * Box(outer, height, outer)
    inner_cut = Pos(0, height / 2, 0) * Box(inner, height * 3, inner)
    return outer_block - inner_cut


def kicker_tab(
    x_len: float = TAB_X_LEN,
    z_len: float = TAB_Z_LEN,
    height: float = TAB_HEIGHT,
):
    """Printed kicker tab bonded to the acrylic underside. Solid box
    centred at part-local origin in X / Z; bottom face at part-local
    Y = 0, top face at Y = `height`. Caller positions the part-local
    origin at (TAB_X_CENTER, TAB_BOTTOM_Y_LOCAL, 0) in cartridge-local."""
    return Pos(0, height / 2, 0) * Box(x_len, height, z_len)


# --- Per-station placement helper --------------------------------------

# Rotation that maps the cartridge-local IMAGE pose onto each station's
# turret-local pose. The cartridge is built in cartridge-local with the
# pin axis along ±X and "outward" along +Z. Each station has its own
# (radial-outward, tangential) directions in turret-local; the rotation
# below picks the proper-rotation root that aligns cartridge axes onto
# them. (Reflections aren't valid placements; sign choices below pick
# the rotation whose determinant is +1.)
#
#   IMAGE  : Ry(  0°)  cartridge +X → turret +X, +Z → +Z
#   LOAD   : Ry(+90°)  cartridge +X → turret -Z, +Z → +X
#   DUMP   : Ry(-90°)  cartridge +X → turret +Z, +Z → -X
#   RETURN : Ry(180°)  cartridge +X → turret -X, +Z → -Z
STATION_ANGLES = {
    "image":  0,
    "load":   90,
    "return": 180,
    "dump":  -90,
}

# Station centroid in turret-local. The cartridge centroid (acrylic
# top-face centre) lands here for each station after rotation.
_STATION_CENTROIDS = {
    "image":  Pos(0, 0, +TURRET_ARM_R),
    "load":   Pos(+TURRET_ARM_R, 0, 0),
    "return": Pos(0, 0, -TURRET_ARM_R),
    "dump":   Pos(-TURRET_ARM_R, 0, 0),
}

# Cartridge-local part placements. Used by both the standalone
# (cartridge in isolation) and the per-station builder below
# (translated + rotated to its turret-local pose).
_IMAGE_LOCS = (
    ("acrylic", Pos(0, 0, 0)),
    ("bracket_pos", Pos(BRACKET_POS_X_LOCAL, BRACKET_BOTTOM_Y_LOCAL, BRACKET_Z_LOCAL)),
    ("bracket_neg", Pos(BRACKET_NEG_X_LOCAL, BRACKET_BOTTOM_Y_LOCAL, BRACKET_Z_LOCAL)),
    ("ring", Pos(0, RING_BOTTOM_Y_LOCAL, 0)),
    ("kicker_tab", Pos(TAB_X_CENTER, TAB_BOTTOM_Y_LOCAL, 0)),
)

_PART_BUILDERS = {
    "acrylic": acrylic_plate,
    "bracket_pos": bracket,
    "bracket_neg": bracket,
    "ring": ring,
    "kicker_tab": kicker_tab,
}


def add_turret_platform(
    a: Assembly, station: str, base=Pos(0, 0, 0)
) -> Assembly:
    """Place one cartridge at the named station ('image', 'load',
    'dump', 'return'). Cartridge geometry is built in cartridge-local
    IMAGE pose; this helper applies a per-station rotation around the
    hub axis (turret-local origin) and translates to the station's
    centroid. `base` (default: identity) is pre-multiplied to lift the
    result into a parent frame. Part names are prefixed with the
    station to avoid collisions across instances
    ("platform_load_acrylic", etc.)."""
    angle = STATION_ANGLES[station]
    centroid = _STATION_CENTROIDS[station]
    rotation = Rot(0, angle, 0)
    for sub_name, image_loc in _IMAGE_LOCS:
        full_name = f"platform_{station}_{sub_name}"
        a = a.add(
            full_name,
            _PART_BUILDERS[sub_name](),
            location=base * centroid * rotation * image_loc,
        )
    return a


# --- Standalone assembly (cartridge-local, no station placement) ------

# Build the cartridge in cartridge-local with no station rotation /
# translation. Pairwise no-interference checks are invariant under
# rigid motion; this verifies the cartridge geometry alone.
assembly = Assembly()
_STANDALONE_NAMES = tuple(f"cartridge_{sub_name}" for sub_name, _ in _IMAGE_LOCS)
for (sub_name, image_loc), full_name in zip(_IMAGE_LOCS, _STANDALONE_NAMES):
    assembly = assembly.add(full_name, _PART_BUILDERS[sub_name](), location=image_loc)

for _i, _a in enumerate(_STANDALONE_NAMES):
    for _b in _STANDALONE_NAMES[_i + 1:]:
        assembly = assembly.assert_no_interference(_a, _b)


if __name__ == "__main__":
    check(assembly, out="outputs")
    inspect(
        bracket(),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=90.0
        ),
        out="outputs",
        name="bracket",
    )
    inspect(
        ring(),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=45.0
        ),
        out="outputs",
        name="ring",
    )
    inspect(
        kicker_tab(),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=45.0
        ),
        out="outputs",
        name="kicker_tab",
    )

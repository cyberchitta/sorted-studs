"""
M03 scanner — rev-3 turret platform cartridge (slice d of the 4-station
turret).

Single platform "cartridge": PMMA acrylic plate + 2 bonded hinge
brackets near the inner (hub-side) edge + 1 bonded perimeter ring on
the top face. Four identical instances ride the slice (c) spider in
operation; this slice ships ONE cartridge in its IMAGE-station pose
(world centroid at origin, top face at y = 0). Slice (f) integrate
will replicate it at the other three stations via 90° rotations
around the hub axis.

Members (4 total — 1 acrylic + 3 printed bonded pieces):
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

Coordinate frame = world frame at IMAGE (matches `../params.py`):
    origin = brick centroid on platform top face
    +X     = operator's right (= along IMAGE pin axis)
    +Y     = up
    +Z     = away from operator (= radial outward from hub at IMAGE)

Bond joint: each printed piece sits BOND_GAP (0.3 mm) away from the
acrylic face it bonds to — same idiom as platform_acrylic. The gap is
filled by structural-acrylic epoxy in reality and keeps the modelled
cross-boundary interference check at clean zero.

Bracket-local frame (one bracket; world placement applies translation
+ X-mirror for the neg_x instance):
    origin = bottom-face centre of the bracket block
    +X     = along pin axis (world ±X at IMAGE)
    +Y     = up toward acrylic underside
    +Z     = perpendicular to pin in horizontal plane (world ±Z at
             IMAGE)

Cantilever note: each bracket extends in Z slightly past the acrylic
inner edge (acrylic spans z ∈ [-90, +90]; bracket centerline at
z = -90 with z-extent 16 mm → bracket spans z ∈ [-98, -82]). The
half from z = -98 to z = -90 sits in open air forward of the acrylic
edge — the unbonded cantilever zone where the pin bore lives. The
half from z = -90 to z = -82 sits under the acrylic in the bond zone
(8 × 5 = 40 mm² per bracket; small but two brackets share the load
and the bond is in shear, not peel).

Print orientation per piece:
  * brackets — bottom face on build plate (+Y up). Horizontal pin
    bore (Ø5.4) bridges fine in FDM; threshold raised to 90° (same
    rationale as platform_hinge_bearing).
  * ring — bottom face on build plate (+Y up). Square frame, no
    bores or overhangs; default 45° threshold suffices.
"""

import sys
from pathlib import Path

from build123d import Box, Cylinder, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
    PLATFORM_ACRYLIC_SIZE,
    PLATFORM_ACRYLIC_T,
    TURRET_HINGE_PIN_D,
    TURRET_HINGE_PIN_RADIAL,
    TURRET_HINGE_PIN_Y,
    TURRET_HUB_X,
    TURRET_HUB_Z,
    WALL_MIN,
)


# --- IMAGE-pose world references ---------------------------------------

# Slice (c) IMAGE pin axis: along world ±X at (y, z) below.
PIN_AXIS_Y_WORLD = TURRET_HINGE_PIN_Y                          # = -22.5
PIN_AXIS_Z_WORLD = TURRET_HUB_Z + TURRET_HINGE_PIN_RADIAL      # = -90

# Modelled gap between any printed piece and the acrylic face it
# bonds to. Filled by structural-acrylic epoxy in real assembly.
BOND_GAP = 0.3


# --- Hinge bracket dimensions ------------------------------------------

# Bracket-local axes:
#   +X = along pin axis        (= world ±X at IMAGE)
#   +Y = up                    (= world +Y)
#   +Z = perpendicular to pin  (= world ±Z at IMAGE)
BRACKET_BLOCK_X = 5.0          # spans one pin-tip protrusion zone
BRACKET_BLOCK_Y = 23.0         # bottom up to bond-side top face
BRACKET_BLOCK_Z = 16.0         # spans across the pin axis

# Pin bore diameter — Ø5.4 = 0.4 mm diametric clearance over the M5
# stock pin from slice (c).
PIN_BORE_D = TURRET_HINGE_PIN_D + 0.4

# Bracket bottom Y in world. Acrylic underside at y = -PLATFORM_ACRYLIC_T,
# minus BOND_GAP for the bond layer, minus block height to land the
# bottom face.
BRACKET_BOTTOM_Y_WORLD = -PLATFORM_ACRYLIC_T - BOND_GAP - BRACKET_BLOCK_Y

# Pin axis Y in bracket-local. World pin Y minus bracket bottom Y.
_PIN_Y_LOCAL = PIN_AXIS_Y_WORLD - BRACKET_BOTTOM_Y_WORLD

# Slice (c) bushing block half-width along the pin axis (mirrors
# slice (c)'s BUSHING_BLOCK_W = 20).
_BUSHING_HALF_X = 10.0
_PIN_AIR_GAP = 0.5             # bracket-to-bushing-block side clearance

# Each bracket centred at world X = ±(bushing half + air gap +
# bracket half) — its inboard face sits AIR_GAP outboard of the
# bushing block, and the bracket spans the rest of the pin tip
# (pin tip at world X = ±15 = bushing 10 + protrusion 5).
BRACKET_X_CENTRE_OFFSET = (
    _BUSHING_HALF_X + _PIN_AIR_GAP + BRACKET_BLOCK_X / 2          # = 13
)
BRACKET_POS_X_WORLD = +BRACKET_X_CENTRE_OFFSET
BRACKET_NEG_X_WORLD = -BRACKET_X_CENTRE_OFFSET

# Bracket centerline Z in world = pin axis Z (so the horizontal bore
# along ±X passes through the bracket centerline).
BRACKET_Z_WORLD = PIN_AXIS_Z_WORLD                                # = -90


# --- Perimeter ring dimensions -----------------------------------------

RING_OUTER = PLATFORM_ACRYLIC_SIZE              # = 180 (matches acrylic)
RING_INNER = 172.0                              # ≥ Ø172 transparent zone
RING_HEIGHT = 5.0                               # above acrylic top face
_RING_FRAME_T = (RING_OUTER - RING_INNER) / 2   # = 4

# Ring bottom face Y in world: BOND_GAP above acrylic top (y = 0).
RING_BOTTOM_Y_WORLD = BOND_GAP


# --- Sanity asserts ----------------------------------------------------

# Pin bore wall to bracket Y (top + bottom) faces.
assert _PIN_Y_LOCAL - PIN_BORE_D / 2 >= WALL_MIN, (
    f"pin-bore to bracket bottom-Y wall "
    f"{_PIN_Y_LOCAL - PIN_BORE_D / 2:.2f} mm < WALL_MIN ({WALL_MIN}); "
    f"raise BRACKET_BLOCK_Y or move TURRET_HINGE_PIN_Y down"
)
assert (BRACKET_BLOCK_Y - _PIN_Y_LOCAL) - PIN_BORE_D / 2 >= WALL_MIN, (
    f"pin-bore to bracket top-Y wall "
    f"{(BRACKET_BLOCK_Y - _PIN_Y_LOCAL) - PIN_BORE_D / 2:.2f} mm < "
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
_inboard_face_x = abs(BRACKET_POS_X_WORLD) - BRACKET_BLOCK_X / 2
assert _inboard_face_x - _BUSHING_HALF_X >= _PIN_AIR_GAP - 1e-9, (
    f"bracket inboard X face {_inboard_face_x:.2f} mm closer than "
    f"AIR_GAP ({_PIN_AIR_GAP}) to bushing block face "
    f"({_BUSHING_HALF_X}); shift the bracket outboard"
)

# Bracket outboard face inside acrylic X footprint.
assert (
    abs(BRACKET_POS_X_WORLD) + BRACKET_BLOCK_X / 2
    <= PLATFORM_ACRYLIC_SIZE / 2
), "bracket outboard X face overhangs acrylic edge"

# Bracket inboard Z face (the face facing the acrylic interior, +Z
# direction at IMAGE) sits inside the acrylic footprint so a real
# bond zone exists.
_bracket_inboard_z = BRACKET_Z_WORLD + BRACKET_BLOCK_Z / 2
_acrylic_inner_edge_z = -PLATFORM_ACRYLIC_SIZE / 2                # = -90
_bond_zone_z_overlap = _bracket_inboard_z - _acrylic_inner_edge_z
assert _bond_zone_z_overlap > 0, (
    f"bracket bond-zone overlap with acrylic in Z is "
    f"{_bond_zone_z_overlap:.2f} mm ≤ 0; bracket sits entirely "
    f"forward of the acrylic — no bond surface"
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
    pin_y_local: float = _PIN_Y_LOCAL,
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


# --- Per-station placement helper --------------------------------------

# Rotation around the hub axis that maps the IMAGE-pose cartridge to
# each station's world pose. Angle = world rotation around +Y axis,
# applied AFTER first translating the cartridge to put the hub axis
# at world origin (sandwich rotation; see add_turret_platform below).
#
#   IMAGE  →  IMAGE   :   0°   (identity)
#   IMAGE  →  LOAD    : +90°   (Ry(+90°) maps platform-local +X (=world +Z
#                              at IMAGE) to world +X — the LOAD radial
#                              direction)
#   IMAGE  →  RETURN  : 180°   (+Z → -Z)
#   IMAGE  →  DUMP    : -90°   (+Z → -X)
STATION_ANGLES = {
    "image":  0,
    "load":   90,
    "return": 180,
    "dump":  -90,
}

# IMAGE-pose part placements (world coords when at IMAGE). Used by
# both the standalone (no rotation) and the per-station builder
# below (rotated around the hub axis).
_IMAGE_LOCS = (
    ("acrylic", Pos(0, 0, 0)),
    ("bracket_pos", Pos(BRACKET_POS_X_WORLD, BRACKET_BOTTOM_Y_WORLD, BRACKET_Z_WORLD)),
    ("bracket_neg", Pos(BRACKET_NEG_X_WORLD, BRACKET_BOTTOM_Y_WORLD, BRACKET_Z_WORLD)),
    ("ring", Pos(0, RING_BOTTOM_Y_WORLD, 0)),
)

_PART_BUILDERS = {
    "acrylic": acrylic_plate,
    "bracket_pos": bracket,
    "bracket_neg": bracket,
    "ring": ring,
}


def add_turret_platform(
    a: Assembly,
    station: str,
    hub_x: float = TURRET_HUB_X,
    hub_z: float = TURRET_HUB_Z,
) -> Assembly:
    """Place one cartridge at the named station ('image', 'load',
    'dump', 'return') in world coordinates. The cartridge geometry
    is built in IMAGE pose; this helper applies a sandwich rotation
    around the hub axis to land it at the station's pose. Part
    names are prefixed with the station to avoid collisions across
    instances ("platform_load_acrylic", etc.)."""
    angle = STATION_ANGLES[station]
    hub_xform = (
        Pos(hub_x, 0, hub_z)
        * Rot(0, angle, 0)
        * Pos(-hub_x, 0, -hub_z)
    )
    for sub_name, image_loc in _IMAGE_LOCS:
        full_name = f"platform_{station}_{sub_name}"
        a = a.add(
            full_name,
            _PART_BUILDERS[sub_name](),
            location=hub_xform * image_loc,
        )
    return a


# --- Standalone assembly (IMAGE pose only) ----------------------------

assembly = add_turret_platform(Assembly(), "image")

_STANDALONE_NAMES = tuple(
    f"platform_image_{sub_name}" for sub_name, _ in _IMAGE_LOCS
)
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

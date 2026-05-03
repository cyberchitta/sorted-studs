"""
M05 hex-tower — rotating square-column trap + fixed ramp (generic floor).

Column-side sub-assembly of the rotating-column delivery design for
Approach F. The column is a square tube; a circular **rotating collar**
at z_local = 220..225 of *each* column rides inside the **bearing of
the floor above** (a short ring at that floor's z_local = 0..5, held
coaxial with the column by a 6-saddle spider that clamps onto radial
2020 aluminum extrusions). Columns couple end-to-end at z_world =
N·220 + 273 via face flanges integrated into each column's top.

Delivery is split into two decoupled jobs:
- **Trap** — a flat plate hinged at the +X interior wall. Closed =
  horizontal, spanning the column cross-section, catching bricks from
  above. Open = free end (-X) lifted up, brick slides toward the hinge
  (+X) and drops off. Gravity-biased to closed; actuator lifts to open.
- **Ramp** — a fixed sloped plate inside the column at the +X wall,
  just below the trap hinge. Catches the brick as it drops off the
  open trap and delivers it through the exit opening to the target
  basket (basket geometry lives in the basket sub-assembly).

Decoupling trap motion from ramp trajectory: the ramp angle is a
free design parameter sized for the basket, not coupled to the
trap's catch-state geometry.

Standalone validation here covers the column-side parts only. Basket
placement + ramp-vs-basket clearance are checked in the top-level
`../assembly.py`.

The model is parameterized on **floor role** ∈ {"base", "middle",
"top"}. Under the **top-bearing architecture** the bearing+spider
geometry is uniform across all roles (a short ring at z_local = 0..5
gripping the lower column's body where its rotating collar sits at
z_lower_local = 220..225). Roles differ only in the column itself:

- **base** — column body extends 0..273 in its own local frame, with
  a bottom collar at 0..5 (motor + ring-gear interface; off-axis pinion
  drives this collar via the base-floor bearing slot at z_world = 0).
  Body, rotating collar at 220..225, top flange at 268..273.
- **middle**, **top** — column body extends 53..273; bottom face at
  z = 53 mates with the lower column's top flange (z_lower_local =
  268..273) at z_world = N·220 + 273. No bottom collar. Top floor
  additionally has its rotating collar engaged from above by the
  ceiling bearing in the hopper sub-assembly (not modeled here).

The bearing slot on **this** floor sits at z_local = 0..5 and engages
the *lower* column's rotating collar — represented in the standalone
middle/top assembly by `lower_column` placed at z = -FLOOR_H. Without
that stand-in there is nothing here for this floor's bearing to grip.

Motor pinion + ring gear, ceiling bearing, castellated joint tabs,
and trap actuator are deferred — see the rolling NEXT_STEPS docs.

Coordinate frame:
    origin = column axis ∩ floor datum (z=0 = bearing/spider base)
    +Z     = up
    +X     = radial outward toward the exit opening
    +Y     = tangent at the opening (hinge axis direction)

Trap tilt convention (`Rot(0, θ, 0)` about the hinge):
    θ =   0° — trap horizontal, spanning column (closed / catching).
    θ = +60° — free end swung up 60° (open / releasing).

Gravity bias: free end (-X, opposite hinge) wants to fall ⇒ closed
is stable without active force.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build123d import Box, Cylinder, Location, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from params import (
    COL_INTERIOR_SIDE,
    COL_OUTER_SIDE,
    FLOOR_H,
    MAX_BRICK_DIM,
    N_BASKETS_PER_FLOOR,
    RAIL_INNER_R,
    RAIL_SIZE,
    R_BASKET_INNER,
)


# --- Derived parameters -------------------------------------------------

# Circular features on the column (rotating collar at mid-height,
# top flange at the joint plane, optional bottom collar on base).
# OD ≈ 138 — circumscribes the square section corners (96·√2 ≈ 135.8)
# with ~1 mm radial overhang so the bearing rolls on a continuous
# circular surface rather than only touching the four corners.
COLLAR_CLEARANCE = 1.0
COLLAR_OD = COL_OUTER_SIDE * math.sqrt(2) + 2 * COLLAR_CLEARANCE  # ≈ 138
COLLAR_T = 5.0

# Hinge pin + knuckle
KNUCKLE_OD = 10.0
PIN_D = 3.0
PIN_HOLE_D = 3.6
PIN_LEN = COL_OUTER_SIDE + 4.0

# Trap — flat plate, knuckle at hinge axis, plate body in -X
TRAP_T = 4.0
TRAP_W = 80.0                                 # ≤ MAX_BRICK_DIM; leaves ±Y for bosses
TRAP_HINGE_X = COL_INTERIOR_SIDE / 2 - 7.0    # 38 — inset so knuckle clears +X wall
TRAP_HINGE_Z = 185.0
# Free end clears -X wall with 2 mm gap + knuckle radius absorbed
TRAP_L = COL_INTERIOR_SIDE / 2 + TRAP_HINGE_X - 2.0 - KNUCKLE_OD / 2  # 76

TRAP_CLOSED_DEG = 0.0
TRAP_OPEN_DEG = 60.0
TRAP_SAMPLES = (15.0, 30.0, 45.0)             # intermediate sweep angles

# Hinge bosses — mounted on inside of +X wall, extend inward past hinge,
# sit outside the trap knuckle's Y extent so they clear the swinging plate.
BOSS_Y = 3.0
BOSS_Y_OFFSET = TRAP_W / 2 + BOSS_Y / 2 + 0.5           # 43
BOSS_X = COL_INTERIOR_SIDE / 2 - (TRAP_HINGE_X - KNUCKLE_OD / 2 - 1.0)  # 13
BOSS_X_CENTER = COL_INTERIOR_SIDE / 2 - BOSS_X / 2      # 38.5
BOSS_H_Z = 12.0

# Ramp — fixed sloped plate, root below hinge knuckle, extends through exit
RAMP_DEG = 20.0                               # slope below horizontal
RAMP_T = 4.0
RAMP_W = 70.0                                 # ≤ opening width minus side margin
RAMP_L = 110.0                                # length along slope direction
RAMP_DROP_BELOW_HINGE = 10.0                  # Z offset below trap hinge
RAMP_ROOT_X = COL_INTERIOR_SIDE / 2 - 2.0     # 43 — just inside +X wall
RAMP_ROOT_Z = TRAP_HINGE_Z - RAMP_DROP_BELOW_HINGE  # 175

# Ramp tip (for documentation / targeting sanity)
_RAMP_DEG_RAD = math.radians(RAMP_DEG)
RAMP_TIP_X = RAMP_ROOT_X + RAMP_L * math.cos(_RAMP_DEG_RAD)
RAMP_TIP_Z = RAMP_ROOT_Z - RAMP_L * math.sin(_RAMP_DEG_RAD)

# Exit opening in the +X column wall — must pass the ramp plus a brick
# sliding on top of it. Ramp enters the wall at x = COL_OUTER_SIDE/2.
_RAMP_WALL_OFFSET = COL_OUTER_SIDE / 2 - RAMP_ROOT_X
_RAMP_WALL_Z = RAMP_ROOT_Z - _RAMP_WALL_OFFSET * math.tan(_RAMP_DEG_RAD)
OPENING_MARGIN = 5.0
OPENING_W = RAMP_W + 2 * OPENING_MARGIN       # 80
OPENING_Z_BOT = _RAMP_WALL_Z - RAMP_T - OPENING_MARGIN
OPENING_Z_TOP = _RAMP_WALL_Z + MAX_BRICK_DIM + OPENING_MARGIN

# Sanity: bosses must fit in the solid +X wall strip outside the opening,
# and the opening must not encroach on the boss footprint.
assert BOSS_Y_OFFSET + BOSS_Y / 2 <= COL_INTERIOR_SIDE / 2, (
    f"Hinge bosses at |Y| = {BOSS_Y_OFFSET + BOSS_Y/2:.1f} mm exceed column "
    f"interior half-width {COL_INTERIOR_SIDE/2:.1f} mm. Reduce TRAP_W."
)
assert OPENING_W / 2 <= BOSS_Y_OFFSET - BOSS_Y / 2, (
    f"Opening half-width ({OPENING_W/2:.1f} mm) overlaps boss inner edge "
    f"({BOSS_Y_OFFSET - BOSS_Y/2:.1f} mm). Reduce RAMP_W or OPENING_MARGIN."
)

# Basket angular-slot width at ramp tip must admit the ramp width.
# This is an UPPER BOUND on the real slot: basket/assembly.py shrinks
# each basket's angular span below 360°/N so its radial walls clear
# the radial-rail corridor, so the real slot at the tip is narrower
# than this formula reports. The tight cross-boundary check lives in
# the top-level ../assembly.py as basket_plus_x-vs-ramp
# no-interference, which will fail if ramp geometry actually clashes
# with the (shrunk) basket walls. Sibling-imports are forbidden, so
# we can't pull BASKET_HALF_DELTA_DEG in here; keep this as a cheap
# parametric sanity check only.
BASKET_SLOT_W_AT_TIP = 2 * RAMP_TIP_X * math.tan(math.pi / N_BASKETS_PER_FLOOR)
assert RAMP_W <= BASKET_SLOT_W_AT_TIP, (
    f"RAMP_W ({RAMP_W:.1f} mm) > basket angular slot upper bound at "
    f"ramp tip ({BASKET_SLOT_W_AT_TIP:.1f} mm at X={RAMP_TIP_X:.1f}, "
    f"R_BASKET_INNER={R_BASKET_INNER:.1f}). Reduce RAMP_W, "
    f"increase R_BASKET_INNER, or decrease N_BASKETS_PER_FLOOR."
)

# Column height — pinned by the column-to-column joint plane. Joints
# stack at z_world = N·FLOOR_H + COL_Z_TOP, with each column extending
# (COL_Z_TOP - FLOOR_H) above the floor's own bracket level to form the
# overhang that the floor *above* grips. The 53 mm overhang must clear
# both the trap-open free-end Z and the +X opening top — asserted below.
RAIL_TOP_Z = 0.0                                     # frame datum (rail top face)
COL_Z_TOP = 273.0                                    # joint plane in this floor's frame

_TRAP_OPEN_RAD = math.radians(TRAP_OPEN_DEG)
_TRAP_FREE_Z_OPEN = TRAP_HINGE_Z + TRAP_L * math.sin(_TRAP_OPEN_RAD) + TRAP_T / 2
assert COL_Z_TOP >= max(OPENING_Z_TOP, _TRAP_FREE_Z_OPEN) + 5.0, (
    f"COL_Z_TOP ({COL_Z_TOP}) too low — trap-open free Z "
    f"({_TRAP_FREE_Z_OPEN:.1f}) or opening top ({OPENING_Z_TOP:.1f}) "
    f"reaches inside the joint zone."
)

# Joint flange thickness at the column top. Top 5 mm of the column
# carries a circular face flange (OD = COLLAR_OD) for end-to-end
# coupling with the column above. Castellated tab geometry deferred —
# see m05_diverter/NEXT_STEPS.md "Column-to-column joint".
JOINT_FLANGE_T = COLLAR_T                            # 5.0
JOINT_FLANGE_Z_BOT = COL_Z_TOP - JOINT_FLANGE_T      # 268
JOINT_FLANGE_Z_TOP = COL_Z_TOP                       # 273

# Rotating collar — circular OD on the column at mid-height that the
# bearing on the floor *above* grips (so this floor's bearing engages
# the *lower* column's rotating collar). Sits at z_local = 220..225,
# which falls inside `OPENING_Z_BOT.._TOP = 164..258` — so the collar
# is split by the +X opening into a ~270° partial ring. Mechanically
# fine for low RPM / low load.
ROT_COLLAR_T = COLLAR_T                              # 5.0
ROT_COLLAR_Z_BOT = FLOOR_H                           # 220
ROT_COLLAR_Z_TOP = ROT_COLLAR_Z_BOT + ROT_COLLAR_T   # 225
assert OPENING_Z_BOT <= ROT_COLLAR_Z_BOT and ROT_COLLAR_Z_TOP <= OPENING_Z_TOP, (
    f"Rotating collar at z={ROT_COLLAR_Z_BOT}..{ROT_COLLAR_Z_TOP} not "
    f"inside opening z={OPENING_Z_BOT:.1f}..{OPENING_Z_TOP:.1f}; the "
    f"partial-ring split assumption no longer holds."
)

# Bearing engagement plane on this floor — short ring at z = 0..5,
# uniform across all roles. Engages the lower column's rotating collar
# (z_lower_local 220..225 = z_local 0..5) for middle/top, or the base
# column's own bottom collar (z_base_local 0..5) for the base slot.
BEARING_T = COLLAR_T                                 # 5.0
BEARING_Z_BOT = RAIL_TOP_Z                           # 0
BEARING_Z_TOP = BEARING_Z_BOT + BEARING_T            # 5

# Lower-floor trap intrusion. Floor below's trap at max-open (60°)
# reaches up into this floor's frame to the Z below. With the bearing
# at 0..5 and this floor's column-bottom at 53, the trap envelope
# (~36 mm) sits in the open volume between them — the assertion is
# that it doesn't punch into either.
LOWER_TRAP_OPEN_Z_MAX = (
    TRAP_HINGE_Z
    + (TRAP_L + KNUCKLE_OD / 2) * math.sin(_TRAP_OPEN_RAD)
    + (TRAP_T / 2) * math.cos(_TRAP_OPEN_RAD)
    - FLOOR_H
)
LOWER_COL_TOP_Z_MAX = COL_Z_TOP - FLOOR_H            # 53 — joint plane

FLOOR_ROLES = ("base", "middle", "top")


def _col_z_bot(role: str) -> float:
    """Bottom of the rotating column for a given floor role.

    Base sits at the rail datum (z=0). Middle/top sit at the joint
    plane (z=53); they meet the lower column's top flange tangent at
    that plane. No clearance gap — the columns are designed to mate
    face-to-face under gravity preload.
    """
    if role == "base":
        return RAIL_TOP_Z
    if role in ("middle", "top"):
        return LOWER_COL_TOP_Z_MAX
    raise ValueError(f"unknown floor role: {role!r} (expected one of {FLOOR_ROLES})")

# Ring bearing
RING_BEARING_ID = COLLAR_OD + 2 * COLLAR_CLEARANCE
RING_BEARING_OD = RING_BEARING_ID + 20.0

# Bearing spider
N_SPIDER_SADDLES = 6
SPIDER_HUB_ID = RING_BEARING_OD + 1.0
SPIDER_HUB_OD = SPIDER_HUB_ID + 10.0

EXT_SIDE = RAIL_SIZE                          # 2020 aluminum extrusion (== frame)

SADDLE_T_TOP = 3.0
SADDLE_T_WALL = 3.0
SADDLE_COVER_H = 12.0
# Saddles sit at azimuth (i+0.5)·(360°/N_SPIDER_SADDLES) — along basket
# boundaries = inner-hex vertex direction = radial-rail axis. Each
# saddle spans from the hub outward past RAIL_INNER_R to grip
# SADDLE_GRIP_LEN of rail at its outer end. The cantilever section
# (hub → rail inner end) is a U-channel in cross-section because the
# extrusion cutout runs the full saddle length; upgrading that to a
# solid bridge is a structural refinement for a later pass.
SADDLE_GRIP_LEN = 20.0
SADDLE_L = RAIL_INNER_R + SADDLE_GRIP_LEN - SPIDER_HUB_OD / 2
SADDLE_HUB_OVERLAP = 2.0
SADDLE_BOLT_D = 4.5
SADDLE_BOLT_OFFSET = RAIL_INNER_R + SADDLE_GRIP_LEN / 2 - SPIDER_HUB_OD / 2

WALL_MIN = 1.5


# --- Helpers ------------------------------------------------------------

def _cyl_along_y(r: float, length: float) -> Part:
    return Rot(90, 0, 0) * Cylinder(r, length)


# --- Part functions -----------------------------------------------------

def bearing_spider(role: str = "middle") -> Part:
    """Printed hub that holds the ring bearing coaxial with the column.
    Six integrated saddles grip the tops of radial 2020 aluminum
    extrusions; the extrusions carry the span out to the inner hex.

    Geometry is **uniform across all roles** under the top-bearing
    architecture: a short annular ring at z = 0..BEARING_T (5 mm) with
    six saddles. The bearing this hub holds engages whatever column
    passes through z = 0..5 — for middle/top that's the *lower*
    column's rotating collar (mapped from z_lower_local 220..225);
    for base it's the base column's own bottom collar.
    """
    hub_z_bot = BEARING_Z_BOT
    hub_z_top = BEARING_Z_TOP
    hub_t = hub_z_top - hub_z_bot
    hub_z_center = (hub_z_bot + hub_z_top) / 2
    ext_top_z = BEARING_Z_TOP                          # saddle clamps just above rail

    hub_outer = Pos(0, 0, hub_z_center) * Cylinder(SPIDER_HUB_OD / 2, hub_t)
    hub_inner = Pos(0, 0, hub_z_center) * Cylinder(SPIDER_HUB_ID / 2, hub_t * 2)
    body = hub_outer - hub_inner

    saddle_w = EXT_SIDE + 2 * SADDLE_T_WALL
    saddle_h = SADDLE_COVER_H + SADDLE_T_TOP
    saddle_x_in = SPIDER_HUB_OD / 2 - SADDLE_HUB_OVERLAP
    saddle_x_out = SPIDER_HUB_OD / 2 + SADDLE_L
    saddle_z_bot = ext_top_z - SADDLE_COVER_H
    saddle_z_top = ext_top_z + SADDLE_T_TOP

    cutout_x_in = SPIDER_HUB_OD / 2
    cutout_x_out = saddle_x_out + 5.0
    cutout_z_top = ext_top_z
    cutout_z_bot = saddle_z_bot - 1.0

    bolt_x = SPIDER_HUB_OD / 2 + SADDLE_BOLT_OFFSET
    bolt_z_bot = ext_top_z - 1.0
    bolt_z_top = saddle_z_top + 1.0

    # Saddles at (i+0.5) * step = basket boundaries / inner-hex vertex
    # azimuths, aligned with the radial rails they grip.
    for i in range(N_SPIDER_SADDLES):
        theta = (i + 0.5) * 360.0 / N_SPIDER_SADDLES
        rot = Rot(0, 0, theta)
        saddle_env = rot * Pos(
            (saddle_x_in + saddle_x_out) / 2, 0,
            (saddle_z_bot + saddle_z_top) / 2,
        ) * Box(saddle_x_out - saddle_x_in, saddle_w, saddle_h)
        cutout = rot * Pos(
            (cutout_x_in + cutout_x_out) / 2, 0,
            (cutout_z_bot + cutout_z_top) / 2,
        ) * Box(cutout_x_out - cutout_x_in, EXT_SIDE, cutout_z_top - cutout_z_bot)
        bolt_hole = rot * Pos(bolt_x, 0, (bolt_z_bot + bolt_z_top) / 2) * Cylinder(
            SADDLE_BOLT_D / 2, bolt_z_top - bolt_z_bot,
        )
        body = body + saddle_env - cutout - bolt_hole
    return body


def ring_bearing(role: str = "middle") -> Part:
    """Stationary annular bearing at this floor's bracket level (z =
    0..BEARING_T). Uniform across roles — see `bearing_spider` for the
    per-role engagement story.
    """
    del role  # geometry is uniform; argument kept for symmetry / future role split
    z_center = (BEARING_Z_BOT + BEARING_Z_TOP) / 2
    body = Pos(0, 0, z_center) * Cylinder(RING_BEARING_OD / 2, BEARING_T)
    hole = Pos(0, 0, z_center) * Cylinder(RING_BEARING_ID / 2, BEARING_T * 2)
    return body - hole


def _circular_feature(z_bot: float, z_top: float) -> Part:
    """Annular feature on the column — circular OD = COLLAR_OD with the
    column interior (square hole) removed. Used for the rotating
    collar, the top joint flange, and the base column's bottom collar.
    """
    h = z_top - z_bot
    z_center = (z_top + z_bot) / 2
    disc = Pos(0, 0, z_center) * Cylinder(COLLAR_OD / 2, h)
    hole = Pos(0, 0, z_center) * Box(
        COL_INTERIOR_SIDE, COL_INTERIOR_SIDE, h * 2
    )
    return disc - hole


def column(role: str = "middle") -> Part:
    """Square-section tube with a +X exit opening, hinge bosses, a
    rotating collar at mid-height (the bearing-engagement surface for
    the floor *above*), a top joint flange, and — for the base role
    only — a bottom collar where the motor pinion meets the ring gear.

    Roles differ only in what sits at the column bottom:
    - **base**: body 0..273, bottom collar at 0..5 (ring-gear interface).
    - **middle**, **top**: body 53..273, mating face at z=53.
    """
    z_bot = _col_z_bot(role)
    col_h = COL_Z_TOP - z_bot
    col_z_center = (COL_Z_TOP + z_bot) / 2

    outer = Pos(0, 0, col_z_center) * Box(COL_OUTER_SIDE, COL_OUTER_SIDE, col_h)
    inner = Pos(0, 0, col_z_center) * Box(
        COL_INTERIOR_SIDE, COL_INTERIOR_SIDE, col_h * 2
    )
    tube = outer - inner

    # Rotating collar — circular OD at z 220..225 that the bearing on
    # the floor *above* grips. Added BEFORE the +X opening cut so the
    # opening also slices the collar into a partial ring.
    tube = tube + _circular_feature(ROT_COLLAR_Z_BOT, ROT_COLLAR_Z_TOP)

    # Top joint flange — castellated face flange placeholder (smooth
    # disc here; tab geometry deferred per NEXT_STEPS).
    tube = tube + _circular_feature(JOINT_FLANGE_Z_BOT, JOINT_FLANGE_Z_TOP)

    # Role-specific bottom feature.
    if role == "base":
        # Bottom collar at z 0..5 — motor pinion engages a ring gear
        # integrated with this collar's OD (gear teeth not modeled).
        tube = tube + _circular_feature(BEARING_Z_BOT, BEARING_Z_TOP)
    else:
        # Middle/top: bottom face flange mating with the lower column's
        # top flange at z = LOWER_COL_TOP_Z_MAX (= 53). Same OD; the two
        # flanges contact face-to-face at the joint plane.
        tube = tube + _circular_feature(z_bot, z_bot + JOINT_FLANGE_T)

    # Exit opening on +X wall — cuts the tube AND any circular feature
    # that overlaps its z range (rotating collar at 220..225 ⊂ 164..258).
    cut_z_center = (OPENING_Z_TOP + OPENING_Z_BOT) / 2
    cut_h = OPENING_Z_TOP - OPENING_Z_BOT
    cut = Pos(COL_OUTER_SIDE, 0, cut_z_center) * Box(
        COL_OUTER_SIDE * 2, OPENING_W, cut_h
    )
    tube = tube - cut

    # Hinge bosses — inside surface of +X wall, ±Y of the opening, at
    # TRAP_HINGE_Z. They extend -X (inward) to anchor the pin.
    boss_plus = Pos(BOSS_X_CENTER, +BOSS_Y_OFFSET, TRAP_HINGE_Z) * Box(
        BOSS_X, BOSS_Y, BOSS_H_Z
    )
    boss_minus = Pos(BOSS_X_CENTER, -BOSS_Y_OFFSET, TRAP_HINGE_Z) * Box(
        BOSS_X, BOSS_Y, BOSS_H_Z
    )
    tube = tube + boss_plus + boss_minus

    # Pin bore through bosses (and the air between)
    pin_bore = Pos(TRAP_HINGE_X, 0, TRAP_HINGE_Z) * _cyl_along_y(
        PIN_HOLE_D / 2, PIN_LEN + 4.0
    )
    return tube - pin_bore


def hinge_pin() -> Part:
    return _cyl_along_y(PIN_D / 2, PIN_LEN)


def trap() -> Part:
    """Flat plate hinged at +X interior wall. In the local frame the
    hinge knuckle is at origin and the plate body extends in -X to
    x = -TRAP_L. Closed state = horizontal (θ=0); open state lifts
    the free end (-X) upward.
    """
    plate_center_x = -(TRAP_L / 2 + KNUCKLE_OD / 2)
    plate = Pos(plate_center_x, 0, 0) * Box(TRAP_L, TRAP_W, TRAP_T)
    knuckle = _cyl_along_y(KNUCKLE_OD / 2, TRAP_W)
    body = plate + knuckle
    pin_hole = _cyl_along_y(PIN_HOLE_D / 2, TRAP_W * 2)
    return body - pin_hole


def ramp() -> Part:
    """Fixed sloped delivery plate. Root in the +X half of the column
    interior just below the trap hinge; extends in +X through the
    exit opening, sloped down at RAMP_DEG.

    Built in local frame with root at origin, plate extending in +X,
    then rotated about Y by +RAMP_DEG (tilts +X end toward -Z).
    """
    plate = Pos(RAMP_L / 2, 0, 0) * Box(RAMP_L, RAMP_W, RAMP_T)
    return Rot(0, RAMP_DEG, 0) * plate


# --- Assembly -----------------------------------------------------------

hinge_loc = Location((TRAP_HINGE_X, 0, TRAP_HINGE_Z))
ramp_loc = Location((RAMP_ROOT_X, 0, RAMP_ROOT_Z))
# Lower-floor stand-ins (middle/top roles only; base has no floor below).
# Both are mapped into this floor's frame via -FLOOR_H.
# - lower_trap_open: the floor below's trap at max-open (60°). Reaches
#   z ≈ 36 — must clear bearing/spider (z ≤ 5) and this floor's column
#   (z ≥ 53), with the open volume in between.
# - lower_column: the floor below's column body. Its top reaches
#   z = LOWER_COL_TOP_Z_MAX (= 53) and its rotating collar sits inside
#   this floor's bearing at z = 0..5 (engagement surface).
lower_trap_loc = Location(
    (TRAP_HINGE_X, 0, TRAP_HINGE_Z - FLOOR_H)
) * Rot(0, TRAP_OPEN_DEG, 0)
lower_column_loc = Location((0, 0, -FLOOR_H))


def make_assembly(role: str = "middle") -> Assembly:
    """Build the standalone single-floor assembly for the given role.

    `role` ∈ FLOOR_ROLES. Middle and top are geometrically identical
    here (both experience the same upward-intruding lower-floor
    column + trap-open envelope); base omits the lower-floor stand-ins
    and includes its bottom collar instead.
    """
    a = Assembly()
    a = (
        a
        .add("bearing_spider", bearing_spider(role))
        .add("ring_bearing", ring_bearing(role))
        .add("column", column(role))
        .add("hinge_pin", hinge_pin(), location=hinge_loc)
        .add("ramp", ramp(), location=ramp_loc)
        .add("trap_closed", trap(), location=hinge_loc * Rot(0, TRAP_CLOSED_DEG, 0))
        .add("trap_open", trap(), location=hinge_loc * Rot(0, TRAP_OPEN_DEG, 0))
    )
    if role != "base":
        # Lower floor's column placed at -FLOOR_H. Modeled as `column("middle")`
        # for both middle (lower=base|middle) and top (lower=middle); the
        # difference between base and middle lower columns lives at
        # z_lower_local = 0..5 (the bottom collar), which maps to
        # z_local = -220..-215 — far below anything modeled in this
        # frame. Using "middle" as the generic stand-in keeps this
        # assembly role-independent of which floor the lower one is.
        a = (
            a
            .add("lower_column", column("middle"), location=lower_column_loc)
            .add("lower_trap_open", trap(), location=lower_trap_loc)
        )
    for angle in TRAP_SAMPLES:
        a = a.add(
            f"trap_at_{int(angle)}", trap(),
            location=hinge_loc * Rot(0, angle, 0),
        )

    a = (
        a
        # Fixed parts don't interfere with each other
        .assert_no_interference("bearing_spider", "ring_bearing")
        .assert_no_interference("bearing_spider", "column")
        .assert_no_interference("ring_bearing", "column")
        .assert_no_interference("hinge_pin", "column")
        .assert_no_interference("ramp", "column")
        .assert_no_interference("ramp", "bearing_spider")
        .assert_no_interference("ramp", "ring_bearing")
        .assert_no_interference("ramp", "hinge_pin")
        # Trap closed — horizontal plate inside column, above ramp
        .assert_no_interference("trap_closed", "column")
        .assert_no_interference("trap_closed", "bearing_spider")
        .assert_no_interference("trap_closed", "ring_bearing")
        .assert_no_interference("trap_closed", "ramp")
        # Trap open — free end up, still inside column
        .assert_no_interference("trap_open", "column")
        .assert_no_interference("trap_open", "bearing_spider")
        .assert_no_interference("trap_open", "ring_bearing")
        .assert_no_interference("trap_open", "ramp")
    )
    if role != "base":
        a = (
            a
            # Bearing/spider engage the LOWER column's rotating collar
            # via grease film (no interference). Spider hub and lower-
            # column body must also clear (spider sits radially outside
            # the bearing, far from the column).
            .assert_no_interference("bearing_spider", "lower_column")
            .assert_no_interference("ring_bearing", "lower_column")
            # This floor's column tangent-meets the lower column at
            # z=53 (joint plane). assert_no_interference allows
            # face-to-face contact.
            .assert_no_interference("column", "lower_column")
            # Ramp / trap / hinge live well above the joint plane,
            # so they're clear of the lower column entirely.
            .assert_no_interference("ramp", "lower_column")
            .assert_no_interference("hinge_pin", "lower_column")
            .assert_no_interference("trap_closed", "lower_column")
            .assert_no_interference("trap_open", "lower_column")
            # Lower-floor trap-open intrusion — must clear THIS floor's
            # column-bottom stack (bearing / spider / column / ramp).
            .assert_no_interference("lower_trap_open", "column")
            .assert_no_interference("lower_trap_open", "bearing_spider")
            .assert_no_interference("lower_trap_open", "ring_bearing")
            .assert_no_interference("lower_trap_open", "ramp")
        )
    for angle in TRAP_SAMPLES:
        name = f"trap_at_{int(angle)}"
        a = (
            a
            .assert_no_interference(name, "column")
            .assert_no_interference(name, "bearing_spider")
            .assert_no_interference(name, "ring_bearing")
            .assert_no_interference(name, "ramp")
        )
    return a


# Module-level "default" assembly — preserved so the top-level
# `m05_diverter/assembly.py` and other consumers can import a ready-
# built middle-floor assembly without going through `make_assembly`.
assembly = make_assembly("middle")


if __name__ == "__main__":
    for role in FLOOR_ROLES:
        check(make_assembly(role), out=f"outputs/{role}")
    method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0)
    # Per-part waivers for known printability false positives. A new
    # failure on any part NOT listed here will raise. Full per-part
    # diagnostics still land in outputs/<role>/<name>-printability.json.
    waivers = {
        "bearing_spider": "ray-sampling artifact at hub annulus + saddle bolt-hole edges",
        "ring_bearing": "ray-sampling artifact at thin annulus edges",
    }
    for role in FLOOR_ROLES:
        for name, part in [
            ("bearing_spider", bearing_spider(role)),
            ("ring_bearing", ring_bearing(role)),
            ("column", column(role)),
        ]:
            try:
                inspect(part, method=method, out=f"outputs/{role}", name=name)
            except SystemExit:
                if name not in waivers:
                    raise
                print(
                    f"WAIVED: {role}/{name} printability ({waivers[name]})",
                    file=sys.stderr,
                )
    # Trap and ramp are role-independent — inspect once at the top.
    for name, part_fn in [("trap", trap), ("ramp", ramp)]:
        try:
            inspect(part_fn(), method=method, out="outputs", name=name)
        except SystemExit:
            if name not in waivers:
                raise
            print(f"WAIVED: {name} printability ({waivers[name]})", file=sys.stderr)

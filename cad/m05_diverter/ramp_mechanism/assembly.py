"""
M05 hex-tower — rotating square-column trap + fixed ramp (generic floor).

Column-side sub-assembly of the rotating-column delivery design for
Approach F. The column is a square tube with a circular collar at the
bottom (bearing interface); the collar rides in a fixed ring bearing
held coaxial with the column by a **spider bracket** — a short printed
hub with six integrated saddles that clamp over the top of radial 2020
aluminum extrusions running out to the inner-hex frame.

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

This is an **intermediate-floor** model — the base floor additionally
has a shaft + motor mount + bearing boss to drive rotation.

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
    MAX_BRICK_DIM,
    N_BASKETS_PER_FLOOR,
    R_BASKET_INNER,
)


# --- Derived parameters -------------------------------------------------

# Bottom collar (bearing interface)
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
TRAP_HINGE_Z = 135.0
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
RAMP_ROOT_Z = TRAP_HINGE_Z - RAMP_DROP_BELOW_HINGE  # 90

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
# Shared params (N_BASKETS_PER_FLOOR, R_BASKET_INNER) ensure this
# assertion fires on the ramp side before the basket sub-assembly loads.
BASKET_SLOT_W_AT_TIP = 2 * RAMP_TIP_X * math.tan(math.pi / N_BASKETS_PER_FLOOR)
assert RAMP_W <= BASKET_SLOT_W_AT_TIP, (
    f"RAMP_W ({RAMP_W:.1f} mm) > basket angular slot at ramp tip "
    f"({BASKET_SLOT_W_AT_TIP:.1f} mm at X={RAMP_TIP_X:.1f}, "
    f"R_BASKET_INNER={R_BASKET_INNER:.1f}). Reduce RAMP_W, "
    f"increase R_BASKET_INNER, or decrease N_BASKETS_PER_FLOOR."
)

# Column height — above trap open position, above opening top
_TRAP_OPEN_RAD = math.radians(TRAP_OPEN_DEG)
_TRAP_FREE_Z_OPEN = TRAP_HINGE_Z + TRAP_L * math.sin(_TRAP_OPEN_RAD) + TRAP_T / 2
COL_Z_BOT = 0.0
COL_Z_TOP = max(OPENING_Z_TOP, _TRAP_FREE_Z_OPEN) + 10.0

# Ring bearing
RING_BEARING_ID = COLLAR_OD + 2 * COLLAR_CLEARANCE
RING_BEARING_OD = RING_BEARING_ID + 20.0

# Bearing spider
N_SPIDER_SADDLES = 6
SPIDER_HUB_ID = RING_BEARING_OD + 1.0
SPIDER_HUB_OD = SPIDER_HUB_ID + 10.0
SPIDER_T = COLLAR_T

EXT_SIDE = 20.0                               # 2020 aluminum extrusion

SADDLE_T_TOP = 3.0
SADDLE_T_WALL = 3.0
SADDLE_COVER_H = 12.0
SADDLE_L = 30.0
SADDLE_HUB_OVERLAP = 2.0
SADDLE_BOLT_D = 4.5
SADDLE_BOLT_OFFSET = 15.0

WALL_MIN = 1.5


# --- Helpers ------------------------------------------------------------

def _cyl_along_y(r: float, length: float) -> Part:
    return Rot(90, 0, 0) * Cylinder(r, length)


# --- Part functions -----------------------------------------------------

def bearing_spider() -> Part:
    """Printed hub that holds the ring bearing coaxial with the column.
    Six integrated saddles grip the tops of radial 2020 aluminum
    extrusions; the extrusions carry the span out to the inner hex.
    """
    hub_z_center = COL_Z_BOT + SPIDER_T / 2
    ext_top_z = COL_Z_BOT + COLLAR_T

    hub_outer = Pos(0, 0, hub_z_center) * Cylinder(SPIDER_HUB_OD / 2, SPIDER_T)
    hub_inner = Pos(0, 0, hub_z_center) * Cylinder(SPIDER_HUB_ID / 2, SPIDER_T * 2)
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

    for i in range(N_SPIDER_SADDLES):
        theta = i * 360.0 / N_SPIDER_SADDLES
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


def ring_bearing() -> Part:
    """Stationary annular bearing at floor level."""
    z_center = COL_Z_BOT + COLLAR_T / 2
    body = Pos(0, 0, z_center) * Cylinder(RING_BEARING_OD / 2, COLLAR_T)
    hole = Pos(0, 0, z_center) * Cylinder(RING_BEARING_ID / 2, COLLAR_T * 2)
    return body - hole


def column() -> Part:
    """Square-section tube with a bottom collar, an exit opening in the
    +X wall sized for the ramp + brick clearance, and hinge bosses
    on the +X interior wall at the trap hinge line.
    """
    col_h = COL_Z_TOP - COL_Z_BOT
    col_z_center = (COL_Z_TOP + COL_Z_BOT) / 2

    outer = Pos(0, 0, col_z_center) * Box(COL_OUTER_SIDE, COL_OUTER_SIDE, col_h)
    inner = Pos(0, 0, col_z_center) * Box(
        COL_INTERIOR_SIDE, COL_INTERIOR_SIDE, col_h * 2
    )
    tube = outer - inner

    # Exit opening on +X wall
    cut_z_center = (OPENING_Z_TOP + OPENING_Z_BOT) / 2
    cut_h = OPENING_Z_TOP - OPENING_Z_BOT
    cut = Pos(COL_OUTER_SIDE, 0, cut_z_center) * Box(
        COL_OUTER_SIDE * 2, OPENING_W, cut_h
    )
    tube = tube - cut

    # Bottom collar with square hole
    collar_z = COL_Z_BOT + COLLAR_T / 2
    collar_disc = Pos(0, 0, collar_z) * Cylinder(COLLAR_OD / 2, COLLAR_T)
    collar_hole = Pos(0, 0, collar_z) * Box(
        COL_INTERIOR_SIDE, COL_INTERIOR_SIDE, COLLAR_T * 2
    )
    tube = tube + (collar_disc - collar_hole)

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

assembly = Assembly()
assembly = (
    assembly
    .add("bearing_spider", bearing_spider())
    .add("ring_bearing", ring_bearing())
    .add("column", column())
    .add("hinge_pin", hinge_pin(), location=hinge_loc)
    .add("ramp", ramp(), location=ramp_loc)
    .add("trap_closed", trap(), location=hinge_loc * Rot(0, TRAP_CLOSED_DEG, 0))
    .add("trap_open", trap(), location=hinge_loc * Rot(0, TRAP_OPEN_DEG, 0))
)
for angle in TRAP_SAMPLES:
    assembly = assembly.add(
        f"trap_at_{int(angle)}", trap(),
        location=hinge_loc * Rot(0, angle, 0),
    )

assembly = (
    assembly
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
for angle in TRAP_SAMPLES:
    name = f"trap_at_{int(angle)}"
    assembly = (
        assembly
        .assert_no_interference(name, "column")
        .assert_no_interference(name, "bearing_spider")
        .assert_no_interference(name, "ring_bearing")
        .assert_no_interference(name, "ramp")
    )


if __name__ == "__main__":
    check(assembly, out="outputs")
    method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0)
    for name, part_fn in [
        ("bearing_spider", bearing_spider),
        ("ring_bearing", ring_bearing),
        ("column", column),
        ("trap", trap),
        ("ramp", ramp),
    ]:
        try:
            inspect(part_fn(), method=method, out="outputs", name=name)
        except SystemExit:
            pass

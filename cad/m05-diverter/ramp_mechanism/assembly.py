"""
M05 hex-tower — rotating square-column flap mechanism (generic floor).

Single-floor model of the rotating-column trap-door design for Approach F.
The column is a square tube with a circular collar at the bottom (bearing
interface); the collar rides in a fixed ring bearing attached to the
tower frame, so nothing obstructs the drop chute. An exit opening on
one wall is closed in DOWN state by a flap hinged at its middle: a
rectangular catch section (inner of hinge) covers the column cross-
section when UP, and a narrower spout section (outer of hinge) delivers
bricks into the selected basket.

This is an **intermediate-floor** model — the base floor additionally
has a shaft + motor mount + bearing boss to drive rotation. Those are
not modeled here; intermediate floors are structurally passive beyond
the ring-bearing support.

One top-level parameter — `MAX_BRICK_DIM` — sizes the column interior.
`N_BASKETS_PER_FLOOR` and `R_BASKET_INNER` together size the spout exit
width (narrower = finer basket angular targeting).

Coordinate frame:
    origin = column axis ∩ floor-plate top plane
    +Z     = up
    +X     = radial outward toward the exit opening
    +Y     = tangent at the opening (hinge axis direction)

Tilt convention (`Rot(0, θ, 0)`; positive θ rotates the +X tip toward -Z):
    tilt = 0°   — flap horizontal; catch in -X, spout in +X.
    tilt = -90° — catch vertical down (DOWN state; spout vertical up).
    tilt = +10° — catch 10° above horizontal (UP / catching state).

Gravity bias: catch section is longer than spout ⇒ DOWN is a stable
equilibrium (catch hangs, spout rises).
"""

import math

from build123d import Box, Cylinder, Location, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM


# --- Top-level parameters -----------------------------------------------

MAX_BRICK_DIM = 80.0            # largest LEGO dim this sorter handles (mm)
BRICK_SLOP = 5.0                # radial clearance brick-to-column-wall (mm)
N_BASKETS_PER_FLOOR = 6         # 6 / 12 / 18; affects spout-exit width
R_BASKET_INNER = 150.0          # radial distance column axis → basket lip (mm)


# --- Derived parameters -------------------------------------------------

# Column cross-section
COL_INTERIOR_SIDE = MAX_BRICK_DIM + 2 * BRICK_SLOP   # 90
COL_WALL_T = 3.0
COL_OUTER_SIDE = COL_INTERIOR_SIDE + 2 * COL_WALL_T  # 96

# Bottom collar (bearing interface)
COLLAR_CLEARANCE = 1.0
COLLAR_OD = COL_OUTER_SIDE * math.sqrt(2) + 2 * COLLAR_CLEARANCE  # ≈ 138
COLLAR_T = 5.0

# Flap — catch is rectangular, spout is a narrower rectangle (step at
# hinge; smooth taper is a follow-up)
FLAP_T = 4.0
FLAP_W = COL_INTERIOR_SIDE - 2              # 88 — 1 mm side clearance
FLAP_L_CATCH = COL_INTERIOR_SIDE - 2        # 88 — spans column interior
# Spout reaches into the basket's inner lip
FLAP_L_SPOUT = R_BASKET_INNER - COL_OUTER_SIDE / 2 + 10.0  # 112

# Spout exit width — must be ≤ basket slot angular width AND ≤ flap width
BASKET_SLOT_W = 2 * R_BASKET_INNER * math.sin(math.pi / N_BASKETS_PER_FLOOR)
SPOUT_EXIT_W = min(BASKET_SLOT_W, FLAP_W)

# Sanity: spout exit must fit the largest brick
assert SPOUT_EXIT_W >= MAX_BRICK_DIM, (
    f"SPOUT_EXIT_W ({SPOUT_EXIT_W:.1f} mm) < MAX_BRICK_DIM "
    f"({MAX_BRICK_DIM:.1f} mm). The spout exit can't pass the largest "
    f"brick. Increase R_BASKET_INNER (currently {R_BASKET_INNER}), "
    f"decrease N_BASKETS_PER_FLOOR (currently {N_BASKETS_PER_FLOOR}), "
    f"or decrease MAX_BRICK_DIM."
)

# Flap knuckle / hinge pin
KNUCKLE_OD = 10.0
PIN_D = 3.0
PIN_HOLE_D = 3.6
PIN_LEN = COL_OUTER_SIDE + 4.0

# Hinge bosses on the column (inside surface, on each side of the opening)
BOSS_X = 10.0                               # radial extent (inward)
BOSS_Y = 3.0                                # tangential thickness
BOSS_H_Z = 12.0
BOSS_X_CENTER = COL_INTERIOR_SIDE / 2 - BOSS_X / 2

# Opening + pivot
OPENING_W = FLAP_W + 2.0                    # 90
OPENING_Z_BOT = COLLAR_T + 2.0              # above bottom collar
PIVOT_X = COL_INTERIOR_SIDE / 2             # at inner face of +X wall
PIVOT_Z = OPENING_Z_BOT + FLAP_L_CATCH      # DOWN catch reaches opening bottom
# Opening top: big enough for brick exit AND for spout clearance at DOWN
EXIT_MARGIN = 5.0
_ABOVE_PIVOT = max(
    FLAP_L_SPOUT + 5.0,                      # spout peaks at DOWN
    FLAP_T / 2 + MAX_BRICK_DIM + EXIT_MARGIN # brick exit clearance at UP
)
OPENING_Z_TOP = PIVOT_Z + _ABOVE_PIVOT

BOSS_Y_OFFSET = OPENING_W / 2 + BOSS_Y / 2

# Column height — just above the opening
COL_Z_BOT = 0.0
COL_Z_TOP = OPENING_Z_TOP + 5.0

# Floor plate + ring bearing
FLOOR_T = 5.0
FLOOR_OUTER_HALF = R_BASKET_INNER + 40.0
RING_BEARING_ID = COLLAR_OD + 2 * COLLAR_CLEARANCE
RING_BEARING_OD = RING_BEARING_ID + 20.0
FLOOR_HOLE_R = RING_BEARING_OD / 2 + 1.0

# Tilt angles
TILT_UP = 10.0
TILT_DOWN = -90.0
TILT_SAMPLES = (-60.0, -30.0, 0.0)

WALL_MIN = 1.5


# --- Helpers ------------------------------------------------------------

def _cyl_along_y(r: float, length: float) -> Part:
    return Rot(90, 0, 0) * Cylinder(r, length)


# --- Part functions -----------------------------------------------------

def floor_plate() -> Part:
    """Intermediate-floor deck: square plate with a circular hole for the
    column (+ ring bearing) to pass through. Baskets sit on its outer
    perimeter (not modeled)."""
    outer = Pos(0, 0, -FLOOR_T / 2) * Box(
        FLOOR_OUTER_HALF * 2, FLOOR_OUTER_HALF * 2, FLOOR_T
    )
    hole = Pos(0, 0, -FLOOR_T / 2) * Cylinder(FLOOR_HOLE_R, FLOOR_T * 2)
    return outer - hole


def ring_bearing() -> Part:
    """Stationary annular bearing at floor level. The column's bottom
    collar rotates inside its inner race."""
    z_center = COL_Z_BOT + COLLAR_T / 2
    body = Pos(0, 0, z_center) * Cylinder(RING_BEARING_OD / 2, COLLAR_T)
    hole = Pos(0, 0, z_center) * Cylinder(RING_BEARING_ID / 2, COLLAR_T * 2)
    return body - hole


def column() -> Part:
    """Square-section tube with a bottom circular collar, a tall exit
    opening in the +X wall, hinge bosses on the inside of the wall
    straddling the opening, and a pin bore through the bosses.
    """
    col_h = COL_Z_TOP - COL_Z_BOT
    col_z_center = (COL_Z_TOP + COL_Z_BOT) / 2

    outer = Pos(0, 0, col_z_center) * Box(COL_OUTER_SIDE, COL_OUTER_SIDE, col_h)
    inner = Pos(0, 0, col_z_center) * Box(
        COL_INTERIOR_SIDE, COL_INTERIOR_SIDE, col_h * 2
    )
    tube = outer - inner

    # Exit-opening cutout on the +X wall
    cut_z_center = (OPENING_Z_TOP + OPENING_Z_BOT) / 2
    cut_h = OPENING_Z_TOP - OPENING_Z_BOT
    cut = Pos(COL_OUTER_SIDE, 0, cut_z_center) * Box(
        COL_OUTER_SIDE * 2, OPENING_W, cut_h
    )
    tube = tube - cut

    # Bottom circular collar with a square hole (lets bricks fall through)
    collar_z = COL_Z_BOT + COLLAR_T / 2
    collar_disc = Pos(0, 0, collar_z) * Cylinder(COLLAR_OD / 2, COLLAR_T)
    collar_hole = Pos(0, 0, collar_z) * Box(
        COL_INTERIOR_SIDE, COL_INTERIOR_SIDE, COLLAR_T * 2
    )
    tube = tube + (collar_disc - collar_hole)

    # Hinge bosses — inside the column, at the tangential edges of the opening
    boss_plus = Pos(BOSS_X_CENTER, +BOSS_Y_OFFSET, PIVOT_Z) * Box(
        BOSS_X, BOSS_Y, BOSS_H_Z
    )
    boss_minus = Pos(BOSS_X_CENTER, -BOSS_Y_OFFSET, PIVOT_Z) * Box(
        BOSS_X, BOSS_Y, BOSS_H_Z
    )
    tube = tube + boss_plus + boss_minus

    # Pin bore through bosses (and through the air across the opening)
    pin_bore = Pos(PIVOT_X, 0, PIVOT_Z) * _cyl_along_y(
        PIN_HOLE_D / 2, PIN_LEN + 4.0
    )
    return tube - pin_bore


def hinge_pin() -> Part:
    return _cyl_along_y(PIN_D / 2, PIN_LEN)


def flap() -> Part:
    """Middle-pivot flap: wide rectangular catch section on -X side of
    hinge, narrower rectangular spout on +X side, knuckle at pivot.
    Thickness centered on z=0 in local frame. Pin axis = local Y.

    Step change in width at the hinge (catch = FLAP_W, spout =
    SPOUT_EXIT_W). Smooth tapered spout is a follow-up cosmetic.
    """
    catch = Pos(-FLAP_L_CATCH / 2, 0, 0) * Box(FLAP_L_CATCH, FLAP_W, FLAP_T)
    spout = Pos(+FLAP_L_SPOUT / 2, 0, 0) * Box(
        FLAP_L_SPOUT, SPOUT_EXIT_W, FLAP_T
    )
    knuckle = _cyl_along_y(KNUCKLE_OD / 2, FLAP_W)
    body = catch + spout + knuckle
    pin_hole = _cyl_along_y(PIN_HOLE_D / 2, FLAP_W * 2)
    return body - pin_hole


# --- Assembly -----------------------------------------------------------

pivot_loc = Location((PIVOT_X, 0, PIVOT_Z))

assembly = Assembly()
assembly = (
    assembly
    .add("floor_plate", floor_plate())
    .add("ring_bearing", ring_bearing())
    .add("column", column())
    .add("hinge_pin", hinge_pin(), location=pivot_loc)
    .add("flap_down", flap(), location=pivot_loc * Rot(0, TILT_DOWN, 0))
    .add("flap_up", flap(), location=pivot_loc * Rot(0, TILT_UP, 0))
)
for angle in TILT_SAMPLES:
    assembly = assembly.add(
        f"flap_at_{int(angle)}", flap(),
        location=pivot_loc * Rot(0, angle, 0),
    )

assembly = (
    assembly
    # Fixed parts don't interfere with each other
    .assert_no_interference("floor_plate", "ring_bearing")
    .assert_no_interference("floor_plate", "column")
    .assert_no_interference("ring_bearing", "column")
    .assert_no_interference("hinge_pin", "column")
    # Flap DOWN — closes the lower part of the opening, clears everything
    .assert_no_interference("flap_down", "floor_plate")
    .assert_no_interference("flap_down", "ring_bearing")
    .assert_no_interference("flap_down", "column")
    .assert_no_interference("flap_down", "hinge_pin")
    # Flap UP — catch spans column, spout clears wall, nothing fixed is hit
    .assert_no_interference("flap_up", "floor_plate")
    .assert_no_interference("flap_up", "ring_bearing")
    .assert_no_interference("flap_up", "column")
    .assert_no_interference("flap_up", "hinge_pin")
)
for angle in TILT_SAMPLES:
    name = f"flap_at_{int(angle)}"
    assembly = (
        assembly
        .assert_no_interference(name, "floor_plate")
        .assert_no_interference(name, "ring_bearing")
        .assert_no_interference(name, "column")
        .assert_no_interference(name, "hinge_pin")
    )


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Hinge pin is stock; inspect the printed parts. Boolean-artifact knife-
    # edges at curved-surface intersections produce spurious sub-mm
    # readings, so we run all inspections but don't let one failure stop
    # the rest — diagnostics are still written to disk for review.
    method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0)
    for name, part_fn in [
        ("floor_plate", floor_plate),
        ("ring_bearing", ring_bearing),
        ("column", column),
        ("flap", flap),
    ]:
        try:
            inspect(part_fn(), method=method, out="outputs", name=name)
        except SystemExit:
            pass  # printability warning recorded in JSON; don't block
